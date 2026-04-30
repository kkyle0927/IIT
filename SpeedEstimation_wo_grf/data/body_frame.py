"""
Body-aligned functional frame transformation for trunk IMU.

Transforms raw back_imu (accel/gyro/quaternion) into a body-aligned coordinate system:
  +X = right, +Y = forward (walking direction), +Z = up (anti-gravity)

Pipeline:
  1. Quaternion sign canonicalization (hemisphere continuity)
  2. Local -> intermediate (global-like) frame via quaternion rotation
  3. Standing-based +Z (up) estimation from gravity direction
  4. Causal forward direction estimation via sliding-window PCA on horizontal accel
  5. Orthonormal body frame construction (right, forward, up)
  6. Gravity removal + body frame projection
  7. Treadmill acceleration fusion for overground-like forward accel

Ported from Misalign_compensation/model_training.py
"""

import numpy as np


def canonicalize_quaternion_sequence(quat_array):
    """
    Enforce temporal sign continuity for quaternion sequences.
    q and -q encode the same rotation; this keeps consecutive samples in one hemisphere.
    """
    quat = np.asarray(quat_array, dtype=np.float32).copy()
    if quat.ndim != 2 or quat.shape[0] < 2 or quat.shape[1] != 4:
        return quat

    norms = np.linalg.norm(quat, axis=1, keepdims=True)
    norms[norms == 0] = 1.0
    quat /= norms

    for i in range(1, quat.shape[0]):
        if np.dot(quat[i - 1], quat[i]) < 0:
            quat[i] = -quat[i]
    return quat


def normalize_vector(vec, eps=1e-8):
    arr = np.asarray(vec, dtype=np.float32)
    norm = np.linalg.norm(arr)
    if norm < eps:
        return np.zeros_like(arr)
    return arr / norm


def normalize_vectors(vecs, axis=-1, eps=1e-8):
    arr = np.asarray(vecs, dtype=np.float32)
    norms = np.linalg.norm(arr, axis=axis, keepdims=True)
    norms[norms < eps] = 1.0
    return arr / norms


def quat_wxyz_to_rotmat_batch(quat_wxyz):
    """Convert (N, 4) wxyz quaternions to (N, 3, 3) rotation matrices."""
    quat = normalize_vectors(quat_wxyz, axis=1)
    if quat.ndim != 2 or quat.shape[1] != 4:
        return np.zeros((0, 3, 3), dtype=np.float32)

    w, x, y, z = quat[:, 0], quat[:, 1], quat[:, 2], quat[:, 3]
    rot = np.empty((quat.shape[0], 3, 3), dtype=np.float32)
    rot[:, 0, 0] = 1.0 - 2.0 * (y * y + z * z)
    rot[:, 0, 1] = 2.0 * (x * y - z * w)
    rot[:, 0, 2] = 2.0 * (x * z + y * w)
    rot[:, 1, 0] = 2.0 * (x * y + z * w)
    rot[:, 1, 1] = 1.0 - 2.0 * (x * x + z * z)
    rot[:, 1, 2] = 2.0 * (y * z - x * w)
    rot[:, 2, 0] = 2.0 * (x * z - y * w)
    rot[:, 2, 1] = 2.0 * (y * z + x * w)
    rot[:, 2, 2] = 1.0 - 2.0 * (x * x + y * y)
    return rot


def project_to_plane(vec, plane_normal):
    """Project vector onto plane defined by its normal."""
    plane_normal = normalize_vector(plane_normal)
    vec = np.asarray(vec, dtype=np.float32)
    return vec - np.dot(vec, plane_normal) * plane_normal


def _project_to_plane_batch(vecs, plane_normal):
    """Project (N, 3) vectors onto plane defined by its normal. Vectorized."""
    n = normalize_vector(plane_normal)
    dots = vecs @ n  # (N,)
    return vecs - dots[:, None] * n[None, :]


def estimate_body_frame_features(acc_local, gyro_local, quat_wxyz, a_tread, fs):
    """
    Estimate body-aligned IMU features from raw sensor data.

    Args:
        acc_local:  (N, 3) raw accelerometer in sensor frame [m/s^2]
        gyro_local: (N, 3) raw gyroscope in sensor frame [rad/s]
        quat_wxyz:  (N, 4) quaternion [w, x, y, z]
        a_tread:    (N,) treadmill acceleration [m/s^2]
        fs:         sampling frequency [Hz]

    Returns:
        dict with keys:
            acc_body:    (N, 3) body-frame linear accel [right, forward, up]
            gyro_body:   (N, 3) body-frame angular vel [right, forward, up]
            forward_accel_overground: (N, 1) forward accel + treadmill accel
    """
    N = len(acc_local)

    # 1. Quaternion canonicalization
    quat_wxyz = canonicalize_quaternion_sequence(quat_wxyz)
    rot_gi = quat_wxyz_to_rotmat_batch(quat_wxyz)
    if rot_gi.shape[0] == 0:
        raise ValueError("Quaternion sequence is empty or invalid.")

    # 2. Local -> intermediate (global-like) frame
    acc_global = np.einsum('nij,nj->ni', rot_gi, acc_local).astype(np.float32)
    gyro_global = np.einsum('nij,nj->ni', rot_gi, gyro_local).astype(np.float32)

    # 3. Standing reference: estimate +Z (up) from gravity
    ref_len = min(N, max(20, int(fs * 2.0)))
    gyro_ref = np.linalg.norm(gyro_global[:ref_len], axis=1)
    ref_mask = gyro_ref < 0.15
    ref_idx = np.where(ref_mask)[0]
    if len(ref_idx) < max(10, int(0.3 * fs)):
        ref_idx = np.arange(min(ref_len, max(20, int(fs * 1.0))))

    z_up = normalize_vector(np.mean(acc_global[ref_idx], axis=0))
    if np.linalg.norm(z_up) < 1e-6:
        z_up = np.array([0.0, 0.0, 1.0], dtype=np.float32)

    # 4. Forward prior from sensor y-axis projected to horizontal plane
    sensor_y_global = np.einsum('nij,j->ni', rot_gi, np.array([0.0, 1.0, 0.0], dtype=np.float32))
    forward_prior = project_to_plane(np.mean(sensor_y_global[ref_idx], axis=0), z_up)
    if np.linalg.norm(forward_prior) < 1e-6:
        sensor_z_global = np.einsum('nij,j->ni', rot_gi, np.array([0.0, 0.0, 1.0], dtype=np.float32))
        forward_prior = project_to_plane(np.mean(sensor_z_global[ref_idx], axis=0), z_up)
    if np.linalg.norm(forward_prior) < 1e-6:
        forward_prior = project_to_plane(np.array([0.0, 1.0, 0.0], dtype=np.float32), z_up)
    forward_prior = normalize_vector(forward_prior)
    forward_prev = forward_prior.copy()

    # 5. Gravity removal
    g_mag = float(np.mean(np.linalg.norm(acc_global[ref_idx], axis=1)))
    acc_lin_global = acc_global - g_mag * z_up[None, :]

    # 6. Causal walking direction estimation via sliding-window PCA
    #    Optimized: precompute horizontal projections and use cumsum for O(1) covariance
    window = max(20, int(fs * 1.5))
    motion_rms_threshold = 0.15
    gyro_rms_threshold = 0.08
    forward_update_alpha = 0.15
    forward_idle_alpha = 0.02

    # Vectorized horizontal projection
    horiz_all = _project_to_plane_batch(acc_lin_global, z_up)  # (N, 3)

    # Cumulative sums for sliding-window covariance: cov = sum(h_i * h_j) for i,j in {x,y,z}
    # cov[a,b] = cumsum[end+1][a,b] - cumsum[start][a,b]
    # We need 6 unique elements of 3x3 symmetric covariance
    hx, hy, hz = horiz_all[:, 0], horiz_all[:, 1], horiz_all[:, 2]
    cum_xx = np.concatenate([[0.0], np.cumsum(hx * hx)])
    cum_xy = np.concatenate([[0.0], np.cumsum(hx * hy)])
    cum_xz = np.concatenate([[0.0], np.cumsum(hx * hz)])
    cum_yy = np.concatenate([[0.0], np.cumsum(hy * hy)])
    cum_yz = np.concatenate([[0.0], np.cumsum(hy * hz)])
    cum_zz = np.concatenate([[0.0], np.cumsum(hz * hz)])

    # Also cumulative energy for the threshold check
    cum_energy = np.concatenate([[0.0], np.cumsum(hx * hx + hy * hy + hz * hz)])

    body_rot_bg = np.zeros((N, 3, 3), dtype=np.float32)
    cov = np.empty((3, 3), dtype=np.float64)

    for i in range(N):
        start = max(0, i - window + 1)
        win_len = i - start + 1
        end1 = i + 1  # exclusive index into cumsum (shifted by 1)

        horiz_energy = cum_energy[end1] - cum_energy[start]
        horiz_rms = float(np.sqrt(horiz_energy / max(win_len, 1)))
        gyro_rms = float(np.mean(np.linalg.norm(gyro_global[start:end1], axis=1)))
        is_moving = win_len >= 5 and (horiz_rms > motion_rms_threshold or gyro_rms > gyro_rms_threshold)

        if is_moving:
            # Build 3x3 covariance from cumsum differences
            cov[0, 0] = cum_xx[end1] - cum_xx[start]
            cov[0, 1] = cov[1, 0] = cum_xy[end1] - cum_xy[start]
            cov[0, 2] = cov[2, 0] = cum_xz[end1] - cum_xz[start]
            cov[1, 1] = cum_yy[end1] - cum_yy[start]
            cov[1, 2] = cov[2, 1] = cum_yz[end1] - cum_yz[start]
            cov[2, 2] = cum_zz[end1] - cum_zz[start]

            eigvals, eigvecs = np.linalg.eigh(cov)
            forward = eigvecs[:, 2].astype(np.float32)  # largest eigenvalue is last for eigh
            # Project to horizontal plane and normalize
            forward = forward - np.dot(forward, z_up) * z_up
            fn = np.linalg.norm(forward)
            if fn > 1e-6:
                forward /= fn
            else:
                forward = forward_prev
            if np.dot(forward, forward_prev) < 0.0:
                forward = -forward
            forward = normalize_vector((1.0 - forward_update_alpha) * forward_prev + forward_update_alpha * forward)
        else:
            forward = normalize_vector((1.0 - forward_idle_alpha) * forward_prev + forward_idle_alpha * forward_prior)

        # 7. Orthonormal body frame: right, forward, up
        x_right = np.cross(forward, z_up)
        xn = np.linalg.norm(x_right)
        if xn < 1e-6:
            x_right = np.array([1.0, 0.0, 0.0], dtype=np.float32)
        else:
            x_right = (x_right / xn).astype(np.float32)

        y_forward = np.cross(z_up, x_right)
        yn = np.linalg.norm(y_forward)
        if yn > 1e-8:
            y_forward = (y_forward / yn).astype(np.float32)
        if np.dot(y_forward, forward_prev) < 0.0:
            y_forward = -y_forward
            x_right = -x_right

        body_rot_bg[i, :, 0] = x_right
        body_rot_bg[i, :, 1] = y_forward
        body_rot_bg[i, :, 2] = z_up
        forward_prev = y_forward

    # 8. Transform to body frame
    body_rot_gb = np.transpose(body_rot_bg, (0, 2, 1))
    acc_body = np.einsum('nij,nj->ni', body_rot_gb, acc_lin_global).astype(np.float32)
    gyro_body = np.einsum('nij,nj->ni', body_rot_gb, gyro_global).astype(np.float32)

    # 9. Overground-like forward acceleration
    forward_accel_overground = acc_body[:, 1] + a_tread.astype(np.float32)

    return {
        'acc_body': acc_body,
        'gyro_body': gyro_body,
        'forward_accel_overground': forward_accel_overground[:, None].astype(np.float32),
    }
