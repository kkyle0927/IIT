"""
Body-aligned and tilt-only IMU feature builders.
"""

import numpy as np


def canonicalize_quaternion_sequence(quat_array):
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
    plane_normal = normalize_vector(plane_normal)
    vec = np.asarray(vec, dtype=np.float32)
    return vec - np.dot(vec, plane_normal) * plane_normal


def _project_to_plane_batch(vecs, plane_normal):
    n = normalize_vector(plane_normal)
    dots = vecs @ n
    return vecs - dots[:, None] * n[None, :]


def _estimate_reference_up(acc_global, gyro_global, fs):
    n = len(acc_global)
    ref_len = min(n, max(20, int(fs * 2.0)))
    gyro_ref = np.linalg.norm(gyro_global[:ref_len], axis=1)
    ref_mask = gyro_ref < 0.15
    ref_idx = np.where(ref_mask)[0]
    if len(ref_idx) < max(10, int(0.3 * fs)):
        ref_idx = np.arange(min(ref_len, max(20, int(fs * 1.0))))
    z_up = normalize_vector(np.mean(acc_global[ref_idx], axis=0))
    if np.linalg.norm(z_up) < 1e-6:
        z_up = np.array([0.0, 0.0, 1.0], dtype=np.float32)
    return z_up.astype(np.float32), ref_idx


def _reference_gravity_mag(acc_samples, ref_idx):
    g_mag = float(np.mean(np.linalg.norm(acc_samples[ref_idx], axis=1)))
    if not np.isfinite(g_mag) or g_mag < 1e-3:
        g_mag = 9.81
    return g_mag


def estimate_gravity_aligned_imu_features(acc_local, gyro_local, quat_wxyz, fs):
    quat_wxyz = canonicalize_quaternion_sequence(quat_wxyz)
    rot_gi = quat_wxyz_to_rotmat_batch(quat_wxyz)
    if rot_gi.shape[0] == 0:
        raise ValueError("Quaternion sequence is empty or invalid.")
    acc_global = np.einsum("nij,nj->ni", rot_gi, acc_local).astype(np.float32)
    gyro_global = np.einsum("nij,nj->ni", rot_gi, gyro_local).astype(np.float32)
    z_up, ref_idx = _estimate_reference_up(acc_global, gyro_global, fs)
    g_mag = _reference_gravity_mag(acc_global, ref_idx)
    acc_lin_global = acc_global - g_mag * z_up[None, :]
    x_horiz = project_to_plane(np.array([1.0, 0.0, 0.0], dtype=np.float32), z_up)
    if np.linalg.norm(x_horiz) < 1e-6:
        x_horiz = project_to_plane(np.array([0.0, 1.0, 0.0], dtype=np.float32), z_up)
    x_horiz = normalize_vector(x_horiz)
    y_horiz = normalize_vector(np.cross(z_up, x_horiz))
    return {
        "acc_hx": (acc_lin_global @ x_horiz).astype(np.float32)[:, None],
        "acc_hy": (acc_lin_global @ y_horiz).astype(np.float32)[:, None],
        "acc_up": (acc_lin_global @ z_up).astype(np.float32)[:, None],
        "gyro_hx": (gyro_global @ x_horiz).astype(np.float32)[:, None],
        "gyro_hy": (gyro_global @ y_horiz).astype(np.float32)[:, None],
        "gyro_up": (gyro_global @ z_up).astype(np.float32)[:, None],
    }


def estimate_tilt_local_imu_features(acc_local, gyro_local, quat_wxyz, fs):
    quat_wxyz = canonicalize_quaternion_sequence(quat_wxyz)
    rot_gi = quat_wxyz_to_rotmat_batch(quat_wxyz)
    if rot_gi.shape[0] == 0:
        raise ValueError("Quaternion sequence is empty or invalid.")
    acc_global = np.einsum("nij,nj->ni", rot_gi, acc_local).astype(np.float32)
    gyro_global = np.einsum("nij,nj->ni", rot_gi, gyro_local).astype(np.float32)
    z_up_global, ref_idx = _estimate_reference_up(acc_global, gyro_global, fs)
    g_mag = _reference_gravity_mag(acc_global, ref_idx)
    rot_ig = np.transpose(rot_gi, (0, 2, 1))
    up_local = np.einsum("nij,j->ni", rot_ig, z_up_global.astype(np.float32)).astype(np.float32)
    up_local = normalize_vectors(up_local, axis=1)
    x_ref = np.array([1.0, 0.0, 0.0], dtype=np.float32)
    y_ref = np.array([0.0, 1.0, 0.0], dtype=np.float32)
    x_proj = x_ref[None, :] - (up_local @ x_ref)[:, None] * up_local
    x_norm = np.linalg.norm(x_proj, axis=1)
    fallback_mask = x_norm < 1e-6
    if np.any(fallback_mask):
        y_proj = y_ref[None, :] - (up_local @ y_ref)[:, None] * up_local
        x_proj[fallback_mask] = y_proj[fallback_mask]
    x_tilt = normalize_vectors(x_proj, axis=1)
    y_tilt = normalize_vectors(np.cross(up_local, x_tilt), axis=1)
    acc_lin_local = acc_local - g_mag * up_local
    return {
        "acc_hx": np.sum(acc_lin_local * x_tilt, axis=1).astype(np.float32)[:, None],
        "acc_hy": np.sum(acc_lin_local * y_tilt, axis=1).astype(np.float32)[:, None],
        "acc_up": np.sum(acc_lin_local * up_local, axis=1).astype(np.float32)[:, None],
        "gyro_hx": np.sum(gyro_local * x_tilt, axis=1).astype(np.float32)[:, None],
        "gyro_hy": np.sum(gyro_local * y_tilt, axis=1).astype(np.float32)[:, None],
        "gyro_up": np.sum(gyro_local * up_local, axis=1).astype(np.float32)[:, None],
    }


def estimate_tilt_cf_imu_features(
    acc_local,
    gyro_local,
    fs,
    alpha=0.985,
    mode="fixed",
    adaptive_accel_trust=False,
    alpha_min=None,
    acc_trust_width=0.12,
    trust_ema=0.9,
    tvcf_omega_low=0.5,
    tvcf_omega_high=2.5,
    tvcf_bias_gamma=5.0,
    tvcf_sigma_floor_acc_g=0.01,
    tvcf_sigma_floor_gyro_deg=1.0,
    tvcf_sigma_floor_jerk_g=0.5,
):
    acc_local = np.asarray(acc_local, dtype=np.float32)
    gyro_local = np.asarray(gyro_local, dtype=np.float32)
    if acc_local.ndim != 2 or acc_local.shape[1] != 3:
        raise ValueError("acc_local must have shape (N, 3)")
    if gyro_local.ndim != 2 or gyro_local.shape[1] != 3:
        raise ValueError("gyro_local must have shape (N, 3)")

    n = acc_local.shape[0]
    dt = 1.0 / max(float(fs), 1.0)
    gyro_norm = np.linalg.norm(gyro_local, axis=1)
    ref_len = min(n, max(20, int(fs * 2.0)))
    ref_mask = gyro_norm[:ref_len] < 0.15
    ref_idx = np.where(ref_mask)[0]
    if len(ref_idx) < max(10, int(0.3 * fs)):
        ref_idx = np.arange(min(ref_len, max(20, int(fs * 1.0))))
    g_mag = _reference_gravity_mag(acc_local, ref_idx)

    roll = np.zeros(n, dtype=np.float32)
    pitch = np.zeros(n, dtype=np.float32)
    ax0, ay0, az0 = acc_local[0]
    curr_r = float(np.arctan2(ay0, ax0 + 1e-8))
    curr_p = float(np.arctan2(-az0, np.sqrt(ax0 ** 2 + ay0 ** 2) + 1e-8))
    roll[0] = curr_r
    pitch[0] = curr_p

    if alpha_min is None:
        alpha_min = alpha
    alpha_min = float(alpha_min)
    alpha_max = float(alpha)

    def _angles_from_acc(ax, ay, az):
        # Sensor mounting:
        #   +x: head/up, +y: body-right, +z: forward
        roll_acc = float(np.arctan2(ay, ax + 1e-8))
        pitch_acc = float(np.arctan2(-az, np.sqrt(ax ** 2 + ay ** 2) + 1e-8))
        return roll_acc, pitch_acc

    def _alpha_from_omega(omega_c):
        omega_c = max(float(omega_c), 1e-6)
        return float(1.0 / (1.0 + omega_c * dt))

    def _logistic_gate(values, sigma, bias_gamma=1.0):
        sigma = max(float(sigma), 1e-6)
        sigma *= float(bias_gamma)
        x0 = 3.0 * sigma
        slope = np.arctanh(0.8) / sigma
        return 0.5 * (1.0 + np.tanh(slope * (values - x0)))

    trust_state = 1.0
    mu_state = 1.0

    acc_norm = np.linalg.norm(acc_local, axis=1).astype(np.float32)
    x1 = np.abs(acc_norm - g_mag).astype(np.float32)
    jerk_vec = np.zeros_like(acc_local, dtype=np.float32)
    jerk_vec[1:] = (acc_local[1:] - acc_local[:-1]) / dt
    x2 = np.linalg.norm(jerk_vec, axis=1).astype(np.float32) / max(g_mag, 1e-6)
    tilt_gyro_mag = np.sqrt(gyro_local[:, 1] ** 2 + gyro_local[:, 2] ** 2).astype(np.float32)
    x3 = tilt_gyro_mag
    gyro_dot = np.zeros_like(tilt_gyro_mag, dtype=np.float32)
    gyro_dot[1:] = np.abs(tilt_gyro_mag[1:] - tilt_gyro_mag[:-1]) / dt
    x4 = gyro_dot

    sigma1 = max(float(np.std(x1[ref_idx])), tvcf_sigma_floor_acc_g * g_mag)
    sigma2 = max(float(np.std(x2[ref_idx])), tvcf_sigma_floor_jerk_g)
    sigma3 = max(float(np.std(x3[ref_idx])), tvcf_sigma_floor_gyro_deg)
    sigma4 = max(float(np.std(x4[ref_idx])), tvcf_sigma_floor_gyro_deg / max(dt, 1e-6))

    for i in range(1, n):
        ax, ay, az = acc_local[i]
        _, gy, gz = gyro_local[i]
        r_acc, p_acc = _angles_from_acc(ax, ay, az)
        curr_alpha = alpha_max
        if mode == "tvcf":
            x1_bar = _logistic_gate(x1[i], sigma1, bias_gamma=tvcf_bias_gamma)
            x2_bar = _logistic_gate(x2[i], sigma2)
            x3_bar = _logistic_gate(x3[i], sigma3, bias_gamma=tvcf_bias_gamma)
            x4_bar = _logistic_gate(x4[i], sigma4)
            mu_inst = float((1.0 - x1_bar) * (1.0 - x2_bar) * (1.0 - x3_bar) * (1.0 - x4_bar))
            mu_state = trust_ema * mu_state + (1.0 - trust_ema) * mu_inst
            omega_c = mu_state * float(tvcf_omega_high) + (1.0 - mu_state) * float(tvcf_omega_low)
            curr_alpha = _alpha_from_omega(omega_c)
        elif adaptive_accel_trust or mode == "adaptive_accel":
            acc_norm = float(np.sqrt(ax ** 2 + ay ** 2 + az ** 2))
            rel_err = abs(acc_norm - g_mag) / max(g_mag, 1e-6)
            inst_trust = float(np.clip(1.0 - rel_err / max(acc_trust_width, 1e-4), 0.0, 1.0))
            trust_state = trust_ema * trust_state + (1.0 - trust_ema) * inst_trust
            curr_alpha = alpha_max - (alpha_max - alpha_min) * trust_state
        curr_r = curr_alpha * (curr_r + float(gz) * dt) + (1.0 - curr_alpha) * r_acc
        curr_p = curr_alpha * (curr_p + float(gy) * dt) + (1.0 - curr_alpha) * p_acc
        roll[i] = curr_r
        pitch[i] = curr_p

    up_local = np.stack(
        [np.cos(roll) * np.cos(pitch), np.sin(roll) * np.cos(pitch), -np.sin(pitch)],
        axis=1,
    ).astype(np.float32)
    up_local = normalize_vectors(up_local, axis=1)
    x_ref = np.array([1.0, 0.0, 0.0], dtype=np.float32)
    y_ref = np.array([0.0, 1.0, 0.0], dtype=np.float32)
    x_proj = x_ref[None, :] - (up_local @ x_ref)[:, None] * up_local
    x_norm = np.linalg.norm(x_proj, axis=1)
    fallback_mask = x_norm < 1e-6
    if np.any(fallback_mask):
        y_proj = y_ref[None, :] - (up_local @ y_ref)[:, None] * up_local
        x_proj[fallback_mask] = y_proj[fallback_mask]
    x_tilt = normalize_vectors(x_proj, axis=1)
    y_tilt = normalize_vectors(np.cross(up_local, x_tilt), axis=1)
    acc_lin_local = acc_local - g_mag * up_local
    return {
        "acc_hx": np.sum(acc_lin_local * x_tilt, axis=1).astype(np.float32)[:, None],
        "acc_hy": np.sum(acc_lin_local * y_tilt, axis=1).astype(np.float32)[:, None],
        "acc_up": np.sum(acc_lin_local * up_local, axis=1).astype(np.float32)[:, None],
        "gyro_hx": np.sum(gyro_local * x_tilt, axis=1).astype(np.float32)[:, None],
        "gyro_hy": np.sum(gyro_local * y_tilt, axis=1).astype(np.float32)[:, None],
        "gyro_up": np.sum(gyro_local * up_local, axis=1).astype(np.float32)[:, None],
        "roll_cf": roll[:, None],
        "pitch_cf": pitch[:, None],
    }


def estimate_body_frame_features(
    acc_local,
    gyro_local,
    quat_wxyz,
    a_tread,
    fs,
    heading_mode="pca",
    stationary_update_alpha=0.2,
):
    n = len(acc_local)
    quat_wxyz = canonicalize_quaternion_sequence(quat_wxyz)
    rot_gi = quat_wxyz_to_rotmat_batch(quat_wxyz)
    if rot_gi.shape[0] == 0:
        raise ValueError("Quaternion sequence is empty or invalid.")
    acc_global = np.einsum("nij,nj->ni", rot_gi, acc_local).astype(np.float32)
    gyro_global = np.einsum("nij,nj->ni", rot_gi, gyro_local).astype(np.float32)
    z_up, ref_idx = _estimate_reference_up(acc_global, gyro_global, fs)
    g_mag = _reference_gravity_mag(acc_global, ref_idx)
    acc_lin_global = acc_global - g_mag * z_up[None, :]

    sensor_y_global = np.einsum("nij,j->ni", rot_gi, np.array([0.0, 1.0, 0.0], dtype=np.float32))
    forward_prior = project_to_plane(np.mean(sensor_y_global[ref_idx], axis=0), z_up)
    if np.linalg.norm(forward_prior) < 1e-6:
        sensor_z_global = np.einsum("nij,j->ni", rot_gi, np.array([0.0, 0.0, 1.0], dtype=np.float32))
        forward_prior = project_to_plane(np.mean(sensor_z_global[ref_idx], axis=0), z_up)
    if np.linalg.norm(forward_prior) < 1e-6:
        forward_prior = project_to_plane(np.array([0.0, 1.0, 0.0], dtype=np.float32), z_up)
    forward_prior = normalize_vector(forward_prior)

    zonly_prior = project_to_plane(np.array([1.0, 0.0, 0.0], dtype=np.float32), z_up)
    if np.linalg.norm(zonly_prior) < 1e-6:
        zonly_prior = project_to_plane(np.array([0.0, 1.0, 0.0], dtype=np.float32), z_up)
    zonly_prior = normalize_vector(zonly_prior)
    forward_prev = zonly_prior.copy() if heading_mode in {"pca_zonly", "pca_gated_zonly"} else forward_prior.copy()

    window = max(20, int(fs * 1.5))
    motion_rms_threshold = 0.15
    gyro_rms_threshold = 0.08
    forward_update_alpha = 0.15
    forward_idle_alpha = 0.02
    gated_ratio = 1.15
    horiz_all = _project_to_plane_batch(acc_lin_global, z_up)
    sensor_y_horiz_all = normalize_vectors(_project_to_plane_batch(sensor_y_global, z_up), axis=1)

    hx, hy, hz = horiz_all[:, 0], horiz_all[:, 1], horiz_all[:, 2]
    cum_xx = np.concatenate([[0.0], np.cumsum(hx * hx)])
    cum_xy = np.concatenate([[0.0], np.cumsum(hx * hy)])
    cum_xz = np.concatenate([[0.0], np.cumsum(hx * hz)])
    cum_yy = np.concatenate([[0.0], np.cumsum(hy * hy)])
    cum_yz = np.concatenate([[0.0], np.cumsum(hy * hz)])
    cum_zz = np.concatenate([[0.0], np.cumsum(hz * hz)])
    cum_energy = np.concatenate([[0.0], np.cumsum(hx * hx + hy * hy + hz * hz)])

    body_rot_bg = np.zeros((n, 3, 3), dtype=np.float32)
    cov = np.empty((3, 3), dtype=np.float64)
    for i in range(n):
        start = max(0, i - window + 1)
        win_len = i - start + 1
        end1 = i + 1
        horiz_energy = cum_energy[end1] - cum_energy[start]
        horiz_rms = float(np.sqrt(horiz_energy / max(win_len, 1)))
        gyro_rms = float(np.mean(np.linalg.norm(gyro_global[start:end1], axis=1)))
        is_moving = win_len >= 5 and (horiz_rms > motion_rms_threshold or gyro_rms > gyro_rms_threshold)
        is_stationary = win_len >= 5 and (horiz_rms <= motion_rms_threshold and gyro_rms <= gyro_rms_threshold)

        if heading_mode in {"pca", "pca_zonly", "pca_gated_zonly"}:
            if is_moving:
                cov[0, 0] = cum_xx[end1] - cum_xx[start]
                cov[0, 1] = cov[1, 0] = cum_xy[end1] - cum_xy[start]
                cov[0, 2] = cov[2, 0] = cum_xz[end1] - cum_xz[start]
                cov[1, 1] = cum_yy[end1] - cum_yy[start]
                cov[1, 2] = cov[2, 1] = cum_yz[end1] - cum_yz[start]
                cov[2, 2] = cum_zz[end1] - cum_zz[start]
                eigvals, eigvecs = np.linalg.eigh(cov)
                candidate = eigvecs[:, 2].astype(np.float32)
                candidate = candidate - np.dot(candidate, z_up) * z_up
                cn = np.linalg.norm(candidate)
                candidate = candidate / cn if cn > 1e-6 else forward_prev
                if np.dot(candidate, forward_prev) < 0.0:
                    candidate = -candidate
                update_ok = True
                if heading_mode == "pca_gated_zonly":
                    ratio = float(eigvals[2] / max(eigvals[1], 1e-8))
                    update_ok = ratio >= gated_ratio
                if update_ok:
                    forward = normalize_vector((1.0 - forward_update_alpha) * forward_prev + forward_update_alpha * candidate)
                else:
                    forward = forward_prev
            else:
                idle_prior = zonly_prior if heading_mode in {"pca_zonly", "pca_gated_zonly"} else forward_prior
                forward = normalize_vector((1.0 - forward_idle_alpha) * forward_prev + forward_idle_alpha * idle_prior)
        elif heading_mode == "fixed":
            forward = forward_prev
        elif heading_mode == "stationary_gated":
            if is_stationary:
                candidate = sensor_y_horiz_all[i].astype(np.float32)
                if np.linalg.norm(candidate) < 1e-6:
                    candidate = forward_prev
                if np.dot(candidate, forward_prev) < 0.0:
                    candidate = -candidate
                forward_prev = normalize_vector((1.0 - stationary_update_alpha) * forward_prev + stationary_update_alpha * candidate)
            forward = forward_prev
        else:
            raise ValueError(f"Unknown heading_mode: {heading_mode}")

        x_right = np.cross(forward, z_up)
        xn = np.linalg.norm(x_right)
        x_right = np.array([1.0, 0.0, 0.0], dtype=np.float32) if xn < 1e-6 else (x_right / xn).astype(np.float32)
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

    body_rot_gb = np.transpose(body_rot_bg, (0, 2, 1))
    acc_body = np.einsum("nij,nj->ni", body_rot_gb, acc_lin_global).astype(np.float32)
    gyro_body = np.einsum("nij,nj->ni", body_rot_gb, gyro_global).astype(np.float32)
    forward_accel_overground = acc_body[:, 1] + a_tread.astype(np.float32)
    return {
        "acc_body": acc_body,
        "gyro_body": gyro_body,
        "forward_accel_overground": forward_accel_overground[:, None].astype(np.float32),
    }
