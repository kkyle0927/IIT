import os
import time as _time
import numpy as np
import h5py
from scipy.signal import butter, sosfilt

from .utils import (
    fs, butter_lowpass_filter, butter_highpass_filter,
    get_ground_contact, get_data_from_group,
)
from .body_frame import estimate_body_frame_features
from .model_com_speed import (
    compute_model_com_speed,
    compute_model_com_speed_nogrf,
    compute_imu_integrated_speed,
    compute_contact_probability_phase,
    compute_contact_probability_kinematics,
    compute_contact_probability_fusion,
)

# ---------------------------------------------------------------------------
# Trial-level dataset cache (avoids re-reading H5 + re-computing derived
# features across LOSO folds within the same job)
# ---------------------------------------------------------------------------
TRIAL_DATASET_CACHE = {}
DATASET_METADATA_REGISTRY = {}


def freeze_nested(value):
    """Convert nested dicts/lists to immutable tuples for use as cache keys."""
    if isinstance(value, dict):
        return tuple(sorted((k, freeze_nested(v)) for k, v in value.items()))
    if isinstance(value, (list, tuple)):
        return tuple(freeze_nested(v) for v in value)
    return value


def register_dataset_metadata(dataset_list, metadata_list):
    """Register per-trial metadata for a returned dataset list."""
    DATASET_METADATA_REGISTRY[id(dataset_list)] = list(metadata_list)


def get_dataset_metadata(dataset_list):
    """Fetch per-trial metadata previously registered for a dataset list."""
    return DATASET_METADATA_REGISTRY.get(id(dataset_list))

# ---------------------------------------------------------------------------
# Body-aligned IMU features (derived/body_frame_imu)
# ---------------------------------------------------------------------------
_BODY_FRAME_IMU_VARS = frozenset([
    'accel_right', 'accel_forward', 'accel_up',
    'gyro_right', 'gyro_forward', 'gyro_up',
    'forward_accel_overground',
])

# ---------------------------------------------------------------------------
# Model-based COM speed (derived/model_com_speed)
# ---------------------------------------------------------------------------
_MODEL_COM_SPEED_VARS = frozenset(['model_com_speed'])

# ---------------------------------------------------------------------------
# GRF-free model-based COM speed variants
# ---------------------------------------------------------------------------
_MODEL_COM_SPEED_NOGRF_VARS = frozenset([
    'model_com_speed_nogrf_phase',
    'model_com_speed_nogrf_kin',
    'model_com_speed_nogrf_fusion',
])

# ---------------------------------------------------------------------------
# IMU integrated speed (derived/imu_integrated_speed)
# ---------------------------------------------------------------------------
_IMU_INTEGRATED_SPEED_VARS = frozenset(['imu_integrated_speed'])

# ---------------------------------------------------------------------------
# Treadmill-independent causal IMU speed prior
# ---------------------------------------------------------------------------
_IMU_CAUSAL_SPEED_VARS = frozenset(['imu_speed_causal'])

# ---------------------------------------------------------------------------
# Robot stride/support progression prior
# ---------------------------------------------------------------------------
_STRIDE_PROGRESSION_VARS = frozenset(['stride_speed_robot'])

# ---------------------------------------------------------------------------
# Human-robot interaction power/work features
# ---------------------------------------------------------------------------
_INTERACTION_POWER_VARS = frozenset([
    'interaction_power_l',
    'interaction_power_r',
    'interaction_work_pos',
    'interaction_work_neg',
])

# ---------------------------------------------------------------------------
# GRF-free contact/support estimates
# ---------------------------------------------------------------------------
_CONTACT_ESTIMATE_VARS = frozenset([
    'contact_prob_l_phase', 'contact_prob_r_phase',
    'contact_prob_l_kin', 'contact_prob_r_kin',
    'contact_prob_l_fusion', 'contact_prob_r_fusion',
])

# ---------------------------------------------------------------------------
# Global IMU Transformation helpers
# ---------------------------------------------------------------------------
_GLOBAL_IMU_VARS = frozenset([
    'global_acc_x', 'global_acc_y', 'global_acc_z',
    'global_gyr_x', 'global_gyr_y', 'global_gyr_z',
    'global_vel_y', 'global_vel_treadmill',
])


def _quat_to_rotmat(q_wxyz: np.ndarray) -> np.ndarray:
    """(4,) wxyz → (3,3) rotation matrix"""
    w, x, y, z = q_wxyz
    return np.array([
        [1 - 2*(y*y + z*z),   2*(x*y - z*w),   2*(x*z + y*w)],
        [  2*(x*y + z*w), 1 - 2*(x*x + z*z),   2*(y*z - x*w)],
        [  2*(x*z - y*w),   2*(y*z + x*w), 1 - 2*(x*x + y*y)],
    ])


def compute_global_imu(accel: np.ndarray, gyro: np.ndarray,
                       quat_wxyz: np.ndarray, calib_n: int = 100):
    """Quaternion-based global frame (X=right, Y=forward, Z=up). Gravity removed from acc.
    Fully vectorized — no Python loops over samples."""
    q_ref = np.mean(quat_wxyz[:calib_n], axis=0)
    q_ref /= (np.linalg.norm(q_ref) + 1e-12)
    R_ref_inv = _quat_to_rotmat(q_ref).T          # (3, 3)
    g_global = np.array([0.0, 0.0, 9.81])

    # Normalize all quaternions at once: (N, 4)
    norms = np.linalg.norm(quat_wxyz, axis=1, keepdims=True) + 1e-12
    q = quat_wxyz / norms
    w, x, y, z = q[:, 0], q[:, 1], q[:, 2], q[:, 3]

    # Build (N, 3, 3) rotation matrices via broadcasting
    R = np.stack([
        np.stack([1 - 2*(y*y + z*z),   2*(x*y - z*w),   2*(x*z + y*w)], axis=1),
        np.stack([  2*(x*y + z*w), 1 - 2*(x*x + z*z),   2*(y*z - x*w)], axis=1),
        np.stack([  2*(x*z - y*w),   2*(y*z + x*w), 1 - 2*(x*x + y*y)], axis=1),
    ], axis=1)  # (N, 3, 3)

    # R_rel = R @ R_ref_inv  →  (N, 3, 3) @ (3, 3)
    R_rel = R @ R_ref_inv   # (N, 3, 3)

    # Rotate vectors: (N, 3, 3) @ (N, 3, 1) → squeeze to (N, 3)
    acc_g = (R_rel @ accel[:, :, None])[:, :, 0] - g_global
    gyr_g = (R_rel @ gyro[:, :, None])[:, :, 0]
    return acc_g, gyr_g


def compute_gait_phase(grf_fz: np.ndarray, threshold: float = 20.0) -> np.ndarray:
    """GRF Fz → Gait Phase [0, 1). Heel strike = rising edge of contact signal."""
    contact = (np.nan_to_num(grf_fz.flatten()) > threshold).astype(float)
    pad = np.pad(contact, (1, 0), constant_values=0)
    strikes = np.where(np.diff(pad) > 0)[0]
    N = len(contact)
    phase = np.zeros(N)
    for i in range(len(strikes) - 1):
        s0, s1 = strikes[i], strikes[i + 1]
        phase[s0:s1] = np.linspace(0.0, 1.0, s1 - s0, endpoint=False)
    return phase


def compute_stride_frequency(grf_fz: np.ndarray, fs: float = 100.0,
                             threshold: float = 20.0) -> np.ndarray:
    """GRF Fz → real-time stride frequency (Hz). Inverse of heel-strike interval."""
    contact = (np.nan_to_num(grf_fz.flatten()) > threshold).astype(float)
    pad = np.pad(contact, (1, 0), constant_values=0)
    strikes = np.where(np.diff(pad) > 0)[0]
    N = len(grf_fz.flatten())
    freq = np.zeros(N)
    for i in range(len(strikes) - 1):
        s0, s1 = strikes[i], strikes[i + 1]
        f = fs / (s1 - s0)
        freq[s0:s1] = f
    if len(strikes) > 0:
        freq[strikes[-1]:] = freq[max(0, strikes[-1] - 1)]
    return freq


# ---------------------------------------------------------------------------
# Biomechanical Feature Helpers (archive_round5)
# ---------------------------------------------------------------------------

# Anthropometric ratios (Winter, 1990 — % of leg length)
_THIGH_RATIO = 0.530   # thigh ≈ 53% of leg length
_SHANK_RATIO = 0.470   # shank ≈ 47% of leg length

# New derived variable names recognised in extract_condition_data_v2
_CHAIN_VELOCITY_VARS = frozenset([
    'com_velocity_chain',        # combined COM forward velocity from kinematic chain
    'com_velocity_chain_v2',     # v2: absolute segment angles, relaxed filtering
    'segment_vel_ankle',         # ankle segment velocity contribution
    'segment_vel_knee',          # knee segment velocity contribution
    'segment_vel_hip',           # hip/pelvis segment velocity contribution
])

_BIOMECH_LEADING_VARS = frozenset([
    'grf_impulse_l', 'grf_impulse_r',       # anterior-posterior GRF impulse per stance
    'stance_ratio_l', 'stance_ratio_r',      # stance phase duty factor
    'cop_vel_y_l', 'cop_vel_y_r',            # CoP forward velocity (from GRF moments)
    'trunk_pitch_dot',                        # trunk sagittal angular velocity (leading)
])


def compute_com_chain_velocity(
    ankle_l, ankle_r, knee_l, knee_r, hip_l, hip_r,
    pelvis_sag, pelvis_rot,
    contact_l, contact_r,
    leg_length_l_m, leg_length_r_m,
    fs_hz=100.0
):
    """
    Kinematic-chain based COM forward velocity in the sagittal plane.

    Model (stance leg, sagittal plane, from ground up):
      v_ankle→knee  = shank_L * dθ_ankle/dt * cos(θ_ankle)
      v_knee→hip    = thigh_L * dθ_knee/dt  * cos(θ_knee)
      v_hip→com     = pelvis_half * dθ_pelvis_sag/dt * cos(θ_pelvis_sag)
                    + pelvis_half * dθ_pelvis_rot/dt * sin(θ_pelvis_sag)  (minor)
      v_com_chain   = -(v_ankle + v_knee + v_hip)  for stance leg

    Sign convention:
      - Angles in DEGREES (converted to rad internally)
      - Positive hip_flexion = flexion (thigh forward)
      - Positive knee_angle  = flexion (shank back)
      - Positive ankle_angle = dorsiflexion (foot up)
      - pelvis_sag (pelvis_list): negative = posterior tilt (observed ~-8° at rest)
        → forward lean during gait → contributes to forward velocity
      - pelvis_rot (pelvis_rotation): ~-90° lab offset, oscillation = pelvic rotation
        → detrended rotation used; positive rotation of LEFT forward

    The negative sign on the total velocity accounts for the stance leg being
    behind the COM during propulsion (the stance foot is fixed on the treadmill
    belt, so positive angular velocities of the joints contribute to moving the
    COM forward relative to the foot).

    Returns:
        com_vel_chain (N,): estimated COM forward velocity (m/s positive = forward)
        seg_ankle (N,): ankle segment contribution
        seg_knee (N,): knee segment contribution
        seg_hip (N,): hip/pelvis segment contribution
    """
    deg2rad = np.pi / 180.0
    N = len(ankle_l)

    # Segment lengths from leg length
    shank_L_l = leg_length_l_m * _SHANK_RATIO
    thigh_L_l = leg_length_l_m * _THIGH_RATIO
    shank_L_r = leg_length_r_m * _SHANK_RATIO
    thigh_L_r = leg_length_r_m * _THIGH_RATIO

    # Approximate pelvis half-width: ~0.16 * leg_length (rough estimate)
    pelvis_half = (leg_length_l_m + leg_length_r_m) / 2.0 * 0.16

    # Convert to radians
    ank_l_rad = ankle_l * deg2rad
    ank_r_rad = ankle_r * deg2rad
    knee_l_rad = knee_l * deg2rad
    knee_r_rad = knee_r * deg2rad
    hip_l_rad = hip_l * deg2rad
    hip_r_rad = hip_r * deg2rad
    pelvis_sag_rad = pelvis_sag * deg2rad
    pelvis_rot_rad = (pelvis_rot - np.mean(pelvis_rot)) * deg2rad  # remove lab offset

    # Angular velocities (rad/s)
    dank_l = np.gradient(ank_l_rad) * fs_hz
    dank_r = np.gradient(ank_r_rad) * fs_hz
    dknee_l = np.gradient(knee_l_rad) * fs_hz
    dknee_r = np.gradient(knee_r_rad) * fs_hz
    dhip_l = np.gradient(hip_l_rad) * fs_hz
    dhip_r = np.gradient(hip_r_rad) * fs_hz
    dpelvis_sag = np.gradient(pelvis_sag_rad) * fs_hz
    dpelvis_rot = np.gradient(pelvis_rot_rad) * fs_hz

    # Smooth angular velocities to avoid noise amplification
    for arr in [dank_l, dank_r, dknee_l, dknee_r, dhip_l, dhip_r, dpelvis_sag, dpelvis_rot]:
        arr[:] = butter_lowpass_filter(arr, 10.0, fs_hz, 2)

    # Segment velocities (LEFT stance)
    # Ankle contribution: shank rotates about the ankle joint
    v_ank_l = shank_L_l * dank_l * np.cos(ank_l_rad)
    v_knee_l = thigh_L_l * dknee_l * np.cos(knee_l_rad)
    v_hip_l = pelvis_half * dpelvis_sag * np.cos(pelvis_sag_rad)

    # RIGHT stance
    v_ank_r = shank_L_r * dank_r * np.cos(ank_r_rad)
    v_knee_r = thigh_L_r * dknee_r * np.cos(knee_r_rad)
    v_hip_r = pelvis_half * dpelvis_sag * np.cos(pelvis_sag_rad)

    # Blend based on contact signals (stance leg contributes)
    cl = contact_l.flatten().astype(float)
    cr = contact_r.flatten().astype(float)

    # During double stance, average both legs; during single stance, use stance leg
    weight_l = cl / (cl + cr + 1e-8)
    weight_r = cr / (cl + cr + 1e-8)

    seg_ankle = -(weight_l * v_ank_l + weight_r * v_ank_r)
    seg_knee  = -(weight_l * v_knee_l + weight_r * v_knee_r)
    seg_hip   = -(weight_l * v_hip_l + weight_r * v_hip_r)

    com_vel_chain = seg_ankle + seg_knee + seg_hip

    # Add pelvis rotation contribution (transverse plane → forward velocity)
    # When pelvis rotates to advance left leg, the COM moves forward
    rot_contrib = pelvis_half * dpelvis_rot * np.cos(pelvis_rot_rad)
    com_vel_chain += rot_contrib

    # Low-pass filter the final output to reduce numerical noise
    com_vel_chain = butter_lowpass_filter(com_vel_chain, 6.0, fs_hz, 2)

    return com_vel_chain, seg_ankle, seg_knee, seg_hip


def compute_com_chain_velocity_v2(
    ankle_l, ankle_r, knee_l, knee_r, hip_l, hip_r,
    pelvis_sag, pelvis_rot,
    contact_l, contact_r,
    leg_length_l_m, leg_length_r_m,
    fs_hz=100.0
):
    """
    COM chain velocity v2 — improvements over v1:
    1. Uses absolute segment angles (cumulative from ground up) instead of local joint angles
    2. Relaxed final LPF: 15Hz instead of 6Hz to preserve gait dynamics
    3. Angular velocity LPF: 15Hz instead of 10Hz

    Absolute segment angle computation:
      θ_shank_abs = θ_ankle  (shank angle from vertical ≈ ankle angle)
      θ_thigh_abs = θ_ankle + θ_knee  (accumulated upward)
      θ_pelvis_abs = θ_ankle + θ_knee + θ_hip  (full chain)
    """
    deg2rad = np.pi / 180.0
    N = len(ankle_l)

    shank_L_l = leg_length_l_m * _SHANK_RATIO
    thigh_L_l = leg_length_l_m * _THIGH_RATIO
    shank_L_r = leg_length_r_m * _SHANK_RATIO
    thigh_L_r = leg_length_r_m * _THIGH_RATIO
    pelvis_half = (leg_length_l_m + leg_length_r_m) / 2.0 * 0.16

    # Convert to radians
    ank_l_rad = ankle_l * deg2rad
    ank_r_rad = ankle_r * deg2rad
    knee_l_rad = knee_l * deg2rad
    knee_r_rad = knee_r * deg2rad
    hip_l_rad = hip_l * deg2rad
    hip_r_rad = hip_r * deg2rad
    pelvis_sag_rad = pelvis_sag * deg2rad
    pelvis_rot_rad = (pelvis_rot - np.mean(pelvis_rot)) * deg2rad

    # Absolute segment angles from vertical (cumulative chain)
    shank_abs_l = ank_l_rad
    thigh_abs_l = ank_l_rad + knee_l_rad
    pelvis_abs_l = ank_l_rad + knee_l_rad + hip_l_rad

    shank_abs_r = ank_r_rad
    thigh_abs_r = ank_r_rad + knee_r_rad
    pelvis_abs_r = ank_r_rad + knee_r_rad + hip_r_rad

    # Angular velocities of each segment (absolute)
    d_shank_l = np.gradient(shank_abs_l) * fs_hz
    d_thigh_l = np.gradient(thigh_abs_l) * fs_hz
    d_pelvis_l = np.gradient(pelvis_abs_l) * fs_hz

    d_shank_r = np.gradient(shank_abs_r) * fs_hz
    d_thigh_r = np.gradient(thigh_abs_r) * fs_hz
    d_pelvis_r = np.gradient(pelvis_abs_r) * fs_hz

    dpelvis_sag = np.gradient(pelvis_sag_rad) * fs_hz
    dpelvis_rot = np.gradient(pelvis_rot_rad) * fs_hz

    # Relaxed LPF on angular velocities: 15Hz instead of 10Hz
    for arr in [d_shank_l, d_thigh_l, d_pelvis_l, d_shank_r, d_thigh_r, d_pelvis_r,
                dpelvis_sag, dpelvis_rot]:
        arr[:] = butter_lowpass_filter(arr, 15.0, fs_hz, 2)

    # Segment velocities using absolute angles for cos projection
    v_shank_l = shank_L_l * d_shank_l * np.cos(shank_abs_l)
    v_thigh_l = thigh_L_l * d_thigh_l * np.cos(thigh_abs_l)
    v_pelvis_l = pelvis_half * dpelvis_sag * np.cos(pelvis_sag_rad)

    v_shank_r = shank_L_r * d_shank_r * np.cos(shank_abs_r)
    v_thigh_r = thigh_L_r * d_thigh_r * np.cos(thigh_abs_r)
    v_pelvis_r = pelvis_half * dpelvis_sag * np.cos(pelvis_sag_rad)

    # Blend based on contact
    cl = contact_l.flatten().astype(float)
    cr = contact_r.flatten().astype(float)
    weight_l = cl / (cl + cr + 1e-8)
    weight_r = cr / (cl + cr + 1e-8)

    seg_ankle = -(weight_l * v_shank_l + weight_r * v_shank_r)
    seg_knee  = -(weight_l * v_thigh_l + weight_r * v_thigh_r)
    seg_hip   = -(weight_l * v_pelvis_l + weight_r * v_pelvis_r)

    com_vel_chain = seg_ankle + seg_knee + seg_hip

    # Pelvis rotation contribution
    rot_contrib = pelvis_half * dpelvis_rot * np.cos(pelvis_rot_rad)
    com_vel_chain += rot_contrib

    # Relaxed final LPF: 15Hz instead of 6Hz
    com_vel_chain = butter_lowpass_filter(com_vel_chain, 15.0, fs_hz, 2)

    return com_vel_chain, seg_ankle, seg_knee, seg_hip


def compute_stance_ratio(grf_fz: np.ndarray, fs_hz: float = 100.0,
                         threshold: float = 20.0) -> np.ndarray:
    """
    GRF Fz → real-time stance duty factor (0~1).
    stance_ratio = stance_duration / stride_duration for the current stride.
    Higher speed → lower stance ratio (shorter stance, longer swing).
    """
    contact = (np.nan_to_num(grf_fz.flatten()) > threshold).astype(float)
    N = len(contact)
    pad = np.pad(contact, (1, 0), constant_values=0)
    strikes = np.where(np.diff(pad) > 0)[0]

    ratio = np.full(N, 0.5)  # default
    for i in range(len(strikes) - 1):
        s0, s1 = strikes[i], strikes[i + 1]
        stride_len = s1 - s0
        stance_len = int(np.sum(contact[s0:s1]))
        r = stance_len / stride_len if stride_len > 0 else 0.5
        ratio[s0:s1] = r
    if len(strikes) > 0:
        ratio[strikes[-1]:] = ratio[max(0, strikes[-1] - 1)]
    return ratio


def compute_grf_impulse_per_stance(grf_ap: np.ndarray, grf_fz: np.ndarray,
                                    fs_hz: float = 100.0,
                                    threshold: float = 20.0) -> np.ndarray:
    """
    Compute cumulative AP impulse within each stance phase.
    Reset at heel-strike. Positive = propulsion, Negative = braking.
    The final value at toe-off represents net propulsion/braking impulse.
    """
    contact = (np.nan_to_num(grf_fz.flatten()) > threshold).astype(float)
    ap = np.nan_to_num(grf_ap.flatten())
    N = len(ap)
    dt = 1.0 / fs_hz

    pad = np.pad(contact, (1, 0), constant_values=0)
    strikes = np.where(np.diff(pad) > 0)[0]

    impulse = np.zeros(N)
    cum = 0.0
    prev_contact = 0.0
    strike_idx = 0

    for i in range(N):
        if contact[i] > 0.5:
            if prev_contact < 0.5:
                cum = 0.0  # reset at heel-strike
            cum += ap[i] * dt
            impulse[i] = cum
        else:
            impulse[i] = cum  # hold last value during swing
        prev_contact = contact[i]

    return impulse


def _causal_lpf_1d(signal_1d, cutoff_hz, fs_hz, order=4):
    x = np.asarray(signal_1d, dtype=np.float32).flatten()
    sos = butter(order, cutoff_hz, btype='low', fs=fs_hz, output='sos')
    return sosfilt(sos, x).astype(np.float32)


def _causal_diff_1d(signal_1d, fs_hz):
    x = np.asarray(signal_1d, dtype=np.float32).flatten()
    dx = np.zeros_like(x, dtype=np.float32)
    if len(x) > 1:
        dx[1:] = np.diff(x) * float(fs_hz)
    return dx


def compute_robot_stride_speed(hip_angle_l, hip_angle_r, height_m, fs_hz=100.0,
                               cadence_cutoff=1.5, excursion_cutoff=1.0,
                               speed_cutoff=0.5, speed_order=4):
    """
    Causal stride progression prior from exoskeleton left/right hip angles.

    Uses only deployable robot encoder signals and subject height.
    The proxy is intentionally low-frequency so it aligns with the corrected
    progression-speed target rather than high-frequency joint motion.
    """
    hip_l = np.deg2rad(np.asarray(hip_angle_l, dtype=np.float32).flatten())
    hip_r = np.deg2rad(np.asarray(hip_angle_r, dtype=np.float32).flatten())
    phase_diff = hip_l - hip_r

    excursion = _causal_lpf_1d(np.abs(phase_diff), excursion_cutoff, fs_hz, order=2)
    cadence_proxy = _causal_lpf_1d(np.abs(_causal_diff_1d(phase_diff, fs_hz)), cadence_cutoff, fs_hz, order=2)

    leg_length_m = max(float(height_m) * 0.53, 0.5)
    step_length_proxy = 0.5 * leg_length_m * excursion
    stride_speed = step_length_proxy * cadence_proxy
    stride_speed = _causal_lpf_1d(stride_speed, speed_cutoff, fs_hz, order=speed_order)
    return stride_speed.astype(np.float32)


def compute_interaction_power_features(hip_angle_l, hip_angle_r, torque_l, torque_r,
                                       fs_hz=100.0, power_cutoff=2.0, work_tau_s=0.75):
    """
    Causal interaction mechanics from robot hip encoders and torque.

    Returns instantaneous power per side and exponentially weighted
    positive/negative work.
    """
    hip_l = np.deg2rad(np.asarray(hip_angle_l, dtype=np.float32).flatten())
    hip_r = np.deg2rad(np.asarray(hip_angle_r, dtype=np.float32).flatten())
    tq_l = np.asarray(torque_l, dtype=np.float32).flatten()
    tq_r = np.asarray(torque_r, dtype=np.float32).flatten()

    omega_l = _causal_lpf_1d(_causal_diff_1d(hip_l, fs_hz), 6.0, fs_hz, order=2)
    omega_r = _causal_lpf_1d(_causal_diff_1d(hip_r, fs_hz), 6.0, fs_hz, order=2)

    power_l = _causal_lpf_1d(tq_l * omega_l, power_cutoff, fs_hz, order=2)
    power_r = _causal_lpf_1d(tq_r * omega_r, power_cutoff, fs_hz, order=2)

    dt = 1.0 / float(fs_hz)
    alpha = float(np.exp(-dt / max(work_tau_s, 1e-3)))
    work_pos = np.zeros_like(power_l, dtype=np.float32)
    work_neg = np.zeros_like(power_l, dtype=np.float32)
    for i in range(1, len(power_l)):
        pos_i = max(power_l[i], 0.0) + max(power_r[i], 0.0)
        neg_i = max(-power_l[i], 0.0) + max(-power_r[i], 0.0)
        work_pos[i] = alpha * work_pos[i - 1] + pos_i * dt
        work_neg[i] = alpha * work_neg[i - 1] + neg_i * dt

    return {
        'interaction_power_l': power_l.astype(np.float32),
        'interaction_power_r': power_r.astype(np.float32),
        'interaction_work_pos': work_pos.astype(np.float32),
        'interaction_work_neg': work_neg.astype(np.float32),
    }


def _get_feature_group(gpath, v_name):
    """Classify a feature variable into its noise-group for per-group LPF/noise."""
    if gpath == 'sub_info':
        return 'subject_info'
    if gpath == 'derived/body_frame_imu':
        if v_name.startswith('accel_') or v_name == 'forward_accel_overground':
            return 'accelerations'
        if v_name.startswith('gyro_'):
            return 'imu'
    if gpath in (
        'derived/imu_integrated_speed',
        'derived/model_com_speed_nogrf',
        'derived/model_com_speed',
        'derived/imu_speed_causal',
        'derived/stride_progression',
    ):
        return 'velocities'
    if gpath == 'derived/contact_estimate':
        return 'velocities'
    if gpath == 'derived/interaction_power':
        return 'torques'
    if v_name.endswith('_ddot'):
        return 'accelerations'
    if v_name.endswith('_dot'):
        return 'velocities'
    if 'back_imu' in gpath:
        return 'imu'
    if 'robot/' in gpath and v_name == 'torque':
        return 'torques'
    if 'forceplate' in gpath or 'grf' in gpath:
        return 'grf'
    if 'mocap' in gpath or 'kin_q' in gpath:
        return 'angles'
    return 'other'


def extract_condition_data_v2(
    f, sub, cond,
    input_vars, output_vars,
    lpf_cutoff=None, lpf_order=4, fs=fs,
    include_levels=None, include_trials=None,
    use_physical_velocity_model=False,
    use_gait_phase=False,
    input_lpf_cutoff=None, input_lpf_order=4,
    input_lpf_per_group=None
):
    """
    input_vars, output_vars: list of (group_path, [var_names...])
    input_lpf_per_group: dict mapping group name -> cutoff Hz (e.g. {'accelerations': 8.0})
    Return: list of (T, InDim) arrays, list of (T, OutDim) arrays
    """
    if sub not in f or cond not in f[sub]:
        return [], []

    cond_group = f[sub][cond]
    level_keys = list(cond_group.keys())

    in_list_all = []
    out_list_all = []

    for lv in level_keys:
        if include_levels and lv not in include_levels:
            continue

        lv_group = cond_group[lv]
        trial_keys = list(lv_group.keys())

        for trial_name in trial_keys:
            if include_trials and trial_name not in include_trials:
                continue
            trial_group = lv_group[trial_name]

            kin_q_arrays = []
            curr_trial_in = []
            valid_trial = True
            trial_cache = {}

            for gpath, vars in input_vars:
                # sub_info: subject-level scalar constants (height, weight, etc.)
                if gpath == 'sub_info':
                    ref = get_data_from_group(trial_group, "treadmill/left/speed_leftbelt")
                    if ref is None:
                        ref = get_data_from_group(trial_group, "robot/left/torque")
                    if ref is None:
                        valid_trial = False; break
                    n_si = len(ref.flatten())
                    si = f[sub].get('sub_info') if sub in f else None
                    si_cols = []
                    for v_name in vars:
                        default = {'height': 1.70, 'weight': 70.0}.get(v_name, 0.0)
                        val = float(si.attrs[v_name]) if (si is not None and v_name in si.attrs) else default
                        si_cols.append(np.full((n_si, 1), val, dtype=np.float32))
                    si_arr = np.concatenate(si_cols, axis=1)
                    kin_q_arrays.append(si_arr)
                    curr_trial_in.append(si_arr)
                    continue

                grp = trial_group
                parts = gpath.split('/')
                missing_grp = False
                if not gpath.startswith('derived'):
                    for p in parts:
                        if p in grp: grp = grp[p]
                        else: missing_grp=True; break

                    if missing_grp:
                        print(f"[DEBUG] {sub}/{cond}/{lv}/{trial_name}: Missing group {gpath}")
                        valid_trial = False
                        break

                block_cols = []
                derived_cache = {}

                a_belt_global = 0.0
                v_belt_data = get_data_from_group(trial_group, "treadmill/left/speed_leftbelt")
                if v_belt_data is not None:
                    v_belt_flat = v_belt_data.flatten()
                    a_belt_global = np.gradient(v_belt_flat, axis=0) * fs

                for v_name in vars:
                    col_data = None
                    base_name = None
                    is_accel = False

                    # 1. Direct Load
                    if not gpath.startswith('derived') and v_name in grp:
                        col_data = grp[v_name][:]
                        # Per-group LPF takes priority over global LPF
                        if input_lpf_per_group is not None:
                            fgroup = _get_feature_group(gpath, v_name)
                            if fgroup in input_lpf_per_group:
                                col_data = butter_lowpass_filter(col_data, input_lpf_per_group[fgroup], fs, input_lpf_order)
                        elif input_lpf_cutoff is not None:
                            col_data = butter_lowpass_filter(col_data, input_lpf_cutoff, fs, input_lpf_order)

                    # 2. Derived Logic (Velocity/Acceleration)
                    if col_data is None and not gpath.startswith('derived') and (v_name.endswith('_dot') or v_name.endswith('_ddot')):
                        if v_name.endswith('_ddot'):
                            base_name = v_name[:-5]
                            is_accel = True
                        else:
                            base_name = v_name[:-4]
                            is_accel = False

                        if base_name in grp:
                            base_data = grp[base_name][:]
                            d1 = np.gradient(base_data, axis=0) * fs
                            if is_accel:
                                target_data = np.gradient(d1, axis=0) * fs
                            else:
                                target_data = d1

                            # Per-group LPF for derived features (replaces default 30Hz)
                            _derived_cutoff = 30.0
                            if input_lpf_per_group is not None:
                                _dgroup = 'accelerations' if is_accel else 'velocities'
                                if _dgroup in input_lpf_per_group:
                                    _derived_cutoff = input_lpf_per_group[_dgroup]
                            target_data = butter_lowpass_filter(target_data, cutoff=_derived_cutoff, fs=fs, order=4)
                            col_data = target_data

                            if use_physical_velocity_model and not is_accel and 'hip' in v_name and 'flexion' in v_name:
                                L = 0.9
                                if sub in f and "leg_length" in f[sub].attrs: L = f[sub].attrs["leg_length"]
                                elif sub in f and "sub_info" in f[sub] and "leg_length" in f[sub]["sub_info"].attrs: L = f[sub]["sub_info"].attrs["leg_length"]
                                col_data = L * target_data * np.cos(base_data)
                        else:
                            print(f"[WARN] Base data {base_name} not found for {v_name}")
                            valid_trial = False; break

                    # [NEW] Fallback for Milestone 1: hip_angle -> motor_angle
                    if col_data is None and 'hip_angle' in v_name:
                        alt_name = v_name.replace('hip_angle', 'motor_angle')
                        if alt_name in grp:
                            col_data = grp[alt_name][:]
                        else:
                            print(f"[DEBUG-FALLBACK] {alt_name} NOT found in grp.")

                    # 3. Derive Contact from GRF Z
                    if col_data is None and v_name == 'contact':
                        if 'z' in grp:
                            col_data = get_ground_contact(grp['z'][:], threshold=20.0).reshape(-1, 1)
                        else:
                            print(f"[DEBUG] GRF Z not found for contact")
                            valid_trial = False; break

                    # 4. Gait Phase (Derived)
                    if col_data is None and gpath == "derived" and "gait_phase" in v_name:
                        side = 'left' if '_L' in v_name else 'right'
                        z_data = get_data_from_group(trial_group, f"forceplate/grf/{side}/z")
                        if z_data is not None:
                            cont = get_ground_contact(z_data)
                            pad = np.pad(cont.flatten(), (1,0), constant_values=0)
                            hs = np.where(np.diff(pad) == 1)[0]
                            ph = np.zeros_like(cont)
                            for k in range(len(hs)-1):
                                s, e = hs[k], hs[k+1]
                                ph[s:e] = np.linspace(0, 1, e-s)
                            if len(hs)>0: ph[hs[-1]:] = 1.0
                            col_data = ph[:, None]
                        else:
                            col_data = np.zeros((get_data_from_group(trial_group, "robot/left/hip_angle").shape[0], 1))

                    # 5. Advanced IMU Features (Orientation & Integrated Velocity)
                    if col_data is None and gpath == "robot/back_imu" and v_name in ["roll", "pitch", "yaw", "vel_forward"]:
                        try:
                            if v_name in derived_cache:
                                col_data = derived_cache[v_name]
                            else:
                                raw_ax = grp['accel_x'][:]
                                raw_ay = grp['accel_y'][:]
                                raw_az = grp['accel_z'][:]
                                raw_gx = grp['gyro_x'][:]
                                raw_gy = grp['gyro_y'][:]
                                raw_gz = grp['gyro_z'][:]

                                if sub.startswith("m1_"): raw_ax = -raw_ax
                                elif sub.startswith("m2_"): raw_az = -raw_az

                                n = len(raw_ax)
                                dt = 1.0 / fs
                                alpha = 0.98
                                rs, ps, ys = np.zeros(n), np.zeros(n), np.zeros(n)
                                curr_r, curr_p, curr_y = 0.0, 0.0, 0.0
                                for i in range(n):
                                    r_a = np.arctan2(raw_ay[i], raw_az[i])
                                    p_a = np.arctan2(-raw_ax[i], np.sqrt(raw_ay[i]**2 + raw_az[i]**2))
                                    curr_r = alpha*(curr_r + raw_gx[i]*dt) + (1-alpha)*r_a
                                    curr_p = alpha*(curr_p + raw_gy[i]*dt) + (1-alpha)*p_a
                                    curr_y += raw_gz[i] * dt
                                    rs[i], ps[i], ys[i] = curr_r, curr_p, curr_y

                                a_lin = raw_ax + 9.81 * np.sin(ps)
                                if isinstance(a_belt_global, np.ndarray) and len(a_belt_global) == n:
                                    a_lin = a_lin + a_belt_global

                                vel_f = butter_highpass_filter(np.cumsum(a_lin)*dt, 0.1, fs, 2)

                                derived_cache["roll"], derived_cache["pitch"], derived_cache["yaw"], derived_cache["vel_forward"] = rs, ps, ys, vel_f
                                col_data = derived_cache[v_name]
                        except Exception as e:
                            print(f"[ERROR] IMU Feature Error: {e}")
                            valid_trial = False; break

                    # 6. Global IMU (quaternion-based frame transform)
                    if col_data is None and gpath == "robot/back_imu" and v_name in _GLOBAL_IMU_VARS:
                        try:
                            if 'global_imu_cache' not in derived_cache:
                                _bg = trial_group
                                for _p in ['robot', 'back_imu']:
                                    _bg = _bg[_p] if _p in _bg else None
                                    if _bg is None: break
                                back_imu_grp = _bg
                                if back_imu_grp is not None and 'quat_w' in back_imu_grp:
                                    quat = np.stack([
                                        back_imu_grp['quat_w'][:], back_imu_grp['quat_x'][:],
                                        back_imu_grp['quat_y'][:], back_imu_grp['quat_z'][:]
                                    ], axis=1)
                                    _ax = back_imu_grp['accel_x'][:]
                                    _ay = back_imu_grp['accel_y'][:]
                                    _az = back_imu_grp['accel_z'][:]
                                    _gx = back_imu_grp['gyro_x'][:]
                                    _gy = back_imu_grp['gyro_y'][:]
                                    _gz = back_imu_grp['gyro_z'][:]
                                    if sub.startswith('m2_'): _az = -_az
                                    elif sub.startswith('m1_'): _ax = -_ax
                                    ag, gg = compute_global_imu(
                                        np.stack([_ax, _ay, _az], axis=1),
                                        np.stack([_gx, _gy, _gz], axis=1),
                                        quat, calib_n=100)
                                    dt_g = 1.0 / fs
                                    vel_y = butter_highpass_filter(np.cumsum(ag[:, 1]) * dt_g, 0.1, fs, 2)
                                    v_belt = get_data_from_group(trial_group, 'treadmill/left/speed_leftbelt')
                                    if v_belt is not None:
                                        n_g = len(ag)
                                        a_b = np.gradient(v_belt.flatten()) * fs
                                        a_b = a_b[:n_g] if len(a_b) >= n_g else np.pad(a_b, (0, n_g - len(a_b)))
                                        vel_tread = butter_highpass_filter(
                                            np.cumsum((ag[:, 1] + a_b)) * dt_g, 0.1, fs, 2)
                                    else:
                                        vel_tread = vel_y
                                else:
                                    n_g = len(grp['accel_x'][:]) if 'accel_x' in grp else 1
                                    ag = np.zeros((n_g, 3))
                                    gg = np.zeros((n_g, 3))
                                    vel_y = vel_tread = np.zeros(n_g)
                                derived_cache['global_imu_cache'] = {
                                    'global_acc_x': ag[:, 0], 'global_acc_y': ag[:, 1], 'global_acc_z': ag[:, 2],
                                    'global_gyr_x': gg[:, 0], 'global_gyr_y': gg[:, 1], 'global_gyr_z': gg[:, 2],
                                    'global_vel_y': vel_y, 'global_vel_treadmill': vel_tread,
                                }
                            col_data = derived_cache['global_imu_cache'].get(v_name, np.zeros(1))
                        except Exception as e:
                            print(f"[ERROR] Global IMU Error: {e}")
                            valid_trial = False; break

                    # 7. Gait Phase / Foot Contact (from forceplate/grf)
                    if col_data is None and "forceplate/grf" in gpath and v_name in ('gait_phase_l', 'gait_phase_r', 'contact_l', 'contact_r', 'stride_freq_l', 'stride_freq_r'):
                        side = 'left' if v_name.endswith('_l') else 'right'
                        fz = get_data_from_group(trial_group, f'forceplate/grf/{side}/z')
                        if fz is not None:
                            if 'gait_phase' in v_name:
                                col_data = compute_gait_phase(fz.flatten(), threshold=20.0).reshape(-1, 1)
                            elif 'stride_freq' in v_name:
                                col_data = compute_stride_frequency(fz.flatten(), fs=fs, threshold=20.0).reshape(-1, 1)
                            else:
                                col_data = get_ground_contact(fz.flatten(), threshold=20.0).reshape(-1, 1)
                        else:
                            valid_trial = False; break

                    # 8. Kinematic Chain COM Velocity (archive_round5)
                    if col_data is None and gpath == 'derived' and v_name in _CHAIN_VELOCITY_VARS:
                        try:
                            if 'chain_vel_cache' not in trial_cache:
                                kinq = trial_group['mocap']['kin_q']
                                ank_l = kinq['ankle_angle_l'][:].flatten()
                                ank_r = kinq['ankle_angle_r'][:].flatten()
                                kne_l = kinq['knee_angle_l'][:].flatten()
                                kne_r = kinq['knee_angle_r'][:].flatten()
                                hip_l = kinq['hip_flexion_l'][:].flatten()
                                hip_r = kinq['hip_flexion_r'][:].flatten()
                                p_sag = kinq['pelvis_list'][:].flatten()
                                p_rot = kinq['pelvis_rotation'][:].flatten()
                                # Ground contact from GRF
                                fz_l = get_data_from_group(trial_group, 'forceplate/grf/left/z')
                                fz_r = get_data_from_group(trial_group, 'forceplate/grf/right/z')
                                if fz_l is None or fz_r is None:
                                    raise ValueError("GRF Fz required for chain velocity")
                                ct_l = get_ground_contact(fz_l.flatten(), threshold=20.0)
                                ct_r = get_ground_contact(fz_r.flatten(), threshold=20.0)
                                # Sign correction for m2
                                if sub.startswith("m2_"):
                                    fz_l_raw = -fz_l.flatten()
                                    fz_r_raw = -fz_r.flatten()
                                    ct_l = get_ground_contact(fz_l_raw, threshold=20.0)
                                    ct_r = get_ground_contact(fz_r_raw, threshold=20.0)
                                # Leg lengths from sub_info
                                leg_l_m = 0.9  # default
                                leg_r_m = 0.9
                                if sub in f and 'sub_info' in f[sub]:
                                    si = f[sub]['sub_info']
                                    if 'left' in si and 'leg length' in si['left']:
                                        leg_l_m = float(si['left']['leg length'][()]) / 1000.0
                                    if 'right' in si and 'leg length' in si['right']:
                                        leg_r_m = float(si['right']['leg length'][()]) / 1000.0
                                n_min = min(len(ank_l), len(ct_l))
                                com_v, seg_a, seg_k, seg_h = compute_com_chain_velocity(
                                    ank_l[:n_min], ank_r[:n_min],
                                    kne_l[:n_min], kne_r[:n_min],
                                    hip_l[:n_min], hip_r[:n_min],
                                    p_sag[:n_min], p_rot[:n_min],
                                    ct_l[:n_min], ct_r[:n_min],
                                    leg_l_m, leg_r_m, fs_hz=fs
                                )
                                # Also compute v2 (absolute segment angles, relaxed filtering)
                                com_v2, _, _, _ = compute_com_chain_velocity_v2(
                                    ank_l[:n_min], ank_r[:n_min],
                                    kne_l[:n_min], kne_r[:n_min],
                                    hip_l[:n_min], hip_r[:n_min],
                                    p_sag[:n_min], p_rot[:n_min],
                                    ct_l[:n_min], ct_r[:n_min],
                                    leg_l_m, leg_r_m, fs_hz=fs
                                )
                                trial_cache['chain_vel_cache'] = {
                                    'com_velocity_chain': com_v,
                                    'com_velocity_chain_v2': com_v2,
                                    'segment_vel_ankle': seg_a,
                                    'segment_vel_knee': seg_k,
                                    'segment_vel_hip': seg_h,
                                }
                            col_data = trial_cache['chain_vel_cache'][v_name].reshape(-1, 1)
                        except Exception as e:
                            print(f"[ERROR] Chain Velocity Error: {e}")
                            valid_trial = False; break

                    # 9. Biomechanical Leading Indicators (archive_round5)
                    if col_data is None and gpath == 'derived' and v_name in _BIOMECH_LEADING_VARS:
                        try:
                            # GRF impulse per stance
                            if v_name.startswith('grf_impulse'):
                                side = 'left' if v_name.endswith('_l') else 'right'
                                fz = get_data_from_group(trial_group, f'forceplate/grf/{side}/z')
                                fy = get_data_from_group(trial_group, f'forceplate/grf/{side}/y')
                                if fz is None or fy is None:
                                    raise ValueError(f"GRF data required for {v_name}")
                                fz_flat = fz.flatten()
                                fy_flat = fy.flatten()
                                if sub.startswith("m2_"):
                                    fz_flat = -fz_flat
                                col_data = compute_grf_impulse_per_stance(
                                    fy_flat, fz_flat, fs_hz=fs
                                ).reshape(-1, 1)

                            # Stance ratio
                            elif v_name.startswith('stance_ratio'):
                                side = 'left' if v_name.endswith('_l') else 'right'
                                fz = get_data_from_group(trial_group, f'forceplate/grf/{side}/z')
                                if fz is None:
                                    raise ValueError(f"GRF Fz required for {v_name}")
                                fz_flat = fz.flatten()
                                if sub.startswith("m2_"):
                                    fz_flat = -fz_flat
                                col_data = compute_stance_ratio(
                                    fz_flat, fs_hz=fs
                                ).reshape(-1, 1)

                            # Trunk pitch angular velocity (from IMU complementary filter)
                            elif v_name == 'trunk_pitch_dot':
                                if 'pitch' not in derived_cache:
                                    # Trigger IMU pitch computation
                                    back_imu = trial_group.get('robot', {}).get('back_imu', None)
                                    if back_imu is None:
                                        raise ValueError("back_imu required for trunk_pitch_dot")
                                    raw_ax = back_imu['accel_x'][:]
                                    raw_ay = back_imu['accel_y'][:]
                                    raw_az = back_imu['accel_z'][:]
                                    raw_gx = back_imu['gyro_x'][:]
                                    raw_gy = back_imu['gyro_y'][:]
                                    if sub.startswith("m1_"): raw_ax = -raw_ax
                                    elif sub.startswith("m2_"): raw_az = -raw_az
                                    n_imu = len(raw_ax)
                                    dt_imu = 1.0 / fs
                                    alpha = 0.98
                                    ps = np.zeros(n_imu)
                                    curr_p = 0.0
                                    for ii in range(n_imu):
                                        p_a = np.arctan2(-raw_ax[ii], np.sqrt(raw_ay[ii]**2 + raw_az[ii]**2))
                                        curr_p = alpha * (curr_p + raw_gy[ii] * dt_imu) + (1 - alpha) * p_a
                                        ps[ii] = curr_p
                                    derived_cache['pitch'] = ps
                                pitch = derived_cache['pitch']
                                pitch_dot = np.gradient(pitch) * fs
                                pitch_dot = butter_lowpass_filter(pitch_dot, 10.0, fs, 2)
                                col_data = pitch_dot.reshape(-1, 1)

                            # CoP forward velocity
                            elif v_name.startswith('cop_vel_y'):
                                side = 'left' if v_name.endswith('_l') else 'right'
                                fz = get_data_from_group(trial_group, f'forceplate/grf/{side}/z')
                                fy = get_data_from_group(trial_group, f'forceplate/grf/{side}/y')
                                if fz is None or fy is None:
                                    raise ValueError(f"GRF data required for {v_name}")
                                fz_flat = fz.flatten()
                                fy_flat = fy.flatten()
                                if sub.startswith("m2_"):
                                    fz_flat = -fz_flat
                                # CoP_y ≈ -Mx/Fz ≈ fy/fz (simplified for plate without moments)
                                # Use AP force / vertical force as proxy for CoP velocity
                                safe_fz = np.where(np.abs(fz_flat) > 20.0, fz_flat, 1.0)
                                cop_ratio = fy_flat / safe_fz
                                cop_vel = np.gradient(cop_ratio) * fs
                                cop_vel = butter_lowpass_filter(cop_vel, 10.0, fs, 2)
                                col_data = cop_vel.reshape(-1, 1)

                            else:
                                if gpath == 'derived':
                                    print(f"[DEBUG] Unknown derived variable: {v_name}")
                                valid_trial = False; break

                        except Exception as e:
                            print(f"[ERROR] Biomech Leading Indicator Error ({v_name}): {e}")
                            valid_trial = False; break

                    # 10. Body Frame IMU (derived/body_frame_imu)
                    if col_data is None and gpath == 'derived/body_frame_imu' and v_name in _BODY_FRAME_IMU_VARS:
                        try:
                            if 'body_frame_cache' not in derived_cache:
                                imu_grp = trial_group['robot']['back_imu']
                                ax = imu_grp['accel_x'][:]
                                ay = imu_grp['accel_y'][:]
                                az = imu_grp['accel_z'][:]
                                gx = imu_grp['gyro_x'][:]
                                gy = imu_grp['gyro_y'][:]
                                gz = imu_grp['gyro_z'][:]
                                qw = imu_grp['quat_w'][:]
                                qx_d = imu_grp['quat_x'][:]
                                qy_d = imu_grp['quat_y'][:]
                                qz_d = imu_grp['quat_z'][:]

                                _acc_local = np.stack([ax, ay, az], axis=1).astype(np.float32)
                                _gyro_local = np.stack([gx, gy, gz], axis=1).astype(np.float32)
                                _quat_wxyz = np.stack([qw, qx_d, qy_d, qz_d], axis=1).astype(np.float32)

                                v_l = get_data_from_group(trial_group, 'treadmill/left/speed_leftbelt')
                                v_r = get_data_from_group(trial_group, 'treadmill/right/speed_rightbelt')
                                v_mean = (v_l.flatten() + v_r.flatten()) / 2.0
                                _a_tread = np.gradient(v_mean) * fs

                                derived_cache['body_frame_cache'] = estimate_body_frame_features(
                                    _acc_local, _gyro_local, _quat_wxyz,
                                    _a_tread.astype(np.float32), fs)

                            _bf = derived_cache['body_frame_cache']
                            _bf_map = {
                                'accel_right': _bf['acc_body'][:, 0:1],
                                'accel_forward': _bf['acc_body'][:, 1:2],
                                'accel_up': _bf['acc_body'][:, 2:3],
                                'gyro_right': _bf['gyro_body'][:, 0:1],
                                'gyro_forward': _bf['gyro_body'][:, 1:2],
                                'gyro_up': _bf['gyro_body'][:, 2:3],
                                'forward_accel_overground': _bf['forward_accel_overground'],
                            }
                            col_data = _bf_map[v_name]
                        except Exception as e:
                            print(f"[ERROR] Body Frame IMU Error ({v_name}): {e}")
                            valid_trial = False; break

                    # 11. Model-based COM Speed (derived/model_com_speed)
                    if col_data is None and gpath == 'derived/model_com_speed' and v_name in _MODEL_COM_SPEED_VARS:
                        try:
                            if 'model_com_speed' not in derived_cache:
                                # Get joint angles from mocap
                                hip_flex_l = get_data_from_group(trial_group, 'mocap/kin_q/hip_flexion_l')
                                hip_flex_r = get_data_from_group(trial_group, 'mocap/kin_q/hip_flexion_r')
                                knee_l = get_data_from_group(trial_group, 'mocap/kin_q/knee_angle_l')
                                knee_r = get_data_from_group(trial_group, 'mocap/kin_q/knee_angle_r')
                                pel_tilt = get_data_from_group(trial_group, 'mocap/kin_q/pelvis_tilt')
                                pel_rot = get_data_from_group(trial_group, 'mocap/kin_q/pelvis_rotation')

                                # GRF for stance detection
                                # Raw GRF z is negative during stance (contact force convention)
                                # No sign flip needed — compute_model_com_speed uses threshold < -20
                                grf_z_l_raw = get_data_from_group(trial_group, 'forceplate/grf/left/z')
                                grf_z_r_raw = get_data_from_group(trial_group, 'forceplate/grf/right/z')

                                # Get height from sub_info (stored as dataset in mm, e.g. b'1780')
                                si = f[sub].get('sub_info') if sub in f else None
                                height_mm = 1700.0
                                if si is not None:
                                    if 'height' in si.attrs:
                                        height_mm = float(si.attrs['height'])
                                    elif 'height' in si:
                                        height_mm = float(si['height'][()])

                                derived_cache['model_com_speed'] = compute_model_com_speed(
                                    hip_flex_l, hip_flex_r,
                                    knee_l, knee_r,
                                    pel_tilt, pel_rot,
                                    grf_z_l_raw, grf_z_r_raw,
                                    height_mm, fs)

                            col_data = derived_cache['model_com_speed'].reshape(-1, 1)
                        except Exception as e:
                            print(f"[ERROR] Model COM Speed Error ({v_name}): {e}")
                            valid_trial = False; break

                    # 11b. GRF-free model-based COM Speed variants
                    if col_data is None and gpath == 'derived/model_com_speed_nogrf' and v_name in _MODEL_COM_SPEED_NOGRF_VARS:
                        try:
                            cache_key = f"{v_name}_cache"
                            if cache_key not in derived_cache:
                                hip_flex_l = get_data_from_group(trial_group, 'mocap/kin_q/hip_flexion_l')
                                hip_flex_r = get_data_from_group(trial_group, 'mocap/kin_q/hip_flexion_r')
                                knee_l = get_data_from_group(trial_group, 'mocap/kin_q/knee_angle_l')
                                knee_r = get_data_from_group(trial_group, 'mocap/kin_q/knee_angle_r')
                                pelvis_tilt = get_data_from_group(trial_group, 'mocap/kin_q/pelvis_tilt')
                                pelvis_rot = get_data_from_group(trial_group, 'mocap/kin_q/pelvis_rotation')
                                if any(x is None for x in [hip_flex_l, hip_flex_r, knee_l, knee_r, pelvis_tilt, pelvis_rot]):
                                    raise ValueError("kinematic inputs required for model_com_speed_nogrf")

                                si = f[sub].get('sub_info') if sub in f else None
                                height_mm = 1700.0
                                if si is not None:
                                    if 'height' in si.attrs:
                                        height_mm = float(si.attrs['height'])
                                    elif 'height' in si:
                                        height_mm = float(si['height'][()])

                                mode = 'fusion'
                                if 'phase' in v_name:
                                    mode = 'phase'
                                elif 'kin' in v_name:
                                    mode = 'kin'

                                acc_forward_body = None
                                acc_up_body = None
                                gyro_forward = None
                                if mode == 'fusion':
                                    if 'body_frame_cache' not in derived_cache:
                                        imu_grp = trial_group['robot']['back_imu']
                                        _acc_local = np.stack([
                                            imu_grp['accel_x'][:], imu_grp['accel_y'][:], imu_grp['accel_z'][:]
                                        ], axis=1).astype(np.float32)
                                        _gyro_local = np.stack([
                                            imu_grp['gyro_x'][:], imu_grp['gyro_y'][:], imu_grp['gyro_z'][:]
                                        ], axis=1).astype(np.float32)
                                        _quat_wxyz = np.stack([
                                            imu_grp['quat_w'][:], imu_grp['quat_x'][:],
                                            imu_grp['quat_y'][:], imu_grp['quat_z'][:]
                                        ], axis=1).astype(np.float32)
                                        zero_tread = np.zeros(len(_acc_local), dtype=np.float32)
                                        derived_cache['body_frame_cache'] = estimate_body_frame_features(
                                            _acc_local, _gyro_local, _quat_wxyz, zero_tread, fs
                                        )
                                    bf = derived_cache['body_frame_cache']
                                    acc_forward_body = bf['acc_body'][:, 1].flatten()
                                    acc_up_body = bf['acc_body'][:, 2].flatten()
                                    gyro_forward = bf['gyro_body'][:, 1].flatten()

                                derived_cache[cache_key] = compute_model_com_speed_nogrf(
                                    hip_flex_l, hip_flex_r, knee_l, knee_r,
                                    pelvis_tilt, pelvis_rot,
                                    height_mm=height_mm, fs=fs,
                                    contact_mode=mode,
                                    acc_forward_body=acc_forward_body,
                                    acc_up_body=acc_up_body,
                                    gyro_forward=gyro_forward,
                                )

                            col_data = derived_cache[cache_key].reshape(-1, 1)
                        except Exception as e:
                            print(f"[ERROR] Model COM Speed No-GRF Error ({v_name}): {e}")
                            valid_trial = False; break

                    # 12. IMU Integrated Speed (derived/imu_integrated_speed)
                    if col_data is None and gpath == 'derived/imu_integrated_speed' and v_name in _IMU_INTEGRATED_SPEED_VARS:
                        try:
                            if 'imu_integrated_speed' not in derived_cache:
                                # Use forward_accel_overground (= body_frame +Y acc + a_tread)
                                if 'body_frame_cache' not in derived_cache:
                                    imu_grp = trial_group['robot']['back_imu']
                                    _acc_local = np.stack([
                                        imu_grp['accel_x'][:], imu_grp['accel_y'][:], imu_grp['accel_z'][:]
                                    ], axis=1).astype(np.float32)
                                    _gyro_local = np.stack([
                                        imu_grp['gyro_x'][:], imu_grp['gyro_y'][:], imu_grp['gyro_z'][:]
                                    ], axis=1).astype(np.float32)
                                    _quat_wxyz = np.stack([
                                        imu_grp['quat_w'][:], imu_grp['quat_x'][:],
                                        imu_grp['quat_y'][:], imu_grp['quat_z'][:]
                                    ], axis=1).astype(np.float32)

                                    v_l = get_data_from_group(trial_group, 'treadmill/left/speed_leftbelt')
                                    v_r = get_data_from_group(trial_group, 'treadmill/right/speed_rightbelt')
                                    v_mean = (v_l.flatten() + v_r.flatten()) / 2.0
                                    _a_tread = np.gradient(v_mean) * fs

                                    derived_cache['body_frame_cache'] = estimate_body_frame_features(
                                        _acc_local, _gyro_local, _quat_wxyz,
                                        _a_tread.astype(np.float32), fs)

                                # forward_accel_overground = acc_body_Y + a_tread (overground equivalent)
                                acc_og = derived_cache['body_frame_cache']['forward_accel_overground'].flatten()
                                derived_cache['imu_integrated_speed'] = compute_imu_integrated_speed(acc_og, fs)

                            col_data = derived_cache['imu_integrated_speed'].reshape(-1, 1)
                        except Exception as e:
                            print(f"[ERROR] IMU Integrated Speed Error ({v_name}): {e}")
                            valid_trial = False; break

                    # 13. Treadmill-independent causal IMU speed prior
                    if col_data is None and gpath == 'derived/imu_speed_causal' and v_name in _IMU_CAUSAL_SPEED_VARS:
                        try:
                            if 'imu_speed_causal' not in derived_cache:
                                if 'body_frame_cache' not in derived_cache:
                                    imu_grp = trial_group['robot']['back_imu']
                                    _acc_local = np.stack([
                                        imu_grp['accel_x'][:], imu_grp['accel_y'][:], imu_grp['accel_z'][:]
                                    ], axis=1).astype(np.float32)
                                    _gyro_local = np.stack([
                                        imu_grp['gyro_x'][:], imu_grp['gyro_y'][:], imu_grp['gyro_z'][:]
                                    ], axis=1).astype(np.float32)
                                    _quat_wxyz = np.stack([
                                        imu_grp['quat_w'][:], imu_grp['quat_x'][:],
                                        imu_grp['quat_y'][:], imu_grp['quat_z'][:]
                                    ], axis=1).astype(np.float32)
                                    zero_tread = np.zeros(len(_acc_local), dtype=np.float32)
                                    derived_cache['body_frame_cache'] = estimate_body_frame_features(
                                        _acc_local, _gyro_local, _quat_wxyz,
                                        zero_tread, fs)

                                acc_forward_body = derived_cache['body_frame_cache']['acc_body'][:, 1].flatten()
                                derived_cache['imu_speed_causal'] = compute_imu_integrated_speed(acc_forward_body, fs)

                            col_data = derived_cache['imu_speed_causal'].reshape(-1, 1)
                        except Exception as e:
                            print(f"[ERROR] IMU Causal Speed Error ({v_name}): {e}")
                            valid_trial = False; break

                    # 13b. GRF-free contact estimates
                    if col_data is None and gpath == 'derived/contact_estimate' and v_name in _CONTACT_ESTIMATE_VARS:
                        try:
                            if 'contact_estimate_cache' not in derived_cache:
                                hip_l = get_data_from_group(trial_group, 'mocap/kin_q/hip_flexion_l')
                                hip_r = get_data_from_group(trial_group, 'mocap/kin_q/hip_flexion_r')
                                knee_l = get_data_from_group(trial_group, 'mocap/kin_q/knee_angle_l')
                                knee_r = get_data_from_group(trial_group, 'mocap/kin_q/knee_angle_r')
                                ankle_l = get_data_from_group(trial_group, 'mocap/kin_q/ankle_angle_l')
                                ankle_r = get_data_from_group(trial_group, 'mocap/kin_q/ankle_angle_r')
                                if any(x is None for x in [hip_l, hip_r, knee_l, knee_r, ankle_l, ankle_r]):
                                    raise ValueError("kinematic inputs required for contact estimate")

                                p_l_phase, p_r_phase = compute_contact_probability_phase(hip_l, hip_r, fs)
                                p_l_kin, p_r_kin = compute_contact_probability_kinematics(
                                    hip_l, hip_r, knee_l, knee_r, fs
                                )

                                if 'body_frame_cache' not in derived_cache:
                                    imu_grp = trial_group['robot']['back_imu']
                                    _acc_local = np.stack([
                                        imu_grp['accel_x'][:], imu_grp['accel_y'][:], imu_grp['accel_z'][:]
                                    ], axis=1).astype(np.float32)
                                    _gyro_local = np.stack([
                                        imu_grp['gyro_x'][:], imu_grp['gyro_y'][:], imu_grp['gyro_z'][:]
                                    ], axis=1).astype(np.float32)
                                    _quat_wxyz = np.stack([
                                        imu_grp['quat_w'][:], imu_grp['quat_x'][:],
                                        imu_grp['quat_y'][:], imu_grp['quat_z'][:]
                                    ], axis=1).astype(np.float32)
                                    zero_tread = np.zeros(len(_acc_local), dtype=np.float32)
                                    derived_cache['body_frame_cache'] = estimate_body_frame_features(
                                        _acc_local, _gyro_local, _quat_wxyz, zero_tread, fs
                                    )
                                bf = derived_cache['body_frame_cache']
                                p_l_fusion, p_r_fusion = compute_contact_probability_fusion(
                                    hip_l, hip_r, knee_l, knee_r,
                                    bf['acc_body'][:, 1].flatten(),
                                    bf['acc_body'][:, 2].flatten(),
                                    bf['gyro_body'][:, 1].flatten(),
                                    fs,
                                )
                                derived_cache['contact_estimate_cache'] = {
                                    'contact_prob_l_phase': p_l_phase,
                                    'contact_prob_r_phase': p_r_phase,
                                    'contact_prob_l_kin': p_l_kin,
                                    'contact_prob_r_kin': p_r_kin,
                                    'contact_prob_l_fusion': p_l_fusion,
                                    'contact_prob_r_fusion': p_r_fusion,
                                }

                            col_data = derived_cache['contact_estimate_cache'][v_name].reshape(-1, 1)
                        except Exception as e:
                            print(f"[ERROR] Contact Estimate Error ({v_name}): {e}")
                            valid_trial = False; break

                    # 14. Support/stride progression prior from robot encoders
                    if col_data is None and gpath == 'derived/stride_progression' and v_name in _STRIDE_PROGRESSION_VARS:
                        try:
                            if 'stride_speed_robot' not in derived_cache:
                                hip_l = get_data_from_group(trial_group, 'robot/left/hip_angle')
                                hip_r = get_data_from_group(trial_group, 'robot/right/hip_angle')
                                if hip_l is None or hip_r is None:
                                    raise ValueError("robot hip_angle required for stride progression prior")

                                si = f[sub].get('sub_info') if sub in f else None
                                height_mm = 1700.0
                                if si is not None:
                                    if 'height' in si.attrs:
                                        height_mm = float(si.attrs['height'])
                                    elif 'height' in si:
                                        height_mm = float(si['height'][()])

                                derived_cache['stride_speed_robot'] = compute_robot_stride_speed(
                                    hip_l, hip_r, height_m=float(height_mm) / 1000.0, fs_hz=fs)

                            col_data = derived_cache['stride_speed_robot'].reshape(-1, 1)
                        except Exception as e:
                            print(f"[ERROR] Stride Progression Error ({v_name}): {e}")
                            valid_trial = False; break

                    # 15. Human-robot interaction power/work
                    if col_data is None and gpath == 'derived/interaction_power' and v_name in _INTERACTION_POWER_VARS:
                        try:
                            if 'interaction_power_cache' not in derived_cache:
                                hip_l = get_data_from_group(trial_group, 'robot/left/hip_angle')
                                hip_r = get_data_from_group(trial_group, 'robot/right/hip_angle')
                                tq_l = get_data_from_group(trial_group, 'robot/left/torque')
                                tq_r = get_data_from_group(trial_group, 'robot/right/torque')
                                if hip_l is None or hip_r is None or tq_l is None or tq_r is None:
                                    raise ValueError("robot hip_angle and torque required for interaction power")
                                derived_cache['interaction_power_cache'] = compute_interaction_power_features(
                                    hip_l, hip_r, tq_l, tq_r, fs_hz=fs)

                            col_data = derived_cache['interaction_power_cache'][v_name].reshape(-1, 1)
                        except Exception as e:
                            print(f"[ERROR] Interaction Power Error ({v_name}): {e}")
                            valid_trial = False; break

                    # Fallback: no handler found
                    if col_data is None:
                        if gpath.startswith('derived'):
                            print(f"[DEBUG] {sub}/{cond}/{lv}/{trial_name}: Unknown derived variable {v_name} in {gpath}")
                        else:
                            print(f"[DEBUG] {sub}/{cond}/{lv}/{trial_name}: Dataset {v_name} not found in {gpath}")
                        valid_trial = False; break

                    # Data Correction: Sign Flipping
                    if col_data is not None:
                        # ... (existing logic)
                        if sub.startswith("m1_"):
                            if "grf" in gpath and "left" in gpath and v_name == "x":
                                col_data = -col_data
                            elif "grf" in gpath and "right" in gpath and v_name == "x":
                                col_data = -col_data
                            elif "knee_angle_l" in v_name:
                                col_data = -col_data
                            elif "knee_angle_r" in v_name:
                                col_data = -col_data
                        elif sub.startswith("m2_"):
                            if "grf" in gpath and "left" in gpath and v_name == "z":
                                col_data = -col_data
                            elif "grf" in gpath and "right" in gpath and v_name == "z":
                                col_data = -col_data

                    if col_data is not None:
                        if col_data.ndim == 1: col_data = col_data[:, None]
                        block_cols.append(col_data)
                    else:
                        valid_trial = False
                        break

                if not valid_trial: break
                arr = np.concatenate(block_cols, axis=1).astype(np.float32)

                # GRF Normalization
                if "forceplate" in gpath.lower() and "grf" in gpath.lower():
                    mass = 70.0
                    if sub in f and "mass" in f[sub].attrs:
                        mass = f[sub].attrs["mass"]
                    bw = mass * 9.81
                    arr = arr / bw

                kin_q_arrays.append(arr)
                curr_trial_in.append(arr)

            if not valid_trial: continue

            # Gait Phase augmentation (placeholder - logic in main loop if needed)
            if use_gait_phase:
                pass

            if use_physical_velocity_model:
                pass

            if not curr_trial_in:
                continue

            # Synchronization
            all_objs = curr_trial_in.copy()
            extracted_outs = []
            valid_out = True
            for gpath, vars in output_vars:
                grp = trial_group
                parts = gpath.split('/')
                found_grp = True
                for p in parts:
                    if p in grp: grp = grp[p]
                    else: found_grp = False; break
                if not found_grp and not ("forceplate/grf" in gpath):
                    valid_out = False; break

                for v in vars:
                    d = None

                    if found_grp and v in grp:
                        d = grp[v][:]
                    elif "forceplate/grf" in gpath and v in ("contact_l", "contact_r", "gait_phase_l", "gait_phase_r", "stride_freq_l", "stride_freq_r"):
                        side = "left" if v.endswith("_l") else "right"
                        fz = get_data_from_group(trial_group, f"forceplate/grf/{side}/z")
                        if fz is not None:
                            fz = np.nan_to_num(fz).flatten()
                            if sub.startswith("m2_"):
                                fz = -fz
                            if "gait_phase" in v:
                                d = compute_gait_phase(fz, threshold=20.0).reshape(-1, 1)
                            elif "stride_freq" in v:
                                d = compute_stride_frequency(fz, fs=fs, threshold=20.0).reshape(-1, 1)
                            else:
                                d = get_ground_contact(fz, threshold=20.0).reshape(-1, 1)
                    elif gpath == "derived/contact_estimate" and v in _CONTACT_ESTIMATE_VARS:
                        if v.endswith("_phase"):
                            if "contact_estimate_phase_cache" not in derived_cache:
                                robot_l = trial_group["robot"]["left"]
                                robot_r = trial_group["robot"]["right"]
                                p_l, p_r = compute_contact_probability_phase(
                                    robot_l["hip_angle"][:].flatten(),
                                    robot_r["hip_angle"][:].flatten(),
                                    fs=fs,
                                )
                                derived_cache["contact_estimate_phase_cache"] = {
                                    "contact_prob_l_phase": p_l.reshape(-1, 1),
                                    "contact_prob_r_phase": p_r.reshape(-1, 1),
                                }
                            d = derived_cache["contact_estimate_phase_cache"].get(v)
                        elif v.endswith("_kin"):
                            if "contact_estimate_kin_cache" not in derived_cache:
                                kinq = trial_group["mocap"]["kin_q"]
                                p_l, p_r = compute_contact_probability_kinematics(
                                    kinq["hip_flexion_l"][:].flatten(),
                                    kinq["hip_flexion_r"][:].flatten(),
                                    kinq["knee_angle_l"][:].flatten(),
                                    kinq["knee_angle_r"][:].flatten(),
                                    fs=fs,
                                )
                                derived_cache["contact_estimate_kin_cache"] = {
                                    "contact_prob_l_kin": p_l.reshape(-1, 1),
                                    "contact_prob_r_kin": p_r.reshape(-1, 1),
                                }
                            d = derived_cache["contact_estimate_kin_cache"].get(v)
                        elif v.endswith("_fusion"):
                            if "contact_estimate_fusion_cache" not in derived_cache:
                                kinq = trial_group["mocap"]["kin_q"]
                                back_imu_grp = trial_group["robot"]["back_imu"]
                                acc_x = back_imu_grp["accel_x"][:].flatten()
                                acc_y = back_imu_grp["accel_y"][:].flatten()
                                acc_z = back_imu_grp["accel_z"][:].flatten()
                                quat = np.stack([
                                    back_imu_grp["quat_w"][:], back_imu_grp["quat_x"][:],
                                    back_imu_grp["quat_y"][:], back_imu_grp["quat_z"][:],
                                ], axis=1)
                                body_frame = estimate_body_frame_features(
                                    np.stack([acc_x, acc_y, acc_z], axis=1).astype(np.float32),
                                    np.stack([
                                        back_imu_grp["gyro_x"][:],
                                        back_imu_grp["gyro_y"][:],
                                        back_imu_grp["gyro_z"][:],
                                    ], axis=1).astype(np.float32),
                                    quat.astype(np.float32),
                                    np.zeros_like(acc_x, dtype=np.float32),
                                    fs,
                                )
                                p_l, p_r = compute_contact_probability_fusion(
                                    kinq["hip_flexion_l"][:].flatten(),
                                    kinq["hip_flexion_r"][:].flatten(),
                                    kinq["knee_angle_l"][:].flatten(),
                                    kinq["knee_angle_r"][:].flatten(),
                                    body_frame["acc_body"][:, 1].flatten(),
                                    body_frame["acc_body"][:, 2].flatten(),
                                    body_frame["gyro_body"][:, 1].flatten(),
                                    fs=fs,
                                )
                                derived_cache["contact_estimate_fusion_cache"] = {
                                    "contact_prob_l_fusion": p_l.reshape(-1, 1),
                                    "contact_prob_r_fusion": p_r.reshape(-1, 1),
                                }
                            d = derived_cache["contact_estimate_fusion_cache"].get(v)

                    if d is None:
                        valid_out = False
                        break

                    d = np.nan_to_num(d).reshape(-1, 1)
                    if lpf_cutoff is not None:
                        d = butter_lowpass_filter(d, lpf_cutoff, fs, lpf_order)
                    extracted_outs.append(d)

            if not valid_out or not extracted_outs:
                continue

            all_objs.extend(extracted_outs)
            min_len_trial = min([a.shape[0] for a in all_objs])

            in_arr_trial = np.hstack([a[:min_len_trial] for a in curr_trial_in])
            out_arr_trial = np.hstack([a[:min_len_trial] for a in extracted_outs])

            in_list_all.append(in_arr_trial)
            out_list_all.append(out_arr_trial)

    if not in_list_all:
        return [], []

    return in_list_all, out_list_all


def build_nn_dataset(
    data_path,
    sub_names, cond_names,
    input_vars, output_vars,
    time_window_input, time_window_output, stride,
    subject_selection=None,
    condition_selection=None,
    debug_plot=False,
    lpf_cutoff=None,
    lpf_order=4,
    est_tick_ranges=None,
    use_physical_velocity_model=False,
    use_gait_phase=False,
    input_lpf_cutoff=None,
    input_lpf_order=4,
    input_lpf_per_group=None,
    return_metadata=False
):
    X_list = []
    Y_list = []
    meta_list = []

    if not os.path.exists(data_path):
        pass

    with h5py.File(data_path, 'r') as f:
        for sub in sub_names:
            if subject_selection:
                if sub not in subject_selection.get('include', sub_names):
                    continue
                if sub in subject_selection.get('exclude', []):
                    continue

            if sub not in f:
                continue

            for cond in cond_names:
                if condition_selection:
                    if condition_selection.get('include') and cond not in condition_selection['include']:
                        continue
                    if cond in condition_selection.get('exclude', []):
                        continue

                if cond not in f[sub]:
                    continue

                X_trials, Y_trials = extract_condition_data_v2(
                    f, sub, cond, input_vars, output_vars,
                    lpf_cutoff=lpf_cutoff, lpf_order=lpf_order,
                    use_physical_velocity_model=use_physical_velocity_model,
                    use_gait_phase=use_gait_phase,
                    input_lpf_cutoff=input_lpf_cutoff,
                    input_lpf_order=input_lpf_order,
                    input_lpf_per_group=input_lpf_per_group
                )

                if not X_trials:
                    continue

                for X_arr, Y_arr in zip(X_trials, Y_trials):
                    if est_tick_ranges:
                        req_out_len = max(est_tick_ranges)
                    else:
                        req_out_len = time_window_output

                    min_len = time_window_input + req_out_len

                    if len(X_arr) < min_len:
                        continue

                    X_list.append(X_arr)
                    Y_list.append(Y_arr)
                    if return_metadata:
                        meta_list.append({"subject": sub, "condition": cond})

    if len(X_list) == 0:
        print("[DEBUG] X_list is empty at end of build_nn_dataset")
        if return_metadata:
            return [], [], []
        return [], []

    if return_metadata:
        return X_list, Y_list, meta_list
    return X_list, Y_list


def build_nn_dataset_multi(
    config,
    sub_names, cond_names,
    input_vars, output_vars,
    time_window_input, time_window_output, stride,
    subject_selection=None,
    condition_selection=None,
    lpf_cutoff=None,
    lpf_order=None,
    est_tick_ranges=None,
    use_physical_velocity_model=False,
    use_gait_phase=False,
    input_lpf_cutoff=None,
    input_lpf_order=4,
    input_lpf_per_group=None
):
    data_sources = {}

    if 'shared' in config and 'data_sources' in config['shared']:
        for ds_name, ds_config in config['shared']['data_sources'].items():
            prefix = ds_config.get('prefix', '')
            path = ds_config.get('path', '')
            exclude = ds_config.get('exclude_subjects', [])
            if path:
                data_sources[prefix] = {'path': path, 'exclude_subjects': exclude}

    if not data_sources:
        data_path = config.get('data_path')
        if not data_path:
            if '01_construction' in config: data_path = config['01_construction'].get('src_h5')
            if not data_path and 'shared' in config: data_path = config['shared'].get('src_h5')
            if not data_path: data_path = './combined_data_S008.h5'

        data_sources = {'': {'path': data_path, 'exclude_subjects': []}}

    X_all = []
    Y_all = []
    meta_all = []

    print(f"[DATA] Loading Multi-Source: keys={list(data_sources.keys())}")

    for prefix, src_cfg in data_sources.items():
        path = src_cfg['path']
        src_exclude = src_cfg.get('exclude_subjects', [])

        current_source_subs = []
        if sub_names:
            for s in sub_names:
                if prefix and s.startswith(prefix):
                    current_source_subs.append(s[len(prefix):])
                elif not prefix:
                    current_source_subs.append(s)

        if not current_source_subs:
            continue

        # ----- Trial-level cache: load once, filter per fold -----
        cache_key = (
            os.path.abspath(path),
            freeze_nested(sorted(current_source_subs)),
            freeze_nested(sorted(cond_names)),
            freeze_nested(input_vars),
            freeze_nested(output_vars),
            time_window_input, time_window_output, stride,
            freeze_nested(condition_selection),
            lpf_cutoff, lpf_order,
            freeze_nested(est_tick_ranges),
            use_physical_velocity_model, use_gait_phase,
            input_lpf_cutoff, input_lpf_order,
            freeze_nested(input_lpf_per_group),
            freeze_nested(sorted(src_exclude)),
        )

        if cache_key not in TRIAL_DATASET_CACHE:
            t0 = _time.time()
            src_sub_select = {'exclude': src_exclude} if src_exclude else None

            X_cached, Y_cached, meta_cached = build_nn_dataset(
                path, current_source_subs, cond_names,
                input_vars, output_vars,
                time_window_input, time_window_output, stride,
                subject_selection=src_sub_select,
                condition_selection=condition_selection,
                lpf_cutoff=lpf_cutoff,
                lpf_order=lpf_order,
                est_tick_ranges=est_tick_ranges,
                use_physical_velocity_model=use_physical_velocity_model,
                use_gait_phase=use_gait_phase,
                input_lpf_cutoff=input_lpf_cutoff,
                input_lpf_order=input_lpf_order,
                input_lpf_per_group=input_lpf_per_group,
                return_metadata=True,
            )
            TRIAL_DATASET_CACHE[cache_key] = (X_cached, Y_cached, meta_cached)
            elapsed = _time.time() - t0
            print(f"  -> Source '{prefix}': {path} | Cached {len(X_cached)} trials ({elapsed:.1f}s)")
        else:
            X_cached, Y_cached, meta_cached = TRIAL_DATASET_CACHE[cache_key]
            print(f"  -> Source '{prefix}': {path} | Cache HIT ({len(X_cached)} trials)")

        # Build fold-level subject filter (strip prefix, no src_exclude)
        fold_include = None
        fold_exclude = set()
        if subject_selection:
            if 'include' in subject_selection:
                inc = []
                for s in subject_selection['include']:
                    if prefix and s.startswith(prefix):
                        inc.append(s[len(prefix):])
                    elif not prefix:
                        inc.append(s)
                fold_include = set(inc)
            if 'exclude' in subject_selection:
                for s in subject_selection['exclude']:
                    if prefix and s.startswith(prefix):
                        fold_exclude.add(s[len(prefix):])
                    elif not prefix:
                        fold_exclude.add(s)

        # Filter cached trials by fold subject selection
        n_selected = 0
        for i, meta in enumerate(meta_cached):
            sub = meta['subject']
            if fold_include is not None and sub not in fold_include:
                continue
            if sub in fold_exclude:
                continue
            X_all.append(X_cached[i])
            Y_all.append(Y_cached[i])
            meta_all.append(meta)
            n_selected += 1

        print(f"     Selected {n_selected} trials for current fold")

    register_dataset_metadata(X_all, meta_all)
    return X_all, Y_all
