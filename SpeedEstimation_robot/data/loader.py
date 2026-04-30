import os
import time as _time
import numpy as np
import h5py

from .utils import (
    fs, butter_lowpass_filter, butter_lowpass_filter_causal, butter_highpass_filter,
    get_ground_contact, get_data_from_group,
)
from .body_frame import (
    estimate_body_frame_features,
    estimate_gravity_aligned_imu_features,
    estimate_tilt_local_imu_features,
    estimate_tilt_cf_imu_features,
)
from .model_com_speed import compute_imu_integrated_speed

# ---------------------------------------------------------------------------
# Trial-level dataset cache (avoids re-reading H5 + re-computing derived
# features across LOSO folds within the same job)
# ---------------------------------------------------------------------------
TRIAL_DATASET_CACHE = {}


def freeze_nested(value):
    """Convert nested dicts/lists to immutable tuples for use as cache keys."""
    if isinstance(value, dict):
        return tuple(sorted((k, freeze_nested(v)) for k, v in value.items()))
    if isinstance(value, (list, tuple)):
        return tuple(freeze_nested(v) for v in value)
    return value


def _apply_output_filter(data, cutoff, fs_hz, order=4, mode="zero_phase"):
    if cutoff is None:
        return data
    if mode == "none":
        return data
    if mode == "causal":
        return butter_lowpass_filter_causal(data, cutoff, fs_hz, order)
    return butter_lowpass_filter(data, cutoff, fs_hz, order)


def _should_filter_output(gpath, resolved_v):
    if gpath != 'common':
        return False
    return resolved_v in ('speed_y', 'v_Y_true')

# ---------------------------------------------------------------------------
# Body-aligned IMU features (derived/body_frame_imu)
# ---------------------------------------------------------------------------
_BODY_FRAME_IMU_VARS = frozenset([
    'accel_right', 'accel_forward', 'accel_up',
    'gyro_right', 'gyro_forward', 'gyro_up',
    'forward_accel_overground',
])
_BODY_FRAME_IMU_FIXED_VARS = _BODY_FRAME_IMU_VARS
_BODY_FRAME_IMU_STATIONARY_VARS = _BODY_FRAME_IMU_VARS
_BODY_FRAME_IMU_STATIONARY_NOBELT_VARS = _BODY_FRAME_IMU_VARS
_BODY_FRAME_IMU_ZONLY_PCA_VARS = _BODY_FRAME_IMU_VARS
_BODY_FRAME_IMU_ZONLY_PCAGATED_VARS = _BODY_FRAME_IMU_VARS
_GRAVITY_ALIGNED_IMU_ZONLY_VARS = frozenset([
    'acc_hx', 'acc_hy', 'acc_up',
    'gyro_hx', 'gyro_hy', 'gyro_up',
])
_TILT_LOCAL_IMU_2D_VARS = _GRAVITY_ALIGNED_IMU_ZONLY_VARS
_TILT_CF_IMU_2D_VARS = frozenset([
    'acc_hx', 'acc_hy', 'acc_up',
    'gyro_hx', 'gyro_hy', 'gyro_up',
    'roll_cf', 'pitch_cf',
])
_TILT_TVCF_IMU_2D_OG_VARS = _TILT_CF_IMU_2D_VARS

# ---------------------------------------------------------------------------
# IMU integrated speed (derived/imu_integrated_speed)
# ---------------------------------------------------------------------------
_IMU_INTEGRATED_SPEED_VARS = frozenset(['imu_integrated_speed'])
_IMU_INTEGRATED_SPEED_FIXED_VARS = frozenset(['imu_integrated_speed'])
_IMU_INTEGRATED_SPEED_STATIONARY_VARS = frozenset(['imu_integrated_speed'])
_IMU_INTEGRATED_SPEED_STATIONARY_NOBELT_VARS = frozenset(['imu_integrated_speed'])
_IMU_INTEGRATED_SPEED_ZONLY_PCA_VARS = frozenset(['imu_integrated_speed'])
_IMU_INTEGRATED_SPEED_ZONLY_PCAGATED_VARS = frozenset(['imu_integrated_speed'])
_IMU_HORIZONTAL_VELOCITY_ZONLY_VARS = frozenset(['vel_hx', 'vel_hy'])
_IMU_TILT_LOCAL_VELOCITY_2D_VARS = frozenset(['vel_hx', 'vel_hy'])
_IMU_TILT_CF_VELOCITY_2D_VARS = frozenset(['vel_hx', 'vel_hy'])
_IMU_TILT_TVCF_VELOCITY_2D_OG_VARS = _IMU_TILT_CF_VELOCITY_2D_VARS
_TILT_LOCAL_SUMMARY_2D_VARS = frozenset(['acc_h_mag', 'gyro_h_mag', 'vel_h_mag'])
_TILT_CF_SUMMARY_2D_VARS = _TILT_LOCAL_SUMMARY_2D_VARS
_TILT_TVCF_SUMMARY_2D_OG_VARS = _TILT_CF_SUMMARY_2D_VARS
_TREADMILL_ACC_SCALAR_VARS = frozenset(['treadmill_acc'])

# ---------------------------------------------------------------------------
# wo_grf-inspired robot-only realtime priors
# ---------------------------------------------------------------------------
_WO_PRIORS_RT_VARS = frozenset([
    'roll_cf', 'pitch_cf', 'yaw_cf', 'vel_forward_cf',
    'stride_speed_wo',
    'interaction_power_l_wo', 'interaction_power_r_wo',
    'interaction_work_pos_wo', 'interaction_work_neg_wo',
    'contact_prob_l_phase_wo', 'contact_prob_r_phase_wo',
    'contact_prob_l_fusion_wo', 'contact_prob_r_fusion_wo',
])

# ---------------------------------------------------------------------------
# Robot-only realtime biomechanical features (paper-safe)
# ---------------------------------------------------------------------------
_ROBOT_BIOMECH_RT_VARS = frozenset([
    'phase_sin_l', 'phase_cos_l', 'phase_sin_r', 'phase_cos_r',
    'cadence_l', 'cadence_r', 'cadence_mean', 'step_time_mean',
    'hip_power_l', 'hip_power_r', 'thigh_power_l', 'thigh_power_r',
    'hip_power_pos_l', 'hip_power_pos_r', 'hip_power_neg_l', 'hip_power_neg_r',
    'thigh_power_pos_l', 'thigh_power_pos_r', 'thigh_power_neg_l', 'thigh_power_neg_r',
    'power_prop_sum', 'power_brake_sum',
    'phase_diff_sin', 'phase_diff_cos',
    'hip_angle_diff', 'thigh_angle_diff', 'torque_diff',
    'hip_excursion_rms', 'thigh_excursion_rms', 'hip_sep_excursion_rms', 'thigh_sep_excursion_rms',
    'torque_rms_l', 'torque_rms_r',
    'thigh_vel_rms_l', 'thigh_vel_rms_r',
    'gyro_forward_rms', 'trunk_pitch', 'trunk_pitch_rate', 'trunk_pitch_rms', 'trunk_harmonic_ratio',
    'pseudo_speed_robot', 'step_length_proxy', 'step_length_proxy_norm', 'speed_anchor_geom',
    'motor_power_l', 'motor_power_r',
    'deflection_l', 'deflection_r',
    'deflection_vel_l', 'deflection_vel_r',
    'deflection_rms_l', 'deflection_rms_r',
    'stiffness_proxy_l', 'stiffness_proxy_r', 'stiffness_proxy_sum',
    'damping_proxy_l', 'damping_proxy_r', 'damping_proxy_sum',
    'torque_abs_ema_fast_l', 'torque_abs_ema_fast_r',
    'torque_abs_ema_slow_l', 'torque_abs_ema_slow_r',
    'torque_abs_ema_fast_sum', 'torque_abs_ema_slow_sum',
    'torque_signed_ema_fast_sum', 'torque_signed_ema_slow_sum',
    'assist_proxy_l', 'assist_proxy_r', 'assist_proxy_sum',
    'torque_stride_impulse_l', 'torque_stride_impulse_r', 'torque_stride_impulse_sum',
    'torque_preview_l_100ms', 'torque_preview_r_100ms', 'torque_preview_sum_100ms',
    'torque_preview_delta_l_100ms', 'torque_preview_delta_r_100ms', 'torque_preview_delta_sum_100ms',
    'assist_active_flag', 'assist_high_flag', 'assist_intensity_norm',
    'imu_anchor_speed_rt',
    'anchor_diff_imu_geom', 'anchor_diff_imu_pseudo', 'anchor_diff_geom_pseudo',
    'anchor_confidence', 'plateau_confidence',
    'anchor_consensus_speed', 'anchor_consensus_spread', 'highspeed_anchor_hint',
    'time_since_assist_on', 'time_since_assist_off',
    'assist_duty_cycle_1s', 'assist_duty_cycle_3s', 'assist_duty_cycle_5s',
    'positive_work_ema_fast', 'positive_work_ema_slow',
    'work_norm_cadence', 'work_norm_excursion', 'work_norm_step_time',
    'trunk_thigh_coherence', 'trunk_hip_coherence',
    'excursion_residual',
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


def _causal_ema(signal: np.ndarray, alpha: float) -> np.ndarray:
    """Simple causal EMA with fixed alpha."""
    x = np.nan_to_num(np.asarray(signal, dtype=np.float32).flatten())
    y = np.zeros_like(x)
    if len(x) == 0:
        return y
    y[0] = x[0]
    for i in range(1, len(x)):
        y[i] = alpha * x[i] + (1.0 - alpha) * y[i - 1]
    return y


def _alpha_from_tau(tau_s: float, fs_hz: float) -> float:
    tau_s = max(float(tau_s), 1e-3)
    return 1.0 - np.exp(-1.0 / (tau_s * fs_hz))


def _causal_derivative(signal: np.ndarray, fs_hz: float) -> np.ndarray:
    """Backward-difference derivative for online-friendly preprocessing."""
    x = np.nan_to_num(np.asarray(signal, dtype=np.float32).flatten())
    d = np.zeros_like(x)
    if len(x) > 1:
        d[1:] = (x[1:] - x[:-1]) * float(fs_hz)
    return d


def _causal_running_rms(signal: np.ndarray, window_samples: int) -> np.ndarray:
    """Causal running RMS computed from past-only samples."""
    x = np.nan_to_num(np.asarray(signal, dtype=np.float32).flatten())
    w = max(int(window_samples), 1)
    out = np.zeros_like(x)
    sq_cum = np.concatenate([[0.0], np.cumsum(x.astype(np.float64) ** 2)])
    for i in range(len(x)):
        start = max(0, i - w + 1)
        n = i - start + 1
        power = (sq_cum[i + 1] - sq_cum[start]) / max(n, 1)
        out[i] = np.sqrt(max(power, 0.0))
    return out.astype(np.float32)


def _causal_lpf_1d(signal_1d, cutoff_hz, fs_hz, order=4):
    x = np.asarray(signal_1d, dtype=np.float32).flatten()
    return butter_lowpass_filter_causal(x, cutoff_hz, fs_hz, order).astype(np.float32)


def _causal_sigmoid(x):
    x = np.asarray(x, dtype=np.float32)
    return 1.0 / (1.0 + np.exp(-np.clip(x, -40.0, 40.0)))


def _causal_stride_impulse(signal_abs: np.ndarray, phase_unwrapped: np.ndarray, fs_hz: float) -> np.ndarray:
    """Causal within-stride impulse using monotonic unwrapped phase."""
    x = np.nan_to_num(np.asarray(signal_abs, dtype=np.float32).flatten())
    phase = np.nan_to_num(np.asarray(phase_unwrapped, dtype=np.float32).flatten())
    if len(x) == 0:
        return np.zeros_like(x)
    out = np.zeros_like(x, dtype=np.float32)
    cycle_idx = np.floor((phase - phase[0]) / (2.0 * np.pi)).astype(np.int64)
    cum = 0.0
    prev_cycle = cycle_idx[0]
    dt = 1.0 / max(float(fs_hz), 1.0)
    for i in range(len(x)):
        if cycle_idx[i] != prev_cycle:
            cum = 0.0
            prev_cycle = cycle_idx[i]
        cum += float(x[i]) * dt
        out[i] = cum
    return out


def compute_wo_priors_rt_features(
    hip_l, hip_r, thigh_l, thigh_r, torque_l, torque_r,
    imu_acc_local, imu_gyro_local, height_m, fs_hz=100.0,
    treadmill_acc=None,
):
    """wo_grf-inspired deployable priors using only robot sensors + trunk IMU."""
    hip_l = np.asarray(hip_l, dtype=np.float32).flatten()
    hip_r = np.asarray(hip_r, dtype=np.float32).flatten()
    thigh_l = np.asarray(thigh_l, dtype=np.float32).flatten()
    thigh_r = np.asarray(thigh_r, dtype=np.float32).flatten()
    torque_l = np.asarray(torque_l, dtype=np.float32).flatten()
    torque_r = np.asarray(torque_r, dtype=np.float32).flatten()
    raw_ax = np.asarray(imu_acc_local[:, 0], dtype=np.float32).flatten()
    raw_ay = np.asarray(imu_acc_local[:, 1], dtype=np.float32).flatten()
    raw_az = np.asarray(imu_acc_local[:, 2], dtype=np.float32).flatten()
    raw_gx = np.asarray(imu_gyro_local[:, 0], dtype=np.float32).flatten()
    raw_gy = np.asarray(imu_gyro_local[:, 1], dtype=np.float32).flatten()
    raw_gz = np.asarray(imu_gyro_local[:, 2], dtype=np.float32).flatten()
    n = len(hip_l)
    dt = 1.0 / float(fs_hz)

    alpha = 0.98
    roll = np.zeros(n, dtype=np.float32)
    pitch = np.zeros(n, dtype=np.float32)
    yaw = np.zeros(n, dtype=np.float32)
    curr_r = curr_p = curr_y = 0.0
    for i in range(n):
        r_a = np.arctan2(raw_ay[i], raw_az[i])
        p_a = np.arctan2(-raw_ax[i], np.sqrt(raw_ay[i] ** 2 + raw_az[i] ** 2) + 1e-8)
        curr_r = alpha * (curr_r + raw_gx[i] * dt) + (1.0 - alpha) * r_a
        curr_p = alpha * (curr_p + raw_gy[i] * dt) + (1.0 - alpha) * p_a
        curr_y = curr_y + raw_gz[i] * dt
        roll[i] = curr_r
        pitch[i] = curr_p
        yaw[i] = curr_y

    a_lin = raw_ax + 9.81 * np.sin(pitch)
    if treadmill_acc is not None:
        a_lin = a_lin + np.asarray(treadmill_acc, dtype=np.float32).flatten()
    vel_forward_cf = butter_highpass_filter(np.cumsum(a_lin) * dt, 0.1, fs_hz, 2).astype(np.float32)

    phase_diff = np.deg2rad(hip_l - hip_r).astype(np.float32)
    excursion = _causal_lpf_1d(np.abs(phase_diff), 1.0, fs_hz, order=2)
    cadence_proxy = _causal_lpf_1d(np.abs(_causal_derivative(phase_diff, fs_hz)), 1.5, fs_hz, order=2)
    leg_length_m = max(float(height_m) * 0.53, 0.5)
    step_length_proxy = 0.5 * leg_length_m * excursion
    stride_speed_wo = _causal_lpf_1d(step_length_proxy * cadence_proxy, 0.5, fs_hz, order=4)

    hip_l_rad = np.deg2rad(hip_l).astype(np.float32)
    hip_r_rad = np.deg2rad(hip_r).astype(np.float32)
    omega_l = _causal_lpf_1d(_causal_derivative(hip_l_rad, fs_hz), 6.0, fs_hz, order=2)
    omega_r = _causal_lpf_1d(_causal_derivative(hip_r_rad, fs_hz), 6.0, fs_hz, order=2)
    interaction_power_l = _causal_lpf_1d(torque_l * omega_l, 2.0, fs_hz, order=2)
    interaction_power_r = _causal_lpf_1d(torque_r * omega_r, 2.0, fs_hz, order=2)
    alpha_work = float(np.exp(-dt / 0.75))
    interaction_work_pos = np.zeros(n, dtype=np.float32)
    interaction_work_neg = np.zeros(n, dtype=np.float32)
    for i in range(1, n):
        pos_i = max(float(interaction_power_l[i]), 0.0) + max(float(interaction_power_r[i]), 0.0)
        neg_i = max(float(-interaction_power_l[i]), 0.0) + max(float(-interaction_power_r[i]), 0.0)
        interaction_work_pos[i] = alpha_work * interaction_work_pos[i - 1] + pos_i * dt
        interaction_work_neg[i] = alpha_work * interaction_work_neg[i - 1] + neg_i * dt

    dhip_l = _causal_derivative(hip_l, fs_hz)
    dhip_r = _causal_derivative(hip_r, fs_hz)
    dthigh_l = _causal_derivative(thigh_l, fs_hz)
    dthigh_r = _causal_derivative(thigh_r, fs_hz)

    score_phase_l = -0.12 * (hip_r - hip_l)
    contact_prob_l_phase = np.clip(_causal_lpf_1d(_causal_sigmoid(score_phase_l), 2.0, fs_hz, order=2), 1e-3, 1.0 - 1e-3)
    contact_prob_r_phase = 1.0 - contact_prob_l_phase

    phase_term = -0.10 * (hip_r - hip_l) - 0.04 * (thigh_r - thigh_l)
    vel_term = -0.015 * ((dhip_l - dhip_r) + 0.5 * (dthigh_l - dthigh_r))
    p_left_kin = np.clip(_causal_lpf_1d(_causal_sigmoid(phase_term + vel_term), 2.0, fs_hz, order=2), 1e-3, 1.0 - 1e-3)
    motion_energy = _causal_lpf_1d(np.abs(raw_ax) + 0.5 * np.abs(raw_az) + 0.4 * np.abs(raw_gy), 4.0, fs_hz, order=2)
    stance_conf = np.clip(_causal_sigmoid(1.5 - 0.35 * motion_energy), 0.1, 0.95)
    contact_prob_l_fusion = np.clip(_causal_lpf_1d(stance_conf * p_left_kin + (1.0 - stance_conf) * 0.5, 2.0, fs_hz, order=2), 1e-3, 1.0 - 1e-3)
    contact_prob_r_fusion = 1.0 - contact_prob_l_fusion

    return {
        'roll_cf': roll.astype(np.float32),
        'pitch_cf': pitch.astype(np.float32),
        'yaw_cf': yaw.astype(np.float32),
        'vel_forward_cf': vel_forward_cf.astype(np.float32),
        'stride_speed_wo': stride_speed_wo.astype(np.float32),
        'interaction_power_l_wo': interaction_power_l.astype(np.float32),
        'interaction_power_r_wo': interaction_power_r.astype(np.float32),
        'interaction_work_pos_wo': interaction_work_pos.astype(np.float32),
        'interaction_work_neg_wo': interaction_work_neg.astype(np.float32),
        'contact_prob_l_phase_wo': contact_prob_l_phase.astype(np.float32),
        'contact_prob_r_phase_wo': contact_prob_r_phase.astype(np.float32),
        'contact_prob_l_fusion_wo': contact_prob_l_fusion.astype(np.float32),
        'contact_prob_r_fusion_wo': contact_prob_r_fusion.astype(np.float32),
    }


def _compute_treadmill_acc_mean(trial_group, fs_hz: float) -> np.ndarray:
    v_l = get_data_from_group(trial_group, 'treadmill/left/speed_leftbelt')
    v_r = get_data_from_group(trial_group, 'treadmill/right/speed_rightbelt')
    if v_l is None or v_r is None:
        return np.zeros(0, dtype=np.float32)
    v_mean = (np.asarray(v_l).flatten() + np.asarray(v_r).flatten()) / 2.0
    return _causal_derivative(v_mean.astype(np.float32), fs_hz).astype(np.float32)


def _estimate_subject_height_weight(f, sub):
    """Return subject scalars from sub_info datasets/attrs with safe defaults."""
    height_m = 1.70
    weight_kg = 70.0
    if sub in f and 'sub_info' in f[sub]:
        si = f[sub]['sub_info']
        if 'height' in si:
            raw_h = float(si['height'][()])
            height_m = raw_h / 1000.0 if raw_h > 10.0 else raw_h
        elif 'height' in si.attrs:
            raw_h = float(si.attrs['height'])
            height_m = raw_h / 1000.0 if raw_h > 10.0 else raw_h
        if 'weight' in si:
            weight_kg = float(si['weight'][()])
        elif 'weight' in si.attrs:
            weight_kg = float(si.attrs['weight'])
    return max(height_m, 1.2), max(weight_kg, 35.0)


def _resolve_output_var_name(grp, v_name):
    """Map project-level target aliases to actual H5 paths."""
    if v_name in grp:
        return v_name
    alias_map = {
        "speed_y": "v_Y_true",
        "speed_z": "v_Z_true",
        "accel_y": "a_Y_true",
        "accel_z": "a_Z_true",
    }
    alias = alias_map.get(v_name)
    if alias and alias in grp:
        return alias
    return None


def compute_robot_biomech_rt_features(
    hip_l, hip_r, thigh_l, thigh_r, torque_l, torque_r,
    motor_l, motor_r,
    imu_acc_local, imu_gyro_local, imu_quat_wxyz,
    height_m, weight_kg, treadmill_acc=None, fs_hz=100.0
):
    """Robot-only realtime features from joints + trunk IMU + sub_info.

    This helper must not read GRF, forceplate, mocap, or output-speed signals.
    All smoothing uses causal EMA / running RMS only.
    """
    hip_l = np.nan_to_num(np.asarray(hip_l, dtype=np.float32).flatten())
    hip_r = np.nan_to_num(np.asarray(hip_r, dtype=np.float32).flatten())
    thigh_l = np.nan_to_num(np.asarray(thigh_l, dtype=np.float32).flatten())
    thigh_r = np.nan_to_num(np.asarray(thigh_r, dtype=np.float32).flatten())
    torque_l = np.nan_to_num(np.asarray(torque_l, dtype=np.float32).flatten())
    torque_r = np.nan_to_num(np.asarray(torque_r, dtype=np.float32).flatten())
    motor_l = np.nan_to_num(np.asarray(motor_l, dtype=np.float32).flatten())
    motor_r = np.nan_to_num(np.asarray(motor_r, dtype=np.float32).flatten())

    hip_dot_l = _causal_derivative(hip_l, fs_hz)
    hip_dot_r = _causal_derivative(hip_r, fs_hz)
    thigh_dot_l = _causal_derivative(thigh_l, fs_hz)
    thigh_dot_r = _causal_derivative(thigh_r, fs_hz)
    motor_dot_l = _causal_derivative(motor_l, fs_hz)
    motor_dot_r = _causal_derivative(motor_r, fs_hz)

    alpha_center = _alpha_from_tau(0.6, fs_hz)
    thigh_center_l = thigh_l - _causal_ema(thigh_l, alpha_center)
    thigh_center_r = thigh_r - _causal_ema(thigh_r, alpha_center)
    hip_center_l = hip_l - _causal_ema(hip_l, alpha_center)
    hip_center_r = hip_r - _causal_ema(hip_r, alpha_center)

    thigh_amp_l = np.maximum(_causal_running_rms(thigh_center_l, int(0.6 * fs_hz)), 5.0)
    thigh_amp_r = np.maximum(_causal_running_rms(thigh_center_r, int(0.6 * fs_hz)), 5.0)
    hip_amp_l = np.maximum(_causal_running_rms(hip_center_l, int(0.6 * fs_hz)), 3.0)
    hip_amp_r = np.maximum(_causal_running_rms(hip_center_r, int(0.6 * fs_hz)), 3.0)
    thigh_vel_scale_l = np.maximum(_causal_running_rms(thigh_dot_l, int(0.6 * fs_hz)), 20.0)
    thigh_vel_scale_r = np.maximum(_causal_running_rms(thigh_dot_r, int(0.6 * fs_hz)), 20.0)

    phase_l = np.unwrap(np.arctan2(thigh_dot_l / thigh_vel_scale_l, thigh_center_l / thigh_amp_l))
    phase_r = np.unwrap(np.arctan2(thigh_dot_r / thigh_vel_scale_r, thigh_center_r / thigh_amp_r))
    phase_sin_l = np.sin(phase_l).astype(np.float32)
    phase_cos_l = np.cos(phase_l).astype(np.float32)
    phase_sin_r = np.sin(phase_r).astype(np.float32)
    phase_cos_r = np.cos(phase_r).astype(np.float32)

    phase_rate_l = np.clip(_causal_derivative(phase_l.astype(np.float32), fs_hz) / (2.0 * np.pi), 0.0, 4.0)
    phase_rate_r = np.clip(_causal_derivative(phase_r.astype(np.float32), fs_hz) / (2.0 * np.pi), 0.0, 4.0)
    cadence_l = _causal_ema(phase_rate_l.astype(np.float32), _alpha_from_tau(0.25, fs_hz))
    cadence_r = _causal_ema(phase_rate_r.astype(np.float32), _alpha_from_tau(0.25, fs_hz))
    cadence_mean = (0.5 * (cadence_l + cadence_r)).astype(np.float32)
    step_time_mean = (1.0 / np.clip(cadence_mean, 0.15, 4.0)).astype(np.float32)

    weight_norm = max(weight_kg * 9.81, 1.0)
    hip_power_l = (torque_l * np.deg2rad(hip_dot_l) / weight_norm).astype(np.float32)
    hip_power_r = (torque_r * np.deg2rad(hip_dot_r) / weight_norm).astype(np.float32)
    thigh_power_l = (torque_l * np.deg2rad(thigh_dot_l) / weight_norm).astype(np.float32)
    thigh_power_r = (torque_r * np.deg2rad(thigh_dot_r) / weight_norm).astype(np.float32)
    motor_power_l = (torque_l * np.deg2rad(motor_dot_l) / weight_norm).astype(np.float32)
    motor_power_r = (torque_r * np.deg2rad(motor_dot_r) / weight_norm).astype(np.float32)
    hip_power_pos_l = np.maximum(hip_power_l, 0.0).astype(np.float32)
    hip_power_pos_r = np.maximum(hip_power_r, 0.0).astype(np.float32)
    hip_power_neg_l = np.maximum(-hip_power_l, 0.0).astype(np.float32)
    hip_power_neg_r = np.maximum(-hip_power_r, 0.0).astype(np.float32)
    thigh_power_pos_l = np.maximum(thigh_power_l, 0.0).astype(np.float32)
    thigh_power_pos_r = np.maximum(thigh_power_r, 0.0).astype(np.float32)
    thigh_power_neg_l = np.maximum(-thigh_power_l, 0.0).astype(np.float32)
    thigh_power_neg_r = np.maximum(-thigh_power_r, 0.0).astype(np.float32)
    power_prop_sum = (hip_power_pos_l + hip_power_pos_r + thigh_power_pos_l + thigh_power_pos_r).astype(np.float32)
    power_brake_sum = (hip_power_neg_l + hip_power_neg_r + thigh_power_neg_l + thigh_power_neg_r).astype(np.float32)

    phase_delta = phase_l - phase_r
    phase_diff_sin = np.sin(phase_delta).astype(np.float32)
    phase_diff_cos = np.cos(phase_delta).astype(np.float32)
    hip_angle_diff = np.deg2rad(hip_l - hip_r).astype(np.float32)
    thigh_angle_diff = np.deg2rad(thigh_l - thigh_r).astype(np.float32)
    torque_diff = ((torque_l - torque_r) / weight_norm).astype(np.float32)

    torque_rms_l = _causal_running_rms(torque_l / weight_norm, int(0.5 * fs_hz))
    torque_rms_r = _causal_running_rms(torque_r / weight_norm, int(0.5 * fs_hz))
    thigh_vel_rms_l = _causal_running_rms(np.deg2rad(thigh_dot_l), int(0.5 * fs_hz))
    thigh_vel_rms_r = _causal_running_rms(np.deg2rad(thigh_dot_r), int(0.5 * fs_hz))
    hip_excursion_rms = (
        0.5 * (
            _causal_running_rms(np.deg2rad(hip_center_l), int(0.6 * fs_hz)) +
            _causal_running_rms(np.deg2rad(hip_center_r), int(0.6 * fs_hz))
        )
    ).astype(np.float32)
    thigh_excursion_rms = (
        0.5 * (
            _causal_running_rms(np.deg2rad(thigh_center_l), int(0.6 * fs_hz)) +
            _causal_running_rms(np.deg2rad(thigh_center_r), int(0.6 * fs_hz))
        )
    ).astype(np.float32)
    hip_sep_excursion_rms = _causal_running_rms(np.deg2rad(hip_l - hip_r), int(0.6 * fs_hz)).astype(np.float32)
    thigh_sep_excursion_rms = _causal_running_rms(np.deg2rad(thigh_l - thigh_r), int(0.6 * fs_hz)).astype(np.float32)

    deflection_l = np.deg2rad(motor_l - hip_l).astype(np.float32)
    deflection_r = np.deg2rad(motor_r - hip_r).astype(np.float32)
    deflection_vel_l = _causal_derivative(deflection_l, fs_hz).astype(np.float32)
    deflection_vel_r = _causal_derivative(deflection_r, fs_hz).astype(np.float32)
    deflection_rms_l = _causal_running_rms(deflection_l, int(0.5 * fs_hz)).astype(np.float32)
    deflection_rms_r = _causal_running_rms(deflection_r, int(0.5 * fs_hz)).astype(np.float32)
    alpha_fast = _alpha_from_tau(0.15, fs_hz)
    alpha_slow = _alpha_from_tau(1.0, fs_hz)
    alpha_mid = _alpha_from_tau(0.5, fs_hz)
    stiff_eps = 0.03
    damp_eps = 0.10
    stiffness_proxy_l = _causal_ema(
        np.clip((torque_l / weight_norm) / np.maximum(np.abs(deflection_l), stiff_eps), -3.0, 3.0),
        alpha_mid,
    ).astype(np.float32)
    stiffness_proxy_r = _causal_ema(
        np.clip((torque_r / weight_norm) / np.maximum(np.abs(deflection_r), stiff_eps), -3.0, 3.0),
        alpha_mid,
    ).astype(np.float32)
    damping_proxy_l = _causal_ema(
        np.clip((torque_l / weight_norm) / np.maximum(np.abs(deflection_vel_l), damp_eps), -3.0, 3.0),
        alpha_mid,
    ).astype(np.float32)
    damping_proxy_r = _causal_ema(
        np.clip((torque_r / weight_norm) / np.maximum(np.abs(deflection_vel_r), damp_eps), -3.0, 3.0),
        alpha_mid,
    ).astype(np.float32)
    stiffness_proxy_sum = (stiffness_proxy_l + stiffness_proxy_r).astype(np.float32)
    damping_proxy_sum = (damping_proxy_l + damping_proxy_r).astype(np.float32)

    torque_abs_norm_l = np.abs(torque_l) / weight_norm
    torque_abs_norm_r = np.abs(torque_r) / weight_norm
    torque_signed_norm_l = torque_l / weight_norm
    torque_signed_norm_r = torque_r / weight_norm
    torque_abs_ema_fast_l = _causal_ema(torque_abs_norm_l, alpha_fast).astype(np.float32)
    torque_abs_ema_fast_r = _causal_ema(torque_abs_norm_r, alpha_fast).astype(np.float32)
    torque_abs_ema_slow_l = _causal_ema(torque_abs_norm_l, alpha_slow).astype(np.float32)
    torque_abs_ema_slow_r = _causal_ema(torque_abs_norm_r, alpha_slow).astype(np.float32)
    torque_abs_ema_fast_sum = (torque_abs_ema_fast_l + torque_abs_ema_fast_r).astype(np.float32)
    torque_abs_ema_slow_sum = (torque_abs_ema_slow_l + torque_abs_ema_slow_r).astype(np.float32)
    torque_signed_ema_fast_sum = (
        _causal_ema(torque_signed_norm_l, alpha_fast) + _causal_ema(torque_signed_norm_r, alpha_fast)
    ).astype(np.float32)
    torque_signed_ema_slow_sum = (
        _causal_ema(torque_signed_norm_l, alpha_slow) + _causal_ema(torque_signed_norm_r, alpha_slow)
    ).astype(np.float32)
    assist_proxy_l = _causal_ema(np.abs(deflection_l) * torque_abs_norm_l, alpha_mid).astype(np.float32)
    assist_proxy_r = _causal_ema(np.abs(deflection_r) * torque_abs_norm_r, alpha_mid).astype(np.float32)
    assist_proxy_sum = (assist_proxy_l + assist_proxy_r).astype(np.float32)
    torque_stride_impulse_l = _causal_stride_impulse(torque_abs_norm_l, phase_l, fs_hz).astype(np.float32)
    torque_stride_impulse_r = _causal_stride_impulse(torque_abs_norm_r, phase_r, fs_hz).astype(np.float32)
    torque_stride_impulse_sum = (torque_stride_impulse_l + torque_stride_impulse_r).astype(np.float32)

    # Guarded preview proxy: assume future assist profile is available only after
    # assistance is already active, not before onset. Before activation, keep the
    # preview equal to the current torque state instead of leaking onset timing.
    preview_steps = max(1, int(round(0.10 * fs_hz)))
    active_thresh = 0.0025
    active_now = (torque_abs_ema_fast_sum > active_thresh).astype(np.float32)

    def _guarded_preview(sig):
        sig = np.asarray(sig, dtype=np.float32)
        fut = np.empty_like(sig)
        fut[:-preview_steps] = sig[preview_steps:]
        fut[-preview_steps:] = sig[-1]
        return (active_now * fut + (1.0 - active_now) * sig).astype(np.float32)

    torque_preview_l_100ms = _guarded_preview(torque_signed_norm_l)
    torque_preview_r_100ms = _guarded_preview(torque_signed_norm_r)
    torque_preview_sum_100ms = (torque_preview_l_100ms + torque_preview_r_100ms).astype(np.float32)
    torque_preview_delta_l_100ms = (torque_preview_l_100ms - torque_signed_norm_l.astype(np.float32)).astype(np.float32)
    torque_preview_delta_r_100ms = (torque_preview_r_100ms - torque_signed_norm_r.astype(np.float32)).astype(np.float32)
    torque_preview_delta_sum_100ms = (
        torque_preview_delta_l_100ms + torque_preview_delta_r_100ms
    ).astype(np.float32)
    assist_active_flag = (torque_abs_ema_slow_sum > 0.010).astype(np.float32)
    assist_high_flag = (torque_abs_ema_slow_sum > 0.025).astype(np.float32)
    assist_intensity_norm = np.clip((torque_abs_ema_slow_sum - 0.005) / 0.040, 0.0, 1.0).astype(np.float32)

    if treadmill_acc is None or len(np.asarray(treadmill_acc).reshape(-1)) != len(hip_l):
        treadmill_acc = np.zeros(len(hip_l), dtype=np.float32)
    else:
        treadmill_acc = np.nan_to_num(np.asarray(treadmill_acc, dtype=np.float32).reshape(-1))

    body_frame = estimate_body_frame_features(
        imu_acc_local, imu_gyro_local, imu_quat_wxyz,
        treadmill_acc, fs_hz
    )
    gyro_forward = body_frame['gyro_body'][:, 1].astype(np.float32)
    gyro_forward_rms = _causal_running_rms(gyro_forward, int(0.5 * fs_hz))
    quat = np.nan_to_num(np.asarray(imu_quat_wxyz, dtype=np.float32))
    w, x, y, z = quat[:, 0], quat[:, 1], quat[:, 2], quat[:, 3]
    pitch_arg = np.clip(2.0 * (w * y - z * x), -1.0, 1.0)
    trunk_pitch = np.arcsin(pitch_arg).astype(np.float32)
    trunk_pitch_rate = _causal_derivative(trunk_pitch, fs_hz).astype(np.float32)
    trunk_pitch_rms = _causal_running_rms(trunk_pitch, int(0.6 * fs_hz)).astype(np.float32)
    pitch_lp = _causal_ema(trunk_pitch, _alpha_from_tau(0.30, fs_hz)).astype(np.float32)
    pitch_hp = (trunk_pitch - pitch_lp).astype(np.float32)
    pitch_lp_rms = _causal_running_rms(pitch_lp, int(0.6 * fs_hz)).astype(np.float32)
    pitch_hp_rms = _causal_running_rms(pitch_hp, int(0.6 * fs_hz)).astype(np.float32)
    trunk_harmonic_ratio = (pitch_lp_rms / np.maximum(pitch_hp_rms, 1e-4)).astype(np.float32)

    leg_length_m = max(0.53 * float(height_m), 0.5)
    excursion = _causal_running_rms(np.deg2rad(thigh_l - thigh_r), int(0.5 * fs_hz))
    step_length_proxy = (leg_length_m * excursion).astype(np.float32)
    step_length_proxy_norm = (excursion).astype(np.float32)
    speed_anchor_geom = (cadence_mean * step_length_proxy).astype(np.float32)
    pseudo_speed_robot = _causal_ema(
        speed_anchor_geom,
        _alpha_from_tau(0.25, fs_hz)
    ).astype(np.float32)
    imu_anchor_speed = compute_imu_integrated_speed(
        body_frame['forward_accel_overground'],
        fs_hz,
        alpha=0.998,
        lpf_cutoff=0.5,
        lpf_order=4,
    ).astype(np.float32)

    anchor_diff_imu_geom = (imu_anchor_speed - speed_anchor_geom).astype(np.float32)
    anchor_diff_imu_pseudo = (imu_anchor_speed - pseudo_speed_robot).astype(np.float32)
    anchor_diff_geom_pseudo = (speed_anchor_geom - pseudo_speed_robot).astype(np.float32)
    anchor_disagreement = (
        np.abs(anchor_diff_imu_geom) +
        np.abs(anchor_diff_imu_pseudo) +
        np.abs(anchor_diff_geom_pseudo)
    ).astype(np.float32)
    anchor_confidence = (1.0 / (1.0 + 2.5 * anchor_disagreement)).astype(np.float32)
    plateau_confidence = (
        anchor_confidence *
        np.clip(trunk_harmonic_ratio / np.maximum(trunk_harmonic_ratio + 1.0, 1e-4), 0.0, 1.0)
    ).astype(np.float32)
    anchor_stack = np.stack(
        [imu_anchor_speed, speed_anchor_geom.astype(np.float32), pseudo_speed_robot.astype(np.float32)],
        axis=0,
    )
    anchor_consensus_speed = np.median(anchor_stack, axis=0).astype(np.float32)
    anchor_consensus_spread = np.mean(
        np.abs(anchor_stack - anchor_consensus_speed[None, :]),
        axis=0,
    ).astype(np.float32)
    highspeed_anchor_hint = np.clip((anchor_consensus_speed - 1.00) / 0.25, 0.0, 1.0).astype(np.float32)

    assist_active_int = assist_active_flag.astype(np.int32)
    time_since_assist_on = np.zeros_like(assist_active_flag, dtype=np.float32)
    time_since_assist_off = np.zeros_like(assist_active_flag, dtype=np.float32)
    on_count = 0
    off_count = 0
    for i in range(len(assist_active_int)):
        if assist_active_int[i]:
            on_count += 1
            off_count = 0
        else:
            off_count += 1
            on_count = 0
        time_since_assist_on[i] = on_count / max(float(fs_hz), 1.0)
        time_since_assist_off[i] = off_count / max(float(fs_hz), 1.0)

    assist_duty_cycle_1s = _causal_ema(assist_active_flag, _alpha_from_tau(1.0, fs_hz)).astype(np.float32)
    assist_duty_cycle_3s = _causal_ema(assist_active_flag, _alpha_from_tau(3.0, fs_hz)).astype(np.float32)
    assist_duty_cycle_5s = _causal_ema(assist_active_flag, _alpha_from_tau(5.0, fs_hz)).astype(np.float32)

    positive_work = np.maximum(power_prop_sum, 0.0).astype(np.float32)
    positive_work_ema_fast = _causal_ema(positive_work, _alpha_from_tau(0.30, fs_hz)).astype(np.float32)
    positive_work_ema_slow = _causal_ema(positive_work, _alpha_from_tau(2.00, fs_hz)).astype(np.float32)
    work_norm_cadence = (
        positive_work_ema_slow / np.maximum(cadence_mean, 0.20)
    ).astype(np.float32)
    work_norm_excursion = (
        positive_work_ema_slow / np.maximum(thigh_excursion_rms, 0.03)
    ).astype(np.float32)
    work_norm_step_time = (
        positive_work_ema_slow / np.maximum(step_time_mean, 0.25)
    ).astype(np.float32)

    thigh_vel_mean = 0.5 * (np.deg2rad(thigh_dot_l) + np.deg2rad(thigh_dot_r))
    hip_vel_mean = 0.5 * (np.deg2rad(hip_dot_l) + np.deg2rad(hip_dot_r))
    trunk_pitch_rate_rms = np.maximum(_causal_running_rms(trunk_pitch_rate, int(0.5 * fs_hz)), 1e-3)
    thigh_vel_mean_rms = np.maximum(_causal_running_rms(thigh_vel_mean, int(0.5 * fs_hz)), 1e-3)
    hip_vel_mean_rms = np.maximum(_causal_running_rms(hip_vel_mean, int(0.5 * fs_hz)), 1e-3)
    trunk_thigh_coherence = _causal_ema(
        (trunk_pitch_rate * thigh_vel_mean) / (trunk_pitch_rate_rms * thigh_vel_mean_rms),
        _alpha_from_tau(0.5, fs_hz),
    ).astype(np.float32)
    trunk_hip_coherence = _causal_ema(
        (trunk_pitch_rate * hip_vel_mean) / (trunk_pitch_rate_rms * hip_vel_mean_rms),
        _alpha_from_tau(0.5, fs_hz),
    ).astype(np.float32)

    expected_excursion = np.clip(
        pseudo_speed_robot / np.maximum(leg_length_m * cadence_mean, 1e-3),
        0.0,
        2.0,
    ).astype(np.float32)
    excursion_residual = (step_length_proxy_norm - expected_excursion).astype(np.float32)

    return {
        'phase_sin_l': phase_sin_l,
        'phase_cos_l': phase_cos_l,
        'phase_sin_r': phase_sin_r,
        'phase_cos_r': phase_cos_r,
        'cadence_l': cadence_l.astype(np.float32),
        'cadence_r': cadence_r.astype(np.float32),
        'cadence_mean': cadence_mean,
        'step_time_mean': step_time_mean,
        'hip_power_l': hip_power_l,
        'hip_power_r': hip_power_r,
        'thigh_power_l': thigh_power_l,
        'thigh_power_r': thigh_power_r,
        'hip_power_pos_l': hip_power_pos_l,
        'hip_power_pos_r': hip_power_pos_r,
        'hip_power_neg_l': hip_power_neg_l,
        'hip_power_neg_r': hip_power_neg_r,
        'thigh_power_pos_l': thigh_power_pos_l,
        'thigh_power_pos_r': thigh_power_pos_r,
        'thigh_power_neg_l': thigh_power_neg_l,
        'thigh_power_neg_r': thigh_power_neg_r,
        'power_prop_sum': power_prop_sum,
        'power_brake_sum': power_brake_sum,
        'phase_diff_sin': phase_diff_sin,
        'phase_diff_cos': phase_diff_cos,
        'hip_angle_diff': hip_angle_diff,
        'thigh_angle_diff': thigh_angle_diff,
        'torque_diff': torque_diff,
        'hip_excursion_rms': hip_excursion_rms,
        'thigh_excursion_rms': thigh_excursion_rms,
        'hip_sep_excursion_rms': hip_sep_excursion_rms,
        'thigh_sep_excursion_rms': thigh_sep_excursion_rms,
        'torque_rms_l': torque_rms_l.astype(np.float32),
        'torque_rms_r': torque_rms_r.astype(np.float32),
        'thigh_vel_rms_l': thigh_vel_rms_l.astype(np.float32),
        'thigh_vel_rms_r': thigh_vel_rms_r.astype(np.float32),
        'gyro_forward_rms': gyro_forward_rms.astype(np.float32),
        'trunk_pitch': trunk_pitch,
        'trunk_pitch_rate': trunk_pitch_rate,
        'trunk_pitch_rms': trunk_pitch_rms,
        'trunk_harmonic_ratio': trunk_harmonic_ratio,
        'imu_anchor_speed_rt': imu_anchor_speed,
        'pseudo_speed_robot': pseudo_speed_robot,
        'step_length_proxy': step_length_proxy,
        'step_length_proxy_norm': step_length_proxy_norm,
        'speed_anchor_geom': speed_anchor_geom,
        'motor_power_l': motor_power_l,
        'motor_power_r': motor_power_r,
        'deflection_l': deflection_l,
        'deflection_r': deflection_r,
        'deflection_vel_l': deflection_vel_l,
        'deflection_vel_r': deflection_vel_r,
        'deflection_rms_l': deflection_rms_l,
        'deflection_rms_r': deflection_rms_r,
        'stiffness_proxy_l': stiffness_proxy_l,
        'stiffness_proxy_r': stiffness_proxy_r,
        'stiffness_proxy_sum': stiffness_proxy_sum,
        'damping_proxy_l': damping_proxy_l,
        'damping_proxy_r': damping_proxy_r,
        'damping_proxy_sum': damping_proxy_sum,
        'torque_abs_ema_fast_l': torque_abs_ema_fast_l,
        'torque_abs_ema_fast_r': torque_abs_ema_fast_r,
        'torque_abs_ema_slow_l': torque_abs_ema_slow_l,
        'torque_abs_ema_slow_r': torque_abs_ema_slow_r,
        'torque_abs_ema_fast_sum': torque_abs_ema_fast_sum,
        'torque_abs_ema_slow_sum': torque_abs_ema_slow_sum,
        'torque_signed_ema_fast_sum': torque_signed_ema_fast_sum,
        'torque_signed_ema_slow_sum': torque_signed_ema_slow_sum,
        'assist_proxy_l': assist_proxy_l,
        'assist_proxy_r': assist_proxy_r,
        'assist_proxy_sum': assist_proxy_sum,
        'torque_stride_impulse_l': torque_stride_impulse_l,
        'torque_stride_impulse_r': torque_stride_impulse_r,
        'torque_stride_impulse_sum': torque_stride_impulse_sum,
        'torque_preview_l_100ms': torque_preview_l_100ms,
        'torque_preview_r_100ms': torque_preview_r_100ms,
        'torque_preview_sum_100ms': torque_preview_sum_100ms,
        'torque_preview_delta_l_100ms': torque_preview_delta_l_100ms,
        'torque_preview_delta_r_100ms': torque_preview_delta_r_100ms,
        'torque_preview_delta_sum_100ms': torque_preview_delta_sum_100ms,
        'assist_active_flag': assist_active_flag,
        'assist_high_flag': assist_high_flag,
        'assist_intensity_norm': assist_intensity_norm,
        'anchor_diff_imu_geom': anchor_diff_imu_geom,
        'anchor_diff_imu_pseudo': anchor_diff_imu_pseudo,
        'anchor_diff_geom_pseudo': anchor_diff_geom_pseudo,
        'anchor_confidence': anchor_confidence,
        'plateau_confidence': plateau_confidence,
        'anchor_consensus_speed': anchor_consensus_speed,
        'anchor_consensus_spread': anchor_consensus_spread,
        'highspeed_anchor_hint': highspeed_anchor_hint,
        'time_since_assist_on': time_since_assist_on,
        'time_since_assist_off': time_since_assist_off,
        'assist_duty_cycle_1s': assist_duty_cycle_1s,
        'assist_duty_cycle_3s': assist_duty_cycle_3s,
        'assist_duty_cycle_5s': assist_duty_cycle_5s,
        'positive_work_ema_fast': positive_work_ema_fast,
        'positive_work_ema_slow': positive_work_ema_slow,
        'work_norm_cadence': work_norm_cadence,
        'work_norm_excursion': work_norm_excursion,
        'work_norm_step_time': work_norm_step_time,
        'trunk_thigh_coherence': trunk_thigh_coherence,
        'trunk_hip_coherence': trunk_hip_coherence,
        'excursion_residual': excursion_residual,
    }


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


def _get_feature_group(gpath, v_name):
    """Classify a feature variable into its noise-group for per-group LPF/noise."""
    if gpath == 'sub_info':
        return 'subject_info'
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
    target_lpf_mode="zero_phase",
    include_levels=None, exclude_levels=None, include_trials=None,
    use_physical_velocity_model=False,
    use_gait_phase=False,
    input_lpf_cutoff=None, input_lpf_order=4,
    input_lpf_per_group=None,
    return_metadata=False,
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
    meta_list_all = []

    for lv in level_keys:
        if include_levels and lv not in include_levels:
            continue
        if exclude_levels and lv in exclude_levels:
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

            # Pre-compute leg_length_mean for _linvel/_arc features
            _trial_leg_length = None
            _needs_leg = any(
                v.endswith('_linvel') or v.endswith('_arc')
                for _, vs in input_vars for v in vs
            )
            if _needs_leg:
                _si = f[sub].get('sub_info') if sub in f else None
                _ah = 1.70
                if _si is not None:
                    if 'height' in _si:
                        _rh = float(_si['height'][()])
                        _ah = _rh / 1000.0 if _rh > 10.0 else _rh
                    elif 'height' in _si.attrs:
                        _rh = float(_si.attrs['height'])
                        _ah = _rh / 1000.0 if _rh > 10.0 else _rh
                _leg_l, _leg_r = None, None
                if _si is not None:
                    if 'left' in _si and 'leg length' in _si['left']:
                        _leg_l = float(_si['left']['leg length'][()]) / 1000.0
                    if 'right' in _si and 'leg length' in _si['right']:
                        _leg_r = float(_si['right']['leg length'][()]) / 1000.0
                if _leg_l is None:
                    _leg_l = max(0.53 * _ah, 0.5)
                if _leg_r is None:
                    _leg_r = max(0.53 * _ah, 0.5)
                _trial_leg_length = 0.5 * (_leg_l + _leg_r)

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
                    approx_height = 1.70
                    if si is not None:
                        if 'height' in si:
                            raw_h = float(si['height'][()])
                            approx_height = raw_h / 1000.0 if raw_h > 10.0 else raw_h
                        elif 'height' in si.attrs:
                            raw_h = float(si.attrs['height'])
                            approx_height = raw_h / 1000.0 if raw_h > 10.0 else raw_h
                    default_leg = max(0.53 * float(approx_height), 0.5)
                    si_cols = []
                    for v_name in vars:
                        default = {'height': 1.70, 'weight': 70.0, 'leg_length_l': default_leg, 'leg_length_r': default_leg, 'leg_length_mean': default_leg, 'leg_length_diff': 0.0}.get(v_name, 0.0)
                        val = default
                        if si is not None:
                            if v_name in si:
                                raw = si[v_name][()]
                                if isinstance(raw, (bytes, bytearray)):
                                    raw = raw.decode('utf-8')
                                val = float(raw)
                            elif v_name in si.attrs:
                                val = float(si.attrs[v_name])
                            elif v_name in {'leg_length_l', 'leg_length_r', 'leg_length_mean', 'leg_length_diff'}:
                                leg_l = None
                                leg_r = None
                                if 'left' in si and 'leg length' in si['left']:
                                    leg_l = float(si['left']['leg length'][()]) / 1000.0
                                if 'right' in si and 'leg length' in si['right']:
                                    leg_r = float(si['right']['leg length'][()]) / 1000.0
                                if leg_l is None:
                                    leg_l = default_leg
                                if leg_r is None:
                                    leg_r = default_leg
                                if v_name == 'leg_length_l':
                                    val = leg_l
                                elif v_name == 'leg_length_r':
                                    val = leg_r
                                elif v_name == 'leg_length_mean':
                                    val = 0.5 * (leg_l + leg_r)
                                elif v_name == 'leg_length_diff':
                                    val = leg_l - leg_r
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
                    a_belt_global = _causal_derivative(v_belt_flat, fs)

                for v_name in vars:
                    col_data = None
                    base_name = None
                    is_accel = False

                    # 1. Direct Load
                    if not gpath.startswith('derived') and v_name in grp and not (v_name.endswith('_dot') or v_name.endswith('_ddot')):
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
                            base_data = np.asarray(base_data, dtype=np.float32).flatten()
                            d1 = _causal_derivative(base_data, fs)
                            if is_accel:
                                target_data = _causal_derivative(d1, fs)
                            else:
                                target_data = d1

                            # Per-group LPF for derived features (replaces default 30Hz)
                            _derived_cutoff = 30.0
                            if input_lpf_per_group is not None:
                                _dgroup = 'accelerations' if is_accel else 'velocities'
                                if _dgroup in input_lpf_per_group:
                                    _derived_cutoff = input_lpf_per_group[_dgroup]
                            target_data = _causal_lpf_1d(target_data, _derived_cutoff, fs, order=4)
                            col_data = target_data

                            if use_physical_velocity_model and not is_accel and 'hip' in v_name and 'flexion' in v_name:
                                L = 0.9
                                if sub in f and "leg_length" in f[sub].attrs: L = f[sub].attrs["leg_length"]
                                elif sub in f and "sub_info" in f[sub] and "leg_length" in f[sub]["sub_info"].attrs: L = f[sub]["sub_info"].attrs["leg_length"]
                                col_data = L * target_data * np.cos(base_data)
                        else:
                            print(f"[WARN] Base data {base_name} not found for {v_name}")
                            valid_trial = False; break

                    # 2b. Leg-length normalized features: _linvel and _arc
                    if col_data is None and not gpath.startswith('derived') and v_name.endswith('_linvel'):
                        # hip_angle_linvel = hip_angle_dot × leg_length (m/s)
                        base_name = v_name[:-7]  # strip '_linvel'
                        if base_name in grp and _trial_leg_length is not None:
                            base_data = np.asarray(grp[base_name][:], dtype=np.float32).flatten()
                            d1 = _causal_derivative(base_data, fs)
                            _derived_cutoff = 30.0
                            if input_lpf_per_group is not None and 'velocities' in input_lpf_per_group:
                                _derived_cutoff = input_lpf_per_group['velocities']
                            d1 = _causal_lpf_1d(d1, _derived_cutoff, fs, order=4)
                            col_data = (np.deg2rad(d1) * _trial_leg_length).astype(np.float32)
                        elif base_name not in grp:
                            print(f"[WARN] Base data {base_name} not found for {v_name}")
                            valid_trial = False; break

                    if col_data is None and not gpath.startswith('derived') and v_name.endswith('_arc'):
                        # hip_angle_arc = hip_angle × leg_length (m)
                        base_name = v_name[:-4]  # strip '_arc'
                        if base_name in grp and _trial_leg_length is not None:
                            base_data = np.asarray(grp[base_name][:], dtype=np.float32).flatten()
                            col_data = (np.deg2rad(base_data) * _trial_leg_length).astype(np.float32)
                        elif base_name not in grp:
                            print(f"[WARN] Base data {base_name} not found for {v_name}")
                            valid_trial = False; break

                    # [NEW] Fallback for Milestone 1: hip_angle -> motor_angle
                    if col_data is None and not gpath.startswith('derived') and 'hip_angle' in v_name:
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
                                        a_b = _causal_derivative(v_belt.flatten(), fs)
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

                    # 6.5. Robot-only realtime biomechanical features
                    if col_data is None and gpath == 'derived/robot_biomech_rt' and v_name in _ROBOT_BIOMECH_RT_VARS:
                        try:
                            if 'robot_biomech_rt_cache' not in trial_cache:
                                left_grp = trial_group['robot']['left']
                                right_grp = trial_group['robot']['right']
                                back_imu = trial_group['robot']['back_imu']

                                hip_l = left_grp['hip_angle'][:].flatten()
                                hip_r = right_grp['hip_angle'][:].flatten()
                                thigh_l = left_grp['thigh_angle'][:].flatten()
                                thigh_r = right_grp['thigh_angle'][:].flatten()
                                torque_l = left_grp['torque'][:].flatten()
                                torque_r = right_grp['torque'][:].flatten()
                                motor_l = left_grp['motor_angle'][:].flatten()
                                motor_r = right_grp['motor_angle'][:].flatten()

                                ax = back_imu['accel_x'][:]
                                ay = back_imu['accel_y'][:]
                                az = back_imu['accel_z'][:]
                                gx = back_imu['gyro_x'][:]
                                gy = back_imu['gyro_y'][:]
                                gz = back_imu['gyro_z'][:]
                                qw = back_imu['quat_w'][:]
                                qx = back_imu['quat_x'][:]
                                qy = back_imu['quat_y'][:]
                                qz = back_imu['quat_z'][:]

                                if sub.startswith('m2_'):
                                    az = -az
                                elif sub.startswith('m1_'):
                                    ax = -ax

                                height_m, weight_kg = _estimate_subject_height_weight(f, sub)
                                treadmill_acc = _compute_treadmill_acc_mean(trial_group, fs)
                                robot_biomech = compute_robot_biomech_rt_features(
                                    hip_l, hip_r, thigh_l, thigh_r, torque_l, torque_r,
                                    motor_l, motor_r,
                                    np.stack([ax, ay, az], axis=1),
                                    np.stack([gx, gy, gz], axis=1),
                                    np.stack([qw, qx, qy, qz], axis=1),
                                    height_m, weight_kg, treadmill_acc=treadmill_acc, fs_hz=fs,
                                )
                                trial_cache['robot_biomech_rt_cache'] = robot_biomech
                            col_data = trial_cache['robot_biomech_rt_cache'][v_name].reshape(-1, 1)
                        except Exception as e:
                            print(f"[ERROR] Robot Biomech RT Error ({v_name}): {e}")
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
                                pitch_dot = _causal_derivative(pitch, fs)
                                pitch_dot = _causal_lpf_1d(pitch_dot, 10.0, fs, order=2)
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

                    # 10. Body Frame IMU variants
                    if col_data is None and gpath in {
                        'derived/body_frame_imu',
                        'derived/body_frame_imu_fixedheading',
                        'derived/body_frame_imu_stationaryheading',
                        'derived/body_frame_imu_stationaryheading_nobelt',
                        'derived/body_frame_imu_zonlypca',
                        'derived/body_frame_imu_zonlypcagated',
                    }:
                        try:
                            cache_key = {
                                'derived/body_frame_imu': 'body_frame_cache',
                                'derived/body_frame_imu_fixedheading': 'body_frame_fixed_cache',
                                'derived/body_frame_imu_stationaryheading': 'body_frame_stationary_cache',
                                'derived/body_frame_imu_stationaryheading_nobelt': 'body_frame_stationary_nobelt_cache',
                                'derived/body_frame_imu_zonlypca': 'body_frame_zonly_pca_cache',
                                'derived/body_frame_imu_zonlypcagated': 'body_frame_zonly_pcagated_cache',
                            }[gpath]
                            valid_vars = {
                                'derived/body_frame_imu': _BODY_FRAME_IMU_VARS,
                                'derived/body_frame_imu_fixedheading': _BODY_FRAME_IMU_FIXED_VARS,
                                'derived/body_frame_imu_stationaryheading': _BODY_FRAME_IMU_STATIONARY_VARS,
                                'derived/body_frame_imu_stationaryheading_nobelt': _BODY_FRAME_IMU_STATIONARY_NOBELT_VARS,
                                'derived/body_frame_imu_zonlypca': _BODY_FRAME_IMU_ZONLY_PCA_VARS,
                                'derived/body_frame_imu_zonlypcagated': _BODY_FRAME_IMU_ZONLY_PCAGATED_VARS,
                            }[gpath]
                            heading_mode = {
                                'derived/body_frame_imu': 'pca',
                                'derived/body_frame_imu_fixedheading': 'fixed',
                                'derived/body_frame_imu_stationaryheading': 'stationary_gated',
                                'derived/body_frame_imu_stationaryheading_nobelt': 'stationary_gated',
                                'derived/body_frame_imu_zonlypca': 'pca_zonly',
                                'derived/body_frame_imu_zonlypcagated': 'pca_gated_zonly',
                            }[gpath]
                            use_treadmill_acc = gpath != 'derived/body_frame_imu_stationaryheading_nobelt'
                            if v_name not in valid_vars:
                                continue
                            if cache_key not in derived_cache:
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

                                _a_tread = _compute_treadmill_acc_mean(trial_group, fs) if use_treadmill_acc else np.zeros(len(ax), dtype=np.float32)

                                derived_cache[cache_key] = estimate_body_frame_features(
                                    _acc_local, _gyro_local, _quat_wxyz,
                                    _a_tread.astype(np.float32), fs,
                                    heading_mode=heading_mode,
                                )

                            _bf = derived_cache[cache_key]
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

                    # 10b. Gravity-aligned IMU without forward-axis calibration
                    if col_data is None and gpath == 'derived/gravity_aligned_imu_zonly':
                        try:
                            cache_key = 'gravity_aligned_imu_zonly_cache'
                            if v_name not in _GRAVITY_ALIGNED_IMU_ZONLY_VARS:
                                continue
                            if cache_key not in derived_cache:
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
                                derived_cache[cache_key] = estimate_gravity_aligned_imu_features(
                                    _acc_local, _gyro_local, _quat_wxyz, fs
                                )
                            col_data = derived_cache[cache_key][v_name]
                        except Exception as e:
                            print(f"[ERROR] Gravity-aligned IMU Error ({v_name}): {e}")
                            valid_trial = False; break

                    # 10c. Tilt-only local IMU (yaw preserved, roll/pitch corrected)
                    if col_data is None and gpath == 'derived/tilt_local_imu_2d':
                        try:
                            cache_key = 'tilt_local_imu_2d_cache'
                            if v_name not in _TILT_LOCAL_IMU_2D_VARS:
                                continue
                            if cache_key not in derived_cache:
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
                                derived_cache[cache_key] = estimate_tilt_local_imu_features(
                                    _acc_local, _gyro_local, _quat_wxyz, fs
                                )
                            col_data = derived_cache[cache_key][v_name]
                        except Exception as e:
                            print(f"[ERROR] Tilt-local IMU Error ({v_name}): {e}")
                            valid_trial = False; break

                    # 10d. Tilt-only local IMU using online complementary-filter roll/pitch
                    if col_data is None and gpath in {
                        'derived/tilt_cf_imu_2d',
                        'derived/tilt_cf_imu_2d_slow',
                        'derived/tilt_cf_imu_2d_adapt',
                        'derived/tilt_tvcf_imu_2d',
                        'derived/tilt_tvcf_imu_2d_og',
                        'derived/tilt_tvcf_imu_2d_wide',
                    }:
                        try:
                            cf_specs = {
                                'derived/tilt_cf_imu_2d': dict(cache_key='tilt_cf_imu_2d_cache', alpha=0.985, mode='fixed'),
                                'derived/tilt_cf_imu_2d_slow': dict(cache_key='tilt_cf_imu_2d_slow_cache', alpha=0.995, mode='fixed'),
                                'derived/tilt_cf_imu_2d_adapt': dict(
                                    cache_key='tilt_cf_imu_2d_adapt_cache',
                                    alpha=0.995,
                                    alpha_min=0.975,
                                    mode='adaptive_accel',
                                    acc_trust_width=0.10,
                                    trust_ema=0.92,
                                ),
                                'derived/tilt_tvcf_imu_2d': dict(
                                    cache_key='tilt_tvcf_imu_2d_cache',
                                    mode='tvcf',
                                    trust_ema=0.92,
                                    tvcf_omega_low=0.5,
                                    tvcf_omega_high=2.5,
                                ),
                                'derived/tilt_tvcf_imu_2d_og': dict(
                                    cache_key='tilt_tvcf_imu_2d_og_cache',
                                    mode='tvcf',
                                    trust_ema=0.92,
                                    tvcf_omega_low=0.5,
                                    tvcf_omega_high=2.5,
                                    overground_compensate=True,
                                ),
                                'derived/tilt_tvcf_imu_2d_wide': dict(
                                    cache_key='tilt_tvcf_imu_2d_wide_cache',
                                    mode='tvcf',
                                    trust_ema=0.90,
                                    tvcf_omega_low=0.35,
                                    tvcf_omega_high=4.0,
                                ),
                            }
                            spec = cf_specs[gpath]
                            cache_key = spec['cache_key']
                            valid_vars = _TILT_TVCF_IMU_2D_OG_VARS if gpath == 'derived/tilt_tvcf_imu_2d_og' else _TILT_CF_IMU_2D_VARS
                            if v_name not in valid_vars:
                                continue
                            if cache_key not in derived_cache:
                                imu_grp = trial_group['robot']['back_imu']
                                _acc_local = np.stack([
                                    imu_grp['accel_x'][:], imu_grp['accel_y'][:], imu_grp['accel_z'][:]
                                ], axis=1).astype(np.float32)
                                _gyro_local = np.stack([
                                    imu_grp['gyro_x'][:], imu_grp['gyro_y'][:], imu_grp['gyro_z'][:]
                                ], axis=1).astype(np.float32)
                                derived_cache[cache_key] = estimate_tilt_cf_imu_features(
                                    _acc_local, _gyro_local, fs,
                                    **{k: v for k, v in spec.items() if k not in {'cache_key', 'overground_compensate'}}
                                )
                                if spec.get('overground_compensate'):
                                    _a_tread = derived_cache.get('treadmill_acc_mean_cache')
                                    if _a_tread is None:
                                        _a_tread = _compute_treadmill_acc_mean(trial_group, fs)
                                        derived_cache['treadmill_acc_mean_cache'] = _a_tread.astype(np.float32)
                                    derived_cache[cache_key]['acc_hy'] = (
                                        derived_cache[cache_key]['acc_hy'][:, 0] + _a_tread
                                    ).astype(np.float32)[:, None]
                            col_data = derived_cache[cache_key][v_name]
                        except Exception as e:
                            print(f"[ERROR] Tilt-CF IMU Error ({v_name}): {e}")
                            valid_trial = False; break

                    # 11. IMU Integrated Speed (derived/imu_integrated_speed)
                    if col_data is None and gpath in {
                        'derived/imu_integrated_speed',
                        'derived/imu_integrated_speed_fixedheading',
                        'derived/imu_integrated_speed_stationaryheading',
                        'derived/imu_integrated_speed_stationaryheading_nobelt',
                        'derived/imu_integrated_speed_zonlypca',
                        'derived/imu_integrated_speed_zonlypcagated',
                    }:
                        try:
                            cache_key = {
                                'derived/imu_integrated_speed': 'imu_integrated_speed',
                                'derived/imu_integrated_speed_fixedheading': 'imu_integrated_speed_fixed',
                                'derived/imu_integrated_speed_stationaryheading': 'imu_integrated_speed_stationary',
                                'derived/imu_integrated_speed_stationaryheading_nobelt': 'imu_integrated_speed_stationary_nobelt',
                                'derived/imu_integrated_speed_zonlypca': 'imu_integrated_speed_zonly_pca',
                                'derived/imu_integrated_speed_zonlypcagated': 'imu_integrated_speed_zonly_pcagated',
                            }[gpath]
                            body_cache_key = {
                                'derived/imu_integrated_speed': 'body_frame_cache',
                                'derived/imu_integrated_speed_fixedheading': 'body_frame_fixed_cache',
                                'derived/imu_integrated_speed_stationaryheading': 'body_frame_stationary_cache',
                                'derived/imu_integrated_speed_stationaryheading_nobelt': 'body_frame_stationary_nobelt_cache',
                                'derived/imu_integrated_speed_zonlypca': 'body_frame_zonly_pca_cache',
                                'derived/imu_integrated_speed_zonlypcagated': 'body_frame_zonly_pcagated_cache',
                            }[gpath]
                            heading_mode = {
                                'derived/imu_integrated_speed': 'pca',
                                'derived/imu_integrated_speed_fixedheading': 'fixed',
                                'derived/imu_integrated_speed_stationaryheading': 'stationary_gated',
                                'derived/imu_integrated_speed_stationaryheading_nobelt': 'stationary_gated',
                                'derived/imu_integrated_speed_zonlypca': 'pca_zonly',
                                'derived/imu_integrated_speed_zonlypcagated': 'pca_gated_zonly',
                            }[gpath]
                            valid_vars = {
                                'derived/imu_integrated_speed': _IMU_INTEGRATED_SPEED_VARS,
                                'derived/imu_integrated_speed_fixedheading': _IMU_INTEGRATED_SPEED_FIXED_VARS,
                                'derived/imu_integrated_speed_stationaryheading': _IMU_INTEGRATED_SPEED_STATIONARY_VARS,
                                'derived/imu_integrated_speed_stationaryheading_nobelt': _IMU_INTEGRATED_SPEED_STATIONARY_NOBELT_VARS,
                                'derived/imu_integrated_speed_zonlypca': _IMU_INTEGRATED_SPEED_ZONLY_PCA_VARS,
                                'derived/imu_integrated_speed_zonlypcagated': _IMU_INTEGRATED_SPEED_ZONLY_PCAGATED_VARS,
                            }[gpath]
                            use_treadmill_acc = gpath != 'derived/imu_integrated_speed_stationaryheading_nobelt'
                            if v_name not in valid_vars:
                                continue
                            if cache_key not in derived_cache:
                                if body_cache_key not in derived_cache:
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

                                    _a_tread = _compute_treadmill_acc_mean(trial_group, fs) if use_treadmill_acc else np.zeros(len(_acc_local), dtype=np.float32)

                                    derived_cache[body_cache_key] = estimate_body_frame_features(
                                        _acc_local, _gyro_local, _quat_wxyz,
                                        _a_tread.astype(np.float32), fs,
                                        heading_mode=heading_mode,
                                    )

                                # forward_accel_overground = acc_body_Y + a_tread (overground equivalent)
                                acc_og = derived_cache[body_cache_key]['forward_accel_overground'].flatten()
                                derived_cache[cache_key] = compute_imu_integrated_speed(acc_og, fs)

                            col_data = derived_cache[cache_key].reshape(-1, 1)
                        except Exception as e:
                            print(f"[ERROR] IMU Integrated Speed Error ({v_name}): {e}")
                            valid_trial = False; break

                    # 11b. Horizontal integrated velocity in a gravity-aligned frame
                    if col_data is None and gpath == 'derived/imu_horizontal_velocity_zonly':
                        try:
                            cache_key = 'imu_horizontal_velocity_zonly_cache'
                            if v_name not in _IMU_HORIZONTAL_VELOCITY_ZONLY_VARS:
                                continue
                            if cache_key not in derived_cache:
                                if 'gravity_aligned_imu_zonly_cache' not in derived_cache:
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
                                    derived_cache['gravity_aligned_imu_zonly_cache'] = estimate_gravity_aligned_imu_features(
                                        _acc_local, _gyro_local, _quat_wxyz, fs
                                    )
                                _ga = derived_cache['gravity_aligned_imu_zonly_cache']
                                derived_cache[cache_key] = {
                                    'vel_hx': compute_imu_integrated_speed(_ga['acc_hx'][:, 0], fs),
                                    'vel_hy': compute_imu_integrated_speed(_ga['acc_hy'][:, 0], fs),
                                }
                            col_data = derived_cache[cache_key][v_name].reshape(-1, 1)
                        except Exception as e:
                            print(f"[ERROR] IMU Horizontal Velocity Error ({v_name}): {e}")
                            valid_trial = False; break

                    # 11c. Horizontal integrated velocity in a tilt-only local frame
                    if col_data is None and gpath == 'derived/tilt_local_velocity_2d':
                        try:
                            cache_key = 'tilt_local_velocity_2d_cache'
                            if v_name not in _IMU_TILT_LOCAL_VELOCITY_2D_VARS:
                                continue
                            if cache_key not in derived_cache:
                                if 'tilt_local_imu_2d_cache' not in derived_cache:
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
                                    derived_cache['tilt_local_imu_2d_cache'] = estimate_tilt_local_imu_features(
                                        _acc_local, _gyro_local, _quat_wxyz, fs
                                    )
                                _tilt = derived_cache['tilt_local_imu_2d_cache']
                                derived_cache[cache_key] = {
                                    'vel_hx': compute_imu_integrated_speed(_tilt['acc_hx'][:, 0], fs),
                                    'vel_hy': compute_imu_integrated_speed(_tilt['acc_hy'][:, 0], fs),
                                }
                            col_data = derived_cache[cache_key][v_name].reshape(-1, 1)
                        except Exception as e:
                            print(f"[ERROR] Tilt-local Velocity Error ({v_name}): {e}")
                            valid_trial = False; break

                    # 11c-2. Horizontal integrated velocity in an online complementary-filter tilt-local frame
                    if col_data is None and gpath in {
                        'derived/tilt_cf_velocity_2d',
                        'derived/tilt_cf_velocity_2d_slow',
                        'derived/tilt_cf_velocity_2d_adapt',
                        'derived/tilt_tvcf_velocity_2d',
                        'derived/tilt_tvcf_velocity_2d_og',
                        'derived/tilt_tvcf_velocity_2d_wide',
                    }:
                        try:
                            cf_specs = {
                                'derived/tilt_cf_velocity_2d': dict(
                                    cache_key='tilt_cf_velocity_2d_cache',
                                    imu_cache_key='tilt_cf_imu_2d_cache',
                                    alpha=0.985,
                                    mode='fixed',
                                ),
                                'derived/tilt_cf_velocity_2d_slow': dict(
                                    cache_key='tilt_cf_velocity_2d_slow_cache',
                                    imu_cache_key='tilt_cf_imu_2d_slow_cache',
                                    alpha=0.995,
                                    mode='fixed',
                                ),
                                'derived/tilt_cf_velocity_2d_adapt': dict(
                                    cache_key='tilt_cf_velocity_2d_adapt_cache',
                                    imu_cache_key='tilt_cf_imu_2d_adapt_cache',
                                    alpha=0.995,
                                    alpha_min=0.975,
                                    mode='adaptive_accel',
                                    acc_trust_width=0.10,
                                    trust_ema=0.92,
                                ),
                                'derived/tilt_tvcf_velocity_2d': dict(
                                    cache_key='tilt_tvcf_velocity_2d_cache',
                                    imu_cache_key='tilt_tvcf_imu_2d_cache',
                                    mode='tvcf',
                                    trust_ema=0.92,
                                    tvcf_omega_low=0.5,
                                    tvcf_omega_high=2.5,
                                ),
                                'derived/tilt_tvcf_velocity_2d_og': dict(
                                    cache_key='tilt_tvcf_velocity_2d_og_cache',
                                    imu_cache_key='tilt_tvcf_imu_2d_og_cache',
                                    mode='tvcf',
                                    trust_ema=0.92,
                                    tvcf_omega_low=0.5,
                                    tvcf_omega_high=2.5,
                                    overground_compensate=True,
                                ),
                                'derived/tilt_tvcf_velocity_2d_wide': dict(
                                    cache_key='tilt_tvcf_velocity_2d_wide_cache',
                                    imu_cache_key='tilt_tvcf_imu_2d_wide_cache',
                                    mode='tvcf',
                                    trust_ema=0.90,
                                    tvcf_omega_low=0.35,
                                    tvcf_omega_high=4.0,
                                ),
                            }
                            spec = cf_specs[gpath]
                            cache_key = spec['cache_key']
                            imu_cache_key = spec['imu_cache_key']
                            valid_vars = _IMU_TILT_TVCF_VELOCITY_2D_OG_VARS if gpath == 'derived/tilt_tvcf_velocity_2d_og' else _IMU_TILT_CF_VELOCITY_2D_VARS
                            if v_name not in valid_vars:
                                continue
                            if cache_key not in derived_cache:
                                if imu_cache_key not in derived_cache:
                                    imu_grp = trial_group['robot']['back_imu']
                                    _acc_local = np.stack([
                                        imu_grp['accel_x'][:], imu_grp['accel_y'][:], imu_grp['accel_z'][:]
                                    ], axis=1).astype(np.float32)
                                    _gyro_local = np.stack([
                                        imu_grp['gyro_x'][:], imu_grp['gyro_y'][:], imu_grp['gyro_z'][:]
                                    ], axis=1).astype(np.float32)
                                    derived_cache[imu_cache_key] = estimate_tilt_cf_imu_features(
                                        _acc_local, _gyro_local, fs,
                                        **{k: v for k, v in spec.items() if k not in {'cache_key', 'imu_cache_key', 'overground_compensate'}}
                                    )
                                    if spec.get('overground_compensate'):
                                        _a_tread = derived_cache.get('treadmill_acc_mean_cache')
                                        if _a_tread is None:
                                            _a_tread = _compute_treadmill_acc_mean(trial_group, fs)
                                            derived_cache['treadmill_acc_mean_cache'] = _a_tread.astype(np.float32)
                                        derived_cache[imu_cache_key]['acc_hy'] = (
                                            derived_cache[imu_cache_key]['acc_hy'][:, 0] + _a_tread
                                        ).astype(np.float32)[:, None]
                                _tilt = derived_cache[imu_cache_key]
                                derived_cache[cache_key] = {
                                    'vel_hx': compute_imu_integrated_speed(_tilt['acc_hx'][:, 0], fs),
                                    'vel_hy': compute_imu_integrated_speed(_tilt['acc_hy'][:, 0], fs),
                                }
                            col_data = derived_cache[cache_key][v_name].reshape(-1, 1)
                        except Exception as e:
                            print(f"[ERROR] Tilt-CF Velocity Error ({v_name}): {e}")
                            valid_trial = False; break

                    # 11d. Compact summary features from tilt-local 2D IMU
                    if col_data is None and gpath == 'derived/tilt_local_summary_2d':
                        try:
                            cache_key = 'tilt_local_summary_2d_cache'
                            if v_name not in _TILT_LOCAL_SUMMARY_2D_VARS:
                                continue
                            if cache_key not in derived_cache:
                                if 'tilt_local_imu_2d_cache' not in derived_cache:
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
                                    derived_cache['tilt_local_imu_2d_cache'] = estimate_tilt_local_imu_features(
                                        _acc_local, _gyro_local, _quat_wxyz, fs
                                    )
                                if 'tilt_local_velocity_2d_cache' not in derived_cache:
                                    _tilt = derived_cache['tilt_local_imu_2d_cache']
                                    derived_cache['tilt_local_velocity_2d_cache'] = {
                                        'vel_hx': compute_imu_integrated_speed(_tilt['acc_hx'][:, 0], fs),
                                        'vel_hy': compute_imu_integrated_speed(_tilt['acc_hy'][:, 0], fs),
                                    }
                                _tilt = derived_cache['tilt_local_imu_2d_cache']
                                _vel = derived_cache['tilt_local_velocity_2d_cache']
                                derived_cache[cache_key] = {
                                    'acc_h_mag': np.sqrt(_tilt['acc_hx'][:, 0] ** 2 + _tilt['acc_hy'][:, 0] ** 2).astype(np.float32),
                                    'gyro_h_mag': np.sqrt(_tilt['gyro_hx'][:, 0] ** 2 + _tilt['gyro_hy'][:, 0] ** 2).astype(np.float32),
                                    'vel_h_mag': np.sqrt(_vel['vel_hx'] ** 2 + _vel['vel_hy'] ** 2).astype(np.float32),
                                }
                            col_data = derived_cache[cache_key][v_name].reshape(-1, 1)
                        except Exception as e:
                            print(f"[ERROR] Tilt-local Summary Error ({v_name}): {e}")
                            valid_trial = False; break

                    # 11d-2. Compact summary features from online complementary-filter tilt-local 2D IMU
                    if col_data is None and gpath in {
                        'derived/tilt_cf_summary_2d',
                        'derived/tilt_cf_summary_2d_slow',
                        'derived/tilt_cf_summary_2d_adapt',
                        'derived/tilt_tvcf_summary_2d',
                        'derived/tilt_tvcf_summary_2d_og',
                        'derived/tilt_tvcf_summary_2d_wide',
                    }:
                        try:
                            cf_specs = {
                                'derived/tilt_cf_summary_2d': dict(
                                    cache_key='tilt_cf_summary_2d_cache',
                                    imu_cache_key='tilt_cf_imu_2d_cache',
                                    vel_cache_key='tilt_cf_velocity_2d_cache',
                                    alpha=0.985,
                                    mode='fixed',
                                ),
                                'derived/tilt_cf_summary_2d_slow': dict(
                                    cache_key='tilt_cf_summary_2d_slow_cache',
                                    imu_cache_key='tilt_cf_imu_2d_slow_cache',
                                    vel_cache_key='tilt_cf_velocity_2d_slow_cache',
                                    alpha=0.995,
                                    mode='fixed',
                                ),
                                'derived/tilt_cf_summary_2d_adapt': dict(
                                    cache_key='tilt_cf_summary_2d_adapt_cache',
                                    imu_cache_key='tilt_cf_imu_2d_adapt_cache',
                                    vel_cache_key='tilt_cf_velocity_2d_adapt_cache',
                                    alpha=0.995,
                                    alpha_min=0.975,
                                    mode='adaptive_accel',
                                    acc_trust_width=0.10,
                                    trust_ema=0.92,
                                ),
                                'derived/tilt_tvcf_summary_2d': dict(
                                    cache_key='tilt_tvcf_summary_2d_cache',
                                    imu_cache_key='tilt_tvcf_imu_2d_cache',
                                    vel_cache_key='tilt_tvcf_velocity_2d_cache',
                                    mode='tvcf',
                                    trust_ema=0.92,
                                    tvcf_omega_low=0.5,
                                    tvcf_omega_high=2.5,
                                ),
                                'derived/tilt_tvcf_summary_2d_og': dict(
                                    cache_key='tilt_tvcf_summary_2d_og_cache',
                                    imu_cache_key='tilt_tvcf_imu_2d_og_cache',
                                    vel_cache_key='tilt_tvcf_velocity_2d_og_cache',
                                    mode='tvcf',
                                    trust_ema=0.92,
                                    tvcf_omega_low=0.5,
                                    tvcf_omega_high=2.5,
                                    overground_compensate=True,
                                ),
                                'derived/tilt_tvcf_summary_2d_wide': dict(
                                    cache_key='tilt_tvcf_summary_2d_wide_cache',
                                    imu_cache_key='tilt_tvcf_imu_2d_wide_cache',
                                    vel_cache_key='tilt_tvcf_velocity_2d_wide_cache',
                                    mode='tvcf',
                                    trust_ema=0.90,
                                    tvcf_omega_low=0.35,
                                    tvcf_omega_high=4.0,
                                ),
                            }
                            spec = cf_specs[gpath]
                            cache_key = spec['cache_key']
                            imu_cache_key = spec['imu_cache_key']
                            vel_cache_key = spec['vel_cache_key']
                            valid_vars = _TILT_TVCF_SUMMARY_2D_OG_VARS if gpath == 'derived/tilt_tvcf_summary_2d_og' else _TILT_CF_SUMMARY_2D_VARS
                            if v_name not in valid_vars:
                                continue
                            if cache_key not in derived_cache:
                                if imu_cache_key not in derived_cache:
                                    imu_grp = trial_group['robot']['back_imu']
                                    _acc_local = np.stack([
                                        imu_grp['accel_x'][:], imu_grp['accel_y'][:], imu_grp['accel_z'][:]
                                    ], axis=1).astype(np.float32)
                                    _gyro_local = np.stack([
                                        imu_grp['gyro_x'][:], imu_grp['gyro_y'][:], imu_grp['gyro_z'][:]
                                    ], axis=1).astype(np.float32)
                                    derived_cache[imu_cache_key] = estimate_tilt_cf_imu_features(
                                        _acc_local, _gyro_local, fs,
                                        **{k: v for k, v in spec.items() if k not in {'cache_key', 'imu_cache_key', 'vel_cache_key', 'overground_compensate'}}
                                    )
                                    if spec.get('overground_compensate'):
                                        _a_tread = derived_cache.get('treadmill_acc_mean_cache')
                                        if _a_tread is None:
                                            _a_tread = _compute_treadmill_acc_mean(trial_group, fs)
                                            derived_cache['treadmill_acc_mean_cache'] = _a_tread.astype(np.float32)
                                        derived_cache[imu_cache_key]['acc_hy'] = (
                                            derived_cache[imu_cache_key]['acc_hy'][:, 0] + _a_tread
                                        ).astype(np.float32)[:, None]
                                if vel_cache_key not in derived_cache:
                                    _tilt = derived_cache[imu_cache_key]
                                    derived_cache[vel_cache_key] = {
                                        'vel_hx': compute_imu_integrated_speed(_tilt['acc_hx'][:, 0], fs),
                                        'vel_hy': compute_imu_integrated_speed(_tilt['acc_hy'][:, 0], fs),
                                    }
                                _tilt = derived_cache[imu_cache_key]
                                _vel = derived_cache[vel_cache_key]
                                derived_cache[cache_key] = {
                                    'acc_h_mag': np.sqrt(_tilt['acc_hx'][:, 0] ** 2 + _tilt['acc_hy'][:, 0] ** 2).astype(np.float32),
                                    'gyro_h_mag': np.sqrt(_tilt['gyro_hx'][:, 0] ** 2 + _tilt['gyro_hy'][:, 0] ** 2).astype(np.float32),
                                    'vel_h_mag': np.sqrt(_vel['vel_hx'] ** 2 + _vel['vel_hy'] ** 2).astype(np.float32),
                                }
                            col_data = derived_cache[cache_key][v_name].reshape(-1, 1)
                        except Exception as e:
                            print(f"[ERROR] Tilt-CF Summary Error ({v_name}): {e}")
                            valid_trial = False; break

                    # 11d-3. Scalar treadmill acceleration input
                    if col_data is None and gpath == 'derived/treadmill_acc_scalar':
                        try:
                            if v_name not in _TREADMILL_ACC_SCALAR_VARS:
                                continue
                            cache_key = 'treadmill_acc_mean_cache'
                            if cache_key not in derived_cache:
                                derived_cache[cache_key] = _compute_treadmill_acc_mean(trial_group, fs).astype(np.float32)
                            col_data = derived_cache[cache_key].reshape(-1, 1)
                        except Exception as e:
                            print(f"[ERROR] Treadmill Acc Scalar Error ({v_name}): {e}")
                            valid_trial = False; break

                    # 12. wo_grf-inspired robot-only realtime priors (derived/wo_priors_rt)
                    if col_data is None and gpath == 'derived/wo_priors_rt' and v_name in _WO_PRIORS_RT_VARS:
                        try:
                            if 'wo_priors_rt_cache' not in trial_cache:
                                robot = trial_group['robot']
                                left = robot['left']
                                right = robot['right']
                                back_imu = robot['back_imu']
                                height_m, _ = _estimate_subject_height_weight(f, sub)
                                imu_acc_local = np.stack([
                                    back_imu['accel_x'][:],
                                    back_imu['accel_y'][:],
                                    back_imu['accel_z'][:],
                                ], axis=1).astype(np.float32)
                                imu_gyro_local = np.stack([
                                    back_imu['gyro_x'][:],
                                    back_imu['gyro_y'][:],
                                    back_imu['gyro_z'][:],
                                ], axis=1).astype(np.float32)
                                treadmill_acc = _compute_treadmill_acc_mean(trial_group, fs)
                                trial_cache['wo_priors_rt_cache'] = compute_wo_priors_rt_features(
                                    left['hip_angle'][:].flatten(),
                                    right['hip_angle'][:].flatten(),
                                    left['thigh_angle'][:].flatten(),
                                    right['thigh_angle'][:].flatten(),
                                    left['torque'][:].flatten(),
                                    right['torque'][:].flatten(),
                                    imu_acc_local,
                                    imu_gyro_local,
                                    height_m=height_m,
                                    fs_hz=fs,
                                    treadmill_acc=treadmill_acc if treadmill_acc.size > 0 else None,
                                )
                            col_data = trial_cache['wo_priors_rt_cache'][v_name].reshape(-1, 1)
                        except Exception as e:
                            print(f"[ERROR] wo_priors_rt Error ({v_name}): {e}")
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
                if not found_grp: valid_out = False; break

                for v in vars:
                    resolved_v = _resolve_output_var_name(grp, v)
                    if resolved_v is None:
                        valid_out = False
                        break
                    d = grp[resolved_v][:]
                    d = np.nan_to_num(d).reshape(-1, 1)
                    if _should_filter_output(gpath, resolved_v):
                        d = _apply_output_filter(d, lpf_cutoff, fs, lpf_order, target_lpf_mode)
                    extracted_outs.append(d)

            if not valid_out or not extracted_outs:
                continue

            all_objs.extend(extracted_outs)
            min_len_trial = min([a.shape[0] for a in all_objs])

            in_arr_trial = np.hstack([a[:min_len_trial] for a in curr_trial_in])
            out_arr_trial = np.hstack([a[:min_len_trial] for a in extracted_outs])

            in_list_all.append(in_arr_trial)
            out_list_all.append(out_arr_trial)
            if return_metadata:
                meta_list_all.append({
                    "subject": sub,
                    "condition": cond,
                    "level": lv,
                    "trial": trial_name,
                })

    if not in_list_all:
        if return_metadata:
            return [], [], []
        return [], []

    if return_metadata:
        return in_list_all, out_list_all, meta_list_all
    return in_list_all, out_list_all


def build_nn_dataset(
    data_path,
    sub_names, cond_names,
    input_vars, output_vars,
    time_window_input, time_window_output, stride,
    subject_selection=None,
    condition_selection=None,
    level_selection=None,
    debug_plot=False,
    lpf_cutoff=None,
    lpf_order=4,
    target_lpf_mode="zero_phase",
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
        include_levels = None
        exclude_levels = None
        if level_selection:
            if isinstance(level_selection, dict):
                include_levels = level_selection.get('include')
                exclude_levels = level_selection.get('exclude')
            else:
                include_levels = level_selection
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

                extracted = extract_condition_data_v2(
                    f, sub, cond, input_vars, output_vars,
                    lpf_cutoff=lpf_cutoff, lpf_order=lpf_order,
                    target_lpf_mode=target_lpf_mode,
                    include_levels=include_levels,
                    exclude_levels=exclude_levels,
                    use_physical_velocity_model=use_physical_velocity_model,
                    use_gait_phase=use_gait_phase,
                    input_lpf_cutoff=input_lpf_cutoff,
                    input_lpf_order=input_lpf_order,
                    input_lpf_per_group=input_lpf_per_group,
                    return_metadata=return_metadata,
                )
                if return_metadata:
                    X_trials, Y_trials, trial_meta = extracted
                else:
                    X_trials, Y_trials = extracted

                if not X_trials:
                    continue

                for trial_idx, (X_arr, Y_arr) in enumerate(zip(X_trials, Y_trials)):
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
                        meta = dict(trial_meta[trial_idx])
                        meta_list.append(meta)

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
    level_selection=None,
    lpf_cutoff=None,
    lpf_order=None,
    target_lpf_mode="zero_phase",
    est_tick_ranges=None,
    use_physical_velocity_model=False,
    use_gait_phase=False,
    input_lpf_cutoff=None,
    input_lpf_order=4,
    input_lpf_per_group=None,
    return_metadata=False,
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
            freeze_nested(level_selection),
            lpf_cutoff, lpf_order,
            target_lpf_mode,
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
                level_selection=level_selection,
                lpf_cutoff=lpf_cutoff,
                lpf_order=lpf_order,
                target_lpf_mode=target_lpf_mode,
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
            if return_metadata:
                meta_all.append({
                    "subject": meta["subject"] if not prefix else f"{prefix}{meta['subject']}",
                    "condition": meta["condition"],
                    "level": meta.get("level"),
                    "trial": meta.get("trial"),
                })
            n_selected += 1

        print(f"     Selected {n_selected} trials for current fold")

    if return_metadata:
        return X_all, Y_all, meta_all
    return X_all, Y_all
