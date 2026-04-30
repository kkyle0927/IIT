"""
Model-based COM forward speed from joint kinematics.

Computes pelvis center velocity relative to stance foot using:
  ankle → knee → hip → pelvis center (forward kinematics chain)

Single stance: stance leg chain only
Double stance: average of both legs

Segment lengths from anthropometric ratios (height-based).
"""

import numpy as np
from scipy.signal import butter, sosfilt


def _causal_lpf(signal, cutoff, fs, order=4):
    """Apply causal (real-time) low-pass Butterworth filter using sosfilt."""
    sos = butter(order, cutoff, btype='low', fs=fs, output='sos')
    return sosfilt(sos, signal)


def _sigmoid(x):
    x = np.clip(np.asarray(x, dtype=np.float64), -30.0, 30.0)
    return 1.0 / (1.0 + np.exp(-x))


def _compute_chain_velocities(hip_flex_l, hip_flex_r,
                              knee_angle_l, knee_angle_r,
                              pelvis_tilt,
                              pelvis_rotation,
                              height_mm, fs):
    """Shared kinematic-chain COM velocity calculation without support blending."""
    height = float(height_mm) / 1000.0

    L_thigh = 0.245 * height
    L_shank = 0.246 * height
    W_pelvis = 0.146 * height

    hip_l = np.deg2rad(hip_flex_l.flatten().astype(np.float64))
    hip_r = np.deg2rad(hip_flex_r.flatten().astype(np.float64))
    knee_l = np.deg2rad(knee_angle_l.flatten().astype(np.float64))
    knee_r = np.deg2rad(knee_angle_r.flatten().astype(np.float64))
    pel_pitch = np.deg2rad(pelvis_tilt.flatten().astype(np.float64))
    pel_yaw = np.deg2rad(pelvis_rotation.flatten().astype(np.float64))
    pel_yaw = pel_yaw - np.mean(pel_yaw)

    theta_thigh_l = pel_pitch + hip_l
    theta_shank_l = pel_pitch + hip_l - knee_l
    theta_thigh_r = pel_pitch + hip_r
    theta_shank_r = pel_pitch + hip_r - knee_r

    x_hip_l = -(L_shank * np.sin(theta_shank_l) + L_thigh * np.sin(theta_thigh_l))
    x_hip_r = -(L_shank * np.sin(theta_shank_r) + L_thigh * np.sin(theta_thigh_r))

    dt = 1.0 / fs
    v_hip_l = np.gradient(x_hip_l, dt)
    v_hip_r = np.gradient(x_hip_r, dt)

    pel_yaw_rate = np.gradient(pel_yaw, dt)
    v_pelvis_from_l = (W_pelvis / 2.0) * pel_yaw_rate
    v_pelvis_from_r = -(W_pelvis / 2.0) * pel_yaw_rate

    v_com_left = v_hip_l + v_pelvis_from_l
    v_com_right = v_hip_r + v_pelvis_from_r
    return v_com_left, v_com_right


def compute_contact_probability_phase(hip_flex_l, hip_flex_r,
                                      fs, cutoff=2.0, order=2, gain=0.12):
    """
    Soft left/right support probabilities from left-right hip phase difference.
    Uses only causal smoothing and current kinematics.
    """
    hip_l = np.asarray(hip_flex_l, dtype=np.float64).flatten()
    hip_r = np.asarray(hip_flex_r, dtype=np.float64).flatten()
    phase_signal = hip_r - hip_l
    score_l = -gain * phase_signal
    p_left = _sigmoid(score_l)
    p_left = _causal_lpf(p_left, cutoff, fs, order)
    p_left = np.clip(p_left, 1e-3, 1.0 - 1e-3)
    p_right = 1.0 - p_left
    return p_left.astype(np.float32), p_right.astype(np.float32)


def compute_contact_probability_kinematics(hip_flex_l, hip_flex_r,
                                           knee_angle_l, knee_angle_r,
                                           fs, cutoff=2.0, order=2):
    """
    Left/right support probabilities from causal kinematic cues.
    Hip phase difference captures left-right stance asymmetry, while hip/knee
    angular velocity asymmetry sharpens transitions around stance exchange.
    """
    hip_l = np.asarray(hip_flex_l, dtype=np.float64).flatten()
    hip_r = np.asarray(hip_flex_r, dtype=np.float64).flatten()
    knee_l = np.asarray(knee_angle_l, dtype=np.float64).flatten()
    knee_r = np.asarray(knee_angle_r, dtype=np.float64).flatten()

    dhip_l = np.gradient(hip_l) * fs
    dhip_r = np.gradient(hip_r) * fs
    dknee_l = np.gradient(knee_l) * fs
    dknee_r = np.gradient(knee_r) * fs

    phase_term = -0.10 * (hip_r - hip_l) - 0.04 * (knee_r - knee_l)
    vel_term = -0.015 * ((dhip_l - dhip_r) + 0.5 * (dknee_l - dknee_r))
    score_l = phase_term + vel_term

    p_left = _sigmoid(score_l)
    p_left = _causal_lpf(p_left, cutoff, fs, order)
    p_left = np.clip(p_left, 1e-3, 1.0 - 1e-3)
    p_right = 1.0 - p_left
    return p_left.astype(np.float32), p_right.astype(np.float32)


def compute_contact_probability_fusion(hip_flex_l, hip_flex_r,
                                       knee_angle_l, knee_angle_r,
                                       acc_forward_body, acc_up_body,
                                       gyro_forward, fs, cutoff=2.0, order=2):
    """
    Kinematic + IMU fusion support probabilities.
    Kinematics decide left-vs-right side, and trunk IMU provides stance
    confidence so the estimate becomes less binary during flight-like or
    highly dynamic transitions.
    """
    p_left_kin, p_right_kin = compute_contact_probability_kinematics(
        hip_flex_l, hip_flex_r, knee_angle_l, knee_angle_r, fs, cutoff=cutoff, order=order
    )

    acc_f = np.asarray(acc_forward_body, dtype=np.float64).flatten()
    acc_u = np.asarray(acc_up_body, dtype=np.float64).flatten()
    gyr_f = np.asarray(gyro_forward, dtype=np.float64).flatten()

    acc_energy = np.abs(acc_f) + 0.5 * np.abs(acc_u)
    gyro_energy = np.abs(gyr_f)
    motion_energy = _causal_lpf(acc_energy + 0.4 * gyro_energy, 4.0, fs, order=2)
    stance_conf = _sigmoid(1.5 - 0.35 * motion_energy)
    stance_conf = np.clip(stance_conf, 0.1, 0.95)

    p_left = stance_conf * p_left_kin + (1.0 - stance_conf) * 0.5
    p_left = _causal_lpf(p_left, cutoff, fs, order)
    p_left = np.clip(p_left, 1e-3, 1.0 - 1e-3)
    p_right = 1.0 - p_left
    return p_left.astype(np.float32), p_right.astype(np.float32)


def compute_model_com_speed(hip_flex_l, hip_flex_r,
                            knee_angle_l, knee_angle_r,
                            pelvis_tilt,
                            pelvis_rotation,
                            grf_z_l, grf_z_r,
                            height_mm, fs,
                            contact_threshold=-20.0,
                            lpf_cutoff=0.5, lpf_order=4):
    """
    Compute model-based COM forward speed from joint kinematics.

    Args:
        hip_flex_l/r:    (N,) hip flexion [deg], positive = flexion (forward)
        knee_angle_l/r:  (N,) knee flexion [deg], positive = flexion (bent)
        pelvis_tilt:     (N,) pelvis sagittal tilt [deg] (OpenSim: anterior +)
        pelvis_rotation: (N,) pelvis yaw [deg] (transverse rotation)
        grf_z_l/r:       (N,) vertical GRF [N], negative = contact force
        height_mm:       scalar, subject height [mm]
        fs:              sampling frequency [Hz]
        contact_threshold: GRF threshold for stance detection
        lpf_cutoff:      causal LPF cutoff [Hz] (matches GT speed preprocessing)
        lpf_order:       LPF filter order

    Returns:
        com_speed: (N,) model-based COM forward speed [m/s]
    """
    N = len(hip_flex_l)
    v_com_left, v_com_right = _compute_chain_velocities(
        hip_flex_l, hip_flex_r, knee_angle_l, knee_angle_r,
        pelvis_tilt, pelvis_rotation, height_mm, fs
    )

    # Stance detection from GRF
    cl = (grf_z_l.flatten() < contact_threshold)
    cr = (grf_z_r.flatten() < contact_threshold)

    both = cl & cr
    left_only = cl & ~cr
    right_only = ~cl & cr
    neither = ~cl & ~cr

    com_speed = np.zeros(N, dtype=np.float64)
    com_speed[left_only] = v_com_left[left_only]
    com_speed[right_only] = v_com_right[right_only]
    com_speed[both] = 0.5 * (v_com_left[both] + v_com_right[both])
    com_speed[neither] = 0.5 * (v_com_left[neither] + v_com_right[neither])

    # Causal LPF: same cutoff as GT speed, real-time applicable
    com_speed = _causal_lpf(com_speed, lpf_cutoff, fs, lpf_order)

    return com_speed.astype(np.float32)


def compute_model_com_speed_nogrf(hip_flex_l, hip_flex_r,
                                  knee_angle_l, knee_angle_r,
                                  pelvis_tilt,
                                  pelvis_rotation,
                                  height_mm, fs,
                                  contact_mode="fusion",
                                  acc_forward_body=None,
                                  acc_up_body=None,
                                  gyro_forward=None,
                                  lpf_cutoff=0.5, lpf_order=4):
    """
    GRF-free model-based COM speed.

    The left/right chain velocities are identical to the GRF-based model, but
    support blending is replaced by causal soft support probabilities from
    kinematic phase or kinematics+IMU fusion.
    """
    v_com_left, v_com_right = _compute_chain_velocities(
        hip_flex_l, hip_flex_r, knee_angle_l, knee_angle_r,
        pelvis_tilt, pelvis_rotation, height_mm, fs
    )

    mode = str(contact_mode).lower()
    if mode == "phase":
        p_left, p_right = compute_contact_probability_phase(hip_flex_l, hip_flex_r, fs)
    elif mode == "kin":
        p_left, p_right = compute_contact_probability_kinematics(
            hip_flex_l, hip_flex_r, knee_angle_l, knee_angle_r, fs
        )
    else:
        if acc_forward_body is None or acc_up_body is None or gyro_forward is None:
            p_left, p_right = compute_contact_probability_kinematics(
                hip_flex_l, hip_flex_r, knee_angle_l, knee_angle_r, fs
            )
        else:
            p_left, p_right = compute_contact_probability_fusion(
                hip_flex_l, hip_flex_r, knee_angle_l, knee_angle_r,
                acc_forward_body, acc_up_body, gyro_forward, fs
            )

    com_speed = p_left * v_com_left + p_right * v_com_right
    com_speed = _causal_lpf(com_speed, lpf_cutoff, fs, lpf_order)
    return com_speed.astype(np.float32)


def compute_imu_integrated_speed(acc_overground, fs,
                                 alpha=0.998, lpf_cutoff=0.5, lpf_order=4):
    """
    Integrate overground-equivalent forward acceleration to estimate speed.

    Input should be body-frame +Y acceleration + treadmill acceleration
    (= forward_accel_overground from body_frame module), which represents
    the acceleration as if walking overground.

    Uses leaky integrator to produce a bounded velocity estimate,
    then applies causal low-pass filter (same cutoff as GT speed).

    Args:
        acc_overground:   (N,) overground-equivalent forward accel [m/s^2]
                          (= body_frame acc_Y + a_treadmill)
        fs:               sampling frequency [Hz]
        alpha:            leaky integrator decay factor (0 < alpha < 1)
        lpf_cutoff:       causal LPF cutoff [Hz] (matches GT speed preprocessing)
        lpf_order:        LPF filter order

    Returns:
        imu_speed: (N,) integrated forward speed estimate [m/s]
    """
    acc = acc_overground.flatten().astype(np.float64)
    dt = 1.0 / fs

    # Leaky integrator: bounded integration without drift
    N = len(acc)
    vel = np.zeros(N, dtype=np.float64)
    for i in range(1, N):
        vel[i] = alpha * vel[i - 1] + acc[i] * dt

    # Causal LPF: same cutoff as GT speed, real-time applicable
    vel_smooth = _causal_lpf(vel, lpf_cutoff, fs, lpf_order)

    return vel_smooth.astype(np.float32)
