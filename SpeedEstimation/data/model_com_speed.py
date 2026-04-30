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
    height = float(height_mm) / 1000.0  # mm -> m
    N = len(hip_flex_l)

    # Segment lengths (anthropometric ratios)
    L_thigh = 0.245 * height
    L_shank = 0.246 * height
    W_pelvis = 0.146 * height  # inter-ASIS width

    # Convert to radians
    hip_l = np.deg2rad(hip_flex_l.flatten().astype(np.float64))
    hip_r = np.deg2rad(hip_flex_r.flatten().astype(np.float64))
    knee_l = np.deg2rad(knee_angle_l.flatten().astype(np.float64))
    knee_r = np.deg2rad(knee_angle_r.flatten().astype(np.float64))
    pel_pitch = np.deg2rad(pelvis_tilt.flatten().astype(np.float64))

    # Pelvis yaw (remove constant offset, keep variation)
    pel_yaw = np.deg2rad(pelvis_rotation.flatten().astype(np.float64))
    pel_yaw = pel_yaw - np.mean(pel_yaw)

    # Absolute segment angles from vertical (positive = tilted forward)
    # Include pelvis pitch: thigh angle = pelvis_pitch + hip_flexion
    theta_thigh_l = pel_pitch + hip_l
    theta_shank_l = pel_pitch + hip_l - knee_l
    theta_thigh_r = pel_pitch + hip_r
    theta_shank_r = pel_pitch + hip_r - knee_r

    # Forward position of hip relative to ankle (sagittal plane)
    # Going up from ankle: knee is BEHIND ankle when leg is forward (heel strike),
    # so the sign is negative: x_hip = -(shank + thigh) contribution
    # At heel strike (θ>0): hip is behind ankle → x_hip < 0
    # At toe-off (θ<0): hip is in front of ankle → x_hip > 0
    # v_com = dx/dt is positive during stance → matches belt speed
    x_hip_l = -(L_shank * np.sin(theta_shank_l) + L_thigh * np.sin(theta_thigh_l))
    x_hip_r = -(L_shank * np.sin(theta_shank_r) + L_thigh * np.sin(theta_thigh_r))

    # Velocity of hip relative to ankle
    dt = 1.0 / fs
    v_hip_l = np.gradient(x_hip_l, dt)
    v_hip_r = np.gradient(x_hip_r, dt)

    # Pelvis center velocity relative to stance hip
    # Left stance:  pelvis center is (W/2) to the right → yaw rotation moves it forward
    # Right stance: pelvis center is (W/2) to the left  → opposite sign
    pel_yaw_rate = np.gradient(pel_yaw, dt)
    v_pelvis_from_l = (W_pelvis / 2.0) * pel_yaw_rate
    v_pelvis_from_r = -(W_pelvis / 2.0) * pel_yaw_rate

    # Total COM speed from each leg chain
    v_com_left = v_hip_l + v_pelvis_from_l
    v_com_right = v_hip_r + v_pelvis_from_r

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
