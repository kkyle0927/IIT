# D6 Inference Guide

## 1. Sampling And Timing

- Sampling rate: `100 Hz`
- Input window: `300` samples
- Input duration: `3.0 s`
- Inference stride in evaluation: `1` sample
- Target horizon: `est_tick_ranges=[5]`
- Output convention: prediction at input end time `t` targets `speed_y[t+5]`

## 2. Input Feature Layout

The model consumes 18 channels in this exact order:

1. `acc_hx`
2. `acc_hy`
3. `acc_up`
4. `gyro_hx`
5. `gyro_hy`
6. `gyro_up`
7. `roll_cf`
8. `pitch_cf`
9. `vel_hx`
10. `vel_hy`
11. `acc_h_mag`
12. `vel_h_mag`
13. `hip_angle_l`
14. `hip_angle_linvel_l`
15. `hip_angle_r`
16. `hip_angle_linvel_r`
17. `cadence_mean`
18. `speed_anchor_geom`

## 3. Raw Inputs Needed

The reference pipeline needs:

- trunk IMU acceleration: `accel_x/y/z`
- trunk IMU gyro: `gyro_x/y/z`
- left/right hip angle
- left/right thigh angle
- subject height

No GRF, mocap, treadmill speed, assist metadata, or future samples are used.

## 4. TVCF Tilt Calibration

The D6 model uses a time-varying complementary filter to estimate online trunk tilt.

### Sensor mounting convention

- `+x`: up
- `+y`: right
- `+z`: forward

### Accelerometer tilt angles

```text
roll_acc  = atan2(a_y, a_x)
pitch_acc = atan2(-a_z, sqrt(a_x^2 + a_y^2))
```

### Gyro integration axes

- roll is integrated from `g_z`
- pitch is integrated from `g_y`

### TVCF parameters used for D6

- `mode = "tvcf"`
- `trust_ema = 0.92`
- `tvcf_omega_low = 0.5`
- `tvcf_omega_high = 2.5`
- `tvcf_bias_gamma = 5.0`
- `tvcf_sigma_floor_acc_g = 0.01`
- `tvcf_sigma_floor_gyro_deg = 1.0`
- `tvcf_sigma_floor_jerk_g = 0.5`

The filter adapts its cutoff from motion-dependent indicators derived from:

- gravity-magnitude error
- jerk magnitude
- tilt-gyro magnitude
- gyro derivative magnitude

## 5. Feature Formulas

### 5.1 Tilt-corrected IMU channels

The TVCF produces:

- `acc_hx`, `acc_hy`, `acc_up`
- `gyro_hx`, `gyro_hy`, `gyro_up`
- `roll_cf`, `pitch_cf`

These are computed by removing roll and pitch while preserving yaw.

### 5.2 Horizontal velocity channels

```text
vel_hx = leaky_integrate(acc_hx)
vel_hy = leaky_integrate(acc_hy)
```

Implementation details:

- leaky integrator `alpha = 0.998`
- causal Butterworth LPF
- cutoff `0.5 Hz`
- order `4`

### 5.3 Magnitude channels

```text
acc_h_mag = sqrt(acc_hx^2 + acc_hy^2)
vel_h_mag = sqrt(vel_hx^2 + vel_hy^2)
```

### 5.4 Hip linear velocity

```text
v_hip = l_leg * d/dt(theta_hip)
```

with degree-to-radian conversion:

```text
v_hip = l_leg * (pi/180) * d/dt(hip_angle)
```

Used constants:

- derivative: backward difference
- LPF after derivative: causal Butterworth, `30 Hz`, order `4`
- representative leg length:

```text
l_leg = max(0.53 * height_m, 0.5)
```

### 5.5 Cadence estimate

The thigh phase is built from centered thigh angle and thigh angular velocity:

```text
phase = unwrap(atan2(thigh_dot / thigh_vel_scale, thigh_center / thigh_amp))
phase_rate = d/dt(phase) / (2*pi)
cadence_mean = 0.5 * [ EMA(phase_rate_l) + EMA(phase_rate_r) ]
```

EMA time constant:

- `tau = 0.25 s`

Centering and amplitude statistics:

- center EMA time constant: `0.6 s`
- running RMS window: `0.6 s`

### 5.6 Geometric speed anchor

```text
step_length_proxy = l_leg * RMS(thigh_l - thigh_r)
speed_anchor_geom = cadence_mean * step_length_proxy
```

where `RMS` is a causal running RMS over a `0.5 s` window.

## 6. Normalization

Each checkpoint includes `scaler.npz` with:

- `mean`: shape `(18,)`
- `scale`: shape `(18,)`

Normalize inputs as:

```text
x_norm = (x - mean) / scale
```

The checkpoint expects normalized 18-channel windows of shape:

```text
(batch, 300, 18)
```

## 7. Model Architecture

- model type: `CausalSmoothTCN_GRU`
- channels: `[16, 16, 32, 32]`
- kernel size: `5`
- dropout: `0.15`
- input norm: `InstanceNorm1d`
- GRU hidden size: `12`
- causal smoothing window: `9`
- output dimension: `1`

## 8. Recommended Deployment Flow

1. stream raw signals at `100 Hz`
2. maintain causal feature states
3. build the 18-channel input vector each sample
4. normalize with checkpoint scaler
5. keep the latest 300 normalized samples
6. run the model every sample or every few samples
7. interpret output as roughly `+50 ms` ahead of the latest input

## 9. Important Cautions

- Do not change IMU axis convention without updating the TVCF equations.
- Do not replace causal filtering with zero-phase filtering for deployment.
- Do not assume the package contains a pooled universal model; it contains 8 LOSO fold checkpoints.

