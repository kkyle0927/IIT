A13: Additional contribution-validation 3-fold ablations for the selected og model.

Purpose
- Separate the contribution of explicit roll/pitch input from tilt calibration itself.
- Separate velocity-magnitude contribution from the compact magnitude block.
- Separate cadence-based speed anchor from cadence-only prior.

Common setup
- Baseline architecture: CausalSmoothTCN_GRU
- TCN channels: [16, 16, 32, 32]
- GRU hidden: 12
- Smoothing window: 9
- 3-fold LOSO subjects: m2_S001, m2_S004, m2_S008
- no_assist / lv0 only

Experiments
- C0_baseline18: current og baseline
- C1_no_rollpitch_input16: keep tilt-calibrated IMU, remove roll_cf/pitch_cf from input only
- C2_no_vel_h_mag17: remove vel_h_mag only
- C3_no_mag_features16: remove acc_h_mag and vel_h_mag
- C4_speed_anchor_only17: keep speed_anchor_geom, remove cadence_mean
- C5_cadence_only17: keep cadence_mean, remove speed_anchor_geom
