# exp_S2A1_J7_hybrid_longwin_envelope

- Rank: 1
- Experiment dir: `exp_S2A1_J7_hybrid_longwin_envelope_Test-m2_S008_seed42`
- Model type: `DualBranchTCN_GRU`
- Window size: `300`
- MAE: `0.060998`
- RMSE: `0.095920`
- Latency (ms): `3.2312`
- Params: `176937`

## Input feature groups
- `derived/body_frame_imu`: accel_right, accel_forward, accel_up, gyro_right, gyro_forward, gyro_up, forward_accel_overground
- `derived/imu_integrated_speed`: imu_integrated_speed
- `derived/robot_biomech_rt`: phase_sin_l, phase_cos_l, phase_sin_r, phase_cos_r, cadence_l, cadence_r, hip_power_l, hip_power_r, thigh_power_l, thigh_power_r, torque_rms_l, torque_rms_r, thigh_vel_rms_l, thigh_vel_rms_r, gyro_forward_rms
- `robot/left`: hip_angle, hip_angle_dot, hip_angle_ddot, thigh_angle, thigh_angle_dot, thigh_angle_ddot, torque
- `robot/right`: hip_angle, hip_angle_dot, hip_angle_ddot, thigh_angle, thigh_angle_dot, thigh_angle_ddot, torque
- `sub_info`: height, weight

## Current interpretation
- 가장 안정적인 메인 후보. long-window + power/envelope hybrid.
- 공통적으로 assisted lv4/lv7 조건에서 underprediction 여부를 plot으로 확인할 필요가 있다.
