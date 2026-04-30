# exp_S2A1_J1_hybrid_power_base

- Rank: 2
- Experiment dir: `exp_S2A1_J1_hybrid_power_base_Test-m2_S008_seed42`
- Model type: `DualBranchTCN_GRU`
- Window size: `220`
- MAE: `0.062299`
- RMSE: `0.098349`
- Latency (ms): `3.0071`
- Params: `80345`

## Input feature groups
- `derived/body_frame_imu`: accel_right, accel_forward, accel_up, gyro_right, gyro_forward, gyro_up, forward_accel_overground
- `derived/imu_integrated_speed`: imu_integrated_speed
- `derived/robot_biomech_rt`: hip_power_l, hip_power_r, thigh_power_l, thigh_power_r
- `robot/left`: hip_angle, hip_angle_dot, hip_angle_ddot, thigh_angle, thigh_angle_dot, thigh_angle_ddot, torque
- `robot/right`: hip_angle, hip_angle_dot, hip_angle_ddot, thigh_angle, thigh_angle_dot, thigh_angle_ddot, torque
- `sub_info`: height, weight

## Current interpretation
- 가장 단순한 hybrid 계열 중 하나. 작고 빠른 baseline 대안.
- 공통적으로 assisted lv4/lv7 조건에서 underprediction 여부를 plot으로 확인할 필요가 있다.
