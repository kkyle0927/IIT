# exp_S2A1_G2_fullkin_deeptcn

- Rank: 4
- Experiment dir: `exp_S2A1_G2_fullkin_deeptcn_Test-m2_S008_seed42`
- Model type: `TCN_GRU`
- Window size: `300`
- MAE: `0.062681`
- RMSE: `0.098396`
- Latency (ms): `3.1564`
- Params: `1602177`

## Input feature groups
- `derived/body_frame_imu`: accel_right, accel_forward, accel_up, gyro_right, gyro_forward, gyro_up, forward_accel_overground
- `robot/left`: hip_angle, hip_angle_dot, hip_angle_ddot, thigh_angle, thigh_angle_dot, thigh_angle_ddot, torque
- `robot/right`: hip_angle, hip_angle_dot, hip_angle_ddot, thigh_angle, thigh_angle_dot, thigh_angle_ddot, torque
- `sub_info`: height, weight

## Current interpretation
- deep TCN 계열의 강한 reference baseline.
- 공통적으로 assisted lv4/lv7 조건에서 underprediction 여부를 plot으로 확인할 필요가 있다.
