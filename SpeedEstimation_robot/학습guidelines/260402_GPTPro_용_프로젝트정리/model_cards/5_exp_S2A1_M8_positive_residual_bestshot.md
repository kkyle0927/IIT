# exp_S2A1_M8_positive_residual_bestshot

- Rank: 5
- Experiment dir: `exp_S2A1_M8_positive_residual_bestshot_Test-m2_S008_seed42`
- Model type: `PositiveResidualTCN_GRU`
- Window size: `320`
- MAE: `0.063931`
- RMSE: `0.091289`
- Latency (ms): `3.3945`
- Params: `229676`

## Input feature groups
- `derived/body_frame_imu`: accel_right, accel_forward, accel_up, gyro_right, gyro_forward, gyro_up, forward_accel_overground
- `derived/imu_integrated_speed`: imu_integrated_speed
- `derived/robot_biomech_rt`: phase_sin_l, phase_cos_l, phase_sin_r, phase_cos_r, cadence_l, cadence_r
- `robot/left`: hip_angle, hip_angle_dot, hip_angle_ddot, thigh_angle, thigh_angle_dot, thigh_angle_ddot
- `robot/right`: hip_angle, hip_angle_dot, hip_angle_ddot, thigh_angle, thigh_angle_dot, thigh_angle_ddot
- `sub_info`: height, weight
- `robot/left`: motor_angle, torque
- `robot/right`: motor_angle, torque
- `derived/robot_biomech_rt`: deflection_l, deflection_r, deflection_vel_l, deflection_vel_r, deflection_rms_l, deflection_rms_r, motor_power_l, motor_power_r, power_prop_sum, power_brake_sum, torque_abs_ema_fast_sum, torque_abs_ema_slow_sum, assist_proxy_sum, torque_stride_impulse_sum

## Current interpretation
- positive residual 계열. RMSE가 특히 좋은 편.
- 공통적으로 assisted lv4/lv7 조건에서 underprediction 여부를 plot으로 확인할 필요가 있다.
