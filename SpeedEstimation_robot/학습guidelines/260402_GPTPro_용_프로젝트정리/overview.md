# 프로젝트 개요

## 목표
- Exoskeleton 센서와 trunk IMU만 사용해 human gait speed를 추정한다.
- 최종 target은 treadmill belt speed가 아니라 `common/speed_y`에 bidirectional LPF를 적용한 값이다.

## 입력
- Exoskeleton:
  - 좌/우 `hip_angle`
  - 좌/우 `thigh_angle`
  - 좌/우 `motor_angle`
  - 좌/우 `torque`
- Trunk IMU (`robot/back_imu`):
  - 3축 acceleration
  - 3축 angular velocity
  - quaternion
- Subject info:
  - `height`, `weight` 등 일부 anthropometric 정보

## 주요 파생 feature
- `derived/body_frame_imu`
  - quaternion 기반 body-frame 변환
  - `accel_right/forward/up`, `gyro_right/forward/up`
  - `forward_accel_overground`
- `derived/imu_integrated_speed`
  - forward acceleration의 causal integration 기반 speed prior
- `derived/robot_biomech_rt`
  - phase / cadence
  - signed power
  - torque RMS / envelope
  - motor-angle 기반 deflection
  - torque-context / stride impulse

## 실시간성 제약
- 입력 feature 전처리는 항상 causal / online 가능해야 한다.
- label 생성용 bidirectional LPF는 허용된다.
- 모델은 latency, parameter count, deployability를 함께 고려한다.

## 현재 문제 정의
- 평균 MAE는 `0.06` 수준까지 내려왔지만, assisted condition에서 systematic underprediction이 남는다.
- 특히 `lv4`, `lv7`의 `level_100mps`, `level_125mps`, `stopandgo`에서 plateau가 낮게 깔리는 양상이 반복된다.
- 이 문제는 단순 random noise라기보다 assist-dependent domain shift / calibration failure로 해석하고 있다.

## season2 기준 상위 5개
- `J7_hybrid_longwin_envelope`
- `J1_hybrid_power_base`
- `L7_affine_calibration_biasloss`
- `G2_fullkin_deeptcn`
- `M8_positive_residual_bestshot`
