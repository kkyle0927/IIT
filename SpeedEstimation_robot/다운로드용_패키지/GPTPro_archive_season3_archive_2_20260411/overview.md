# Overview

## Project
This project estimates human gait speed from:
- exoskeleton robot sensors
- trunk IMU
- subject anthropometrics

The target is not treadmill belt speed. The target is `common/speed_y` smoothed with offline bidirectional low-pass filtering.

## Allowed raw inputs
- `robot/left`, `robot/right`: hip angle, thigh angle, torque and causal derivatives
- `robot/back_imu`: accel, gyro, quaternion
- `sub_info`: height, weight, leg-length features

## Allowed derived inputs
- quaternion-based body-frame IMU
- `forward_accel_overground`
- IMU integrated speed prior
- cadence / step-time / excursion / trunk harmonic features
- interaction summaries such as power, deflection, stiffness-like proxy, torque EMA, stride impulse, guarded torque preview

## Disallowed inputs
- mocap
- GRF / forceplate
- direct treadmill speed input
- any derived feature that leaks those signals
- `common/assist_level` as an inference input

## Why this archive exists
Previous archives improved overall MAE, but a recurring failure remained:
- in `level_100mps` and `level_125mps`
- especially `lv4` and `lv7`
- the estimate often sat at a shifted plateau level

This archive focuses on structural fixes for that offset:
- plateau-specific objective
- stronger absolute-speed anchor
- persistent slow bias state
- assist on/off routing
- regime mixture
- full assist off/on split
- hardware-inspired cadence/step-length anchor from the similar hip-assistive robot paper

## Main comparison question
Which structure reduces plateau offset without sacrificing fast response?

