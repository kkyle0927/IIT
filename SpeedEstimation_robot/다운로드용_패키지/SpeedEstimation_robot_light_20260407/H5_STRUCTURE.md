# combined_data_S008.h5 Structure

This package does **not** include the full `combined_data_S008.h5` file to keep download size small.
Instead, this document summarizes the dataset structure used by the project.

Resolved source path on server:
- `/home/chanyoungko/IIT/combined_data_S008.h5`

## Purpose

`combined_data_S008.h5` is the unified source dataset for this project.
It contains:
- robot/exoskeleton sensor streams
- trunk IMU streams
- treadmill signals
- common reference signals
- forceplate signals
- mocap signals
- subject anthropometric metadata

In the project, **deployment-time input** is restricted to:
- `robot/back_imu`
- `robot/left`
- `robot/right`
- selected `sub_info`

The project target is based on `common/speed_y` semantics.
In the actual H5 file, this is stored as:
- `common/v_Y_true`

## Top-Level Hierarchy

Root keys:
- `S001`
- `S002`
- `S003`
- `S004`
- `S005`
- `S006`
- `S007`
- `S008`

Each subject contains:
- conditions:
  - `accel_sine`
  - `decline_5deg`
  - `incline_10deg`
  - `level_075mps`
  - `level_100mps`
  - `level_125mps`
  - `stopandgo`
- `sub_info`

## Trial Hierarchy

Typical trial path:

```text
S001/level_100mps/lv4/trial_01
```

Under each trial, the main groups are:
- `common`
- `forceplate`
- `mocap`
- `robot`
- `treadmill`

## Subject Metadata

Example path:

```text
S001/sub_info
```

Contents:
- scalar datasets:
  - `age`
  - `height`
  - `sex`
  - `weight`
- sided groups:
  - `left`
  - `right`

Example sided anthropometry entries:
- `ankle width`
- `elbow width`
- `hand thickness`
- `knee width`
- `leg length`
- `shoulder offset`
- `wrist width`

## common Group

Example path:

```text
S001/level_100mps/lv4/trial_01/common
```

Observed datasets:
- `a_Y_true`
- `a_Z_true`
- `assist_level`
- `gait_stage`
- `loop_count`
- `mode`
- `time`
- `v_Y_true`
- `v_Z_true`

Project notes:
- target speed uses `common/speed_y` semantics
- actual stored dataset is `common/v_Y_true`
- `common/assist_level` exists in H5 but is **not used directly** as a model input

## robot Group

Example path:

```text
S001/level_100mps/lv4/trial_01/robot
```

Subgroups:
- `back_imu`
- `hip_imu`
- `insole`
- `left`
- `right`

### robot/back_imu

Datasets:
- `accel_x`
- `accel_y`
- `accel_z`
- `gyro_x`
- `gyro_y`
- `gyro_z`
- `quat_w`
- `quat_x`
- `quat_y`
- `quat_z`

### robot/left and robot/right

Datasets:
- `hip_angle`
- `motor_angle`
- `thigh_angle`
- `torque`

These are the main exoskeleton-side signals used by the project.

## treadmill Group

Example path:

```text
S001/level_100mps/lv4/trial_01/treadmill
```

Subgroups:
- `left`
- `right`
- `pitch`

These signals exist in the raw H5 but are not used as direct deployment-time model inputs.

## forceplate Group

Example path:

```text
S001/level_100mps/lv4/trial_01/forceplate
```

Subgroups:
- `cop`
- `grf`
- `moment`

These are useful as privileged/reference signals, but not valid deployment-time inputs.

## mocap Group

Example path:

```text
S001/level_100mps/lv4/trial_01/mocap
```

Subgroups:
- `joint_moment`
- `kin_q`
- `marker`

These are also privileged/reference signals only.

## Example Dataset Shapes

Observed example trial:

```text
S001/level_100mps/lv4/trial_01
```

Example shapes:
- `common/v_Y_true`: `(19274,)`
- `robot/back_imu/accel_x`: `(19274,)`
- `robot/left/hip_angle`: `(19274,)`
- `sub_info/height`: scalar dataset

This is consistent with synchronized 1D time series per trial, plus subject-level metadata.

## What the Project Derives From H5

From the deployment-allowed inputs, the project derives:
- body-frame IMU
- forward-axis aligned acceleration/gyro
- causal IMU-integrated speed prior
- joint derivatives
- pseudo gait phase / cadence
- signed power
- propulsion / braking proxies
- motor-angle deflection
- torque envelope / EMA / RMS
- stride impulse

Important rule:
- label/reference preprocessing may be offline and bidirectional
- input feature preprocessing must remain causal / online-feasible

