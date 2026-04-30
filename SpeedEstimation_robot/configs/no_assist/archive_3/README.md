`archive_3` is used for lightweight single-branch no-assist screening around
true `tilt-only local 2D` IMU features.

Goal:
- keep yaw/local horizontal orientation
- correct only roll/pitch using the current up axis
- avoid stale forward-axis projection
- stay causal and real-time friendly

Archive 3 experiments:
- `T1`: tilt-only local base14 (`imu2d + vel2d + raw hip + anchors`)
- `T2`: no-anchor12 (`imu2d + vel2d + raw hip`)
- `T3`: linvel10 (`imu2d + vel2d + hip_linvel only`)
- `T4`: base14 with smaller channels (`XS`)
- `T5`: base14 with shorter window (`200`)
- `T6`: minimal8 with shorter window (`imu2d + hip_linvel`, `200`)

All archive_3 runs are screening-only:
- representative LOSO folds: `m2_S001`, `m2_S004`, `m2_S008`
- `lv0` only (`noassist`)
