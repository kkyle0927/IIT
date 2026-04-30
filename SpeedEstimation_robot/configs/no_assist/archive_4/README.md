`archive_4` is used for lightweight `tilt-only local 2D` improvement screening.

Goal:
- improve `T1_tiltbase14` accuracy
- reduce noisy speed output
- keep parameter/input increase minimal
- remain causal and real-time friendly

Archive 4 experiments:
- `U1`: residual14 (`speed_anchor_geom` residual skip)
- `U2`: big14 (slightly larger TCN channels)
- `U3`: smoothgru14 (small causal smoothing GRU head)
- `U4`: mag16 (`acc_h_mag`, `vel_h_mag` added)
- `U5`: evalclamp14 (evaluation-time acceleration clamp)
- `U6`: thigh16 (left/right `thigh_angle_linvel` added)

All archive_4 runs are screening-only:
- representative LOSO folds: `m2_S001`, `m2_S004`, `m2_S008`
- `lv0` only (`noassist`)
