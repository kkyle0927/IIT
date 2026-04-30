`archive_5` is used for combination screening around `U3_smoothgru14`.

Goal:
- keep the `CausalSmoothTCN_GRU` backbone from `U3`
- improve accuracy and output smoothness with minimal feature/model growth
- compare soft post-processing (`EMA`, soft clamp) against structural/input changes
- remain causal and real-time friendly

Archive 5 experiments:
- `V1`: smoothgru_big14 (`U3` + slightly larger channels)
- `V2`: smoothgru_mag16 (`U3` + `acc_h_mag`, `vel_h_mag`)
- `V3`: smoothgru14_ema (`U3` + eval-time EMA)
- `V4`: smoothgru14_softclamp08 (`U3` + eval-time soft acceleration clamp)
- `V5`: smoothgru_bigmag16 (`U3` + larger channels + magnitude features)
- `V6`: smoothgru_thigh16 (`U3` + left/right `thigh_angle_linvel`)

All archive_5 runs are screening-only:
- representative LOSO folds: `m2_S001`, `m2_S004`, `m2_S008`
- `lv0` only (`noassist`)
