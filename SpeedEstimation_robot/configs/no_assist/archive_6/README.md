`archive_6` is used for complementary-filter calibration experiments around `V5`.

Goal:
- replace stationary/quaternion-based tilt correction with online complementary-filter roll/pitch
- test whether continuously updated tilt calibration improves direction robustness
- keep the same causal, deployable `tilt-only local 2D` philosophy

Archive 6 screening experiments:
- `C1`: cf_bigmag16 (`V5` with online complementary-filter tilt calibration)
- `C2`: cfslow_bigmag16 (slower complementary-filter correction)
- `C3`: cfroll18 (`C1` + explicit `roll_cf`, `pitch_cf` inputs)
- `C4`: cfslow_roll18 (`C2` + explicit `roll_cf`, `pitch_cf` inputs)

Legacy config kept for reference:
- `V5_full_loso`: previous full LOSO config for the archive_5 winner
