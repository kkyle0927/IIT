Archive 7: complementary-filter modifications around the V5 backbone.

Common baseline:
- no-assist only (`lv0`)
- single-branch `CausalSmoothTCN_GRU`
- channels `[16,16,32,32]`, `gru_hidden=12`, `smooth_window=9`
- screening LOSO subjects: `m2_S001`, `m2_S004`, `m2_S008`

Variants:
- D1: fixed complementary filter (`alpha=0.985`) baseline
- D2: adaptive accel-trust complementary filter
- D3: time-varying complementary filter (paper-inspired)
- D4: time-varying complementary filter with wider cutoff span
- D5: time-varying complementary filter without velocity features
- D6: time-varying complementary filter with `roll_cf`, `pitch_cf` inputs
