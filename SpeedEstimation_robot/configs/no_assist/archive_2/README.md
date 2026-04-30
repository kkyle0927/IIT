`archive_2` is used for lightweight single-branch no-assist screening around the
`LinvelA14` family.

Current flow:
- `archive_1`: earlier screening on lean/T1/statfix variants
- `archive_2`: `LinvelA14`-derived single-branch screening with heading/calibration ablations

Archive 2 experiments:
- `L1`: z-only gravity calibration + 2D horizontal IMU
- `L2`: L1 + horizontal integrated velocity
- `L3`: z-only + hip linear velocity only + anchors
- `L4`: z-only + raw hip only (no derived anchors)
- `L5`: z-only + horizontal integrated velocity + hip linear velocity only
- `L6`: z-only + raw hip + anchors + subject anthropometry

All archive_2 runs are screening-only:
- representative LOSO folds: `m2_S001`, `m2_S004`, `m2_S008`
- `lv0` only (`noassist`)
