# Databank — H5 motion-data integrity workflow

## Files

| file | purpose |
|---|---|
| `combined_data_S<NN>.h5` | source data (one or more, holding subjects S001..S<NN>) |
| `combined_data_S<NN>_clean.h5` | spike-removed copy (auto-generated) |
| `data_integrity_check.py` | runs the full integrity check, produces report + plots |
| `postprocess_clean.py` | removes catastrophic single-sample spikes (interp) |
| `integrity_report/` | regenerated each run — report.md / summary.json / plots/ |

## Workflow for a new H5 file

```bash
# 1. drop the new file in this folder. If its name is `combined_data_<NN>.h5`
#    (no leading 'S'), the integrity script auto-renames it once upload finishes.

# 2. (optional) clean catastrophic spikes — only needed if the file has
#    sensor-overflow values like 1e30+
python Databank/postprocess_clean.py
#    -> writes combined_data_S<NN>_clean.h5 (original kept untouched)

# 3. run the integrity check (auto-uses the _clean version when present)
python Databank/data_integrity_check.py
#    -> integrity_report/report.md
#    -> integrity_report/summary.json
#    -> integrity_report/plots/<file_stem>/{coverage,lr_symmetry,
#                                            imu_mount,anomalies_summary}.png
#    -> integrity_report/plots/<file_stem>/S001/{timeseries,gaitcycle}_*.png
```

## What the integrity check verifies

### per-trial (verdict: PASS / WARN / FAIL)
- 5 required groups present (`common`, `robot`, `treadmill`, `forceplate`, `mocap`)
- no NaN burst (`<5%`) or flat-line on critical channels
- monotonic time, dt jitter, loop_count step consistency
- condition label vs treadmill speed/pitch (e.g. level_100mps must read ≈1 m/s)
- robot motor_angle ↔ mocap hip_flexion correlation (segment-splice detector)
- GRF total peak / body weight ratio
- quaternion |q| ≈ 1 (back IMU)
- `gait_stage` 1→2→3→4 cycle order
- forceplate baseline drift (head vs tail of trial)
- IMU gyro_z bias drift
- robot vs GRF stride-frequency mismatch (S003-style splice)

### per-subject
- `sub_info` plausibility (age/height/weight/sex/leg lengths)
- L/R RMS symmetry on level conditions
- trunk-IMU mount tilt (gravity vector → off-vertical angle, lateral lean)
- protocol compliance (expected trial count per condition vs actual)

### cross-subject
- robust z-score on subject-mean of every distribution channel
- subject-wide hip/knee baseline offsets (S004/S007/S017-style pelvis offsets)

## Configuration

Edit thresholds at the top of `data_integrity_check.py`:
- `EXPECTED_CONDITIONS` — protocol definition (conditions / asls / trials per asl)
- `TH` — every numeric threshold (NaN fraction, |q| tolerance, drift limits, etc.)
- `PLOT_SUBJECT` — whose detailed timeseries/gaitcycle PNGs to draw

## Verdict levels

- **PASS** — clean
- **WARN** — flag-worthy condition or quality issue, but data usable
- **FAIL** — missing critical group or channel, flat-line, or excessive NaN
