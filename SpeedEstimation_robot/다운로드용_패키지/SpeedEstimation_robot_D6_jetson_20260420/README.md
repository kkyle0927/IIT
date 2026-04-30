# SpeedEstimation_robot D6 Jetson Package

This package bundles the no-assist `D6_tvcf_roll18` model family for Jetson-side inference.

## Included

- `checkpoints/full_loso/`
  - 8 full-LOSO checkpoints
  - each bundle includes `model.pt`, `scaler.npz`, `config.yaml`, `metrics.json`, `env.txt`
- `results/archive_8/`
  - full-LOSO summaries
  - per-fold `condition_r2.png`
  - per-fold `trajectory.png`
- `src/`
  - minimal source files required to reconstruct the model and feature preprocessing
- `scripts/d6_inference.py`
  - reference inference script for offline or stream-simulated use
- `INFERENCE_GUIDE.md`
  - exact feature definitions, sampling, timing, and preprocessing notes
- `requirements_runtime.txt`
  - minimal Python runtime dependencies for this package

## Model Summary

- Model family: `CausalSmoothTCN_GRU`
- Input feature count: `18`
- Window size: `300` samples
- Sampling rate: `100 Hz`
- Effective input history: `3.0 s`
- Output target: `common/speed_y`
- Prediction offset: `est_tick_ranges=[5]`
  - model output corresponds to roughly `+50 ms` ahead of the last input sample

## Input Signals Required At Inference

The reference script expects an `npz` file containing:

- `accel_x`, `accel_y`, `accel_z`
- `gyro_x`, `gyro_y`, `gyro_z`
- `hip_angle_l`, `hip_angle_r`
- `thigh_angle_l`, `thigh_angle_r`
- `height_m` or `height_mm`

All time-series inputs must be sampled at `100 Hz`.

## Sensor Axis Convention

The back IMU mounting assumed by this package is:

- `+x`: head / up direction
- `+y`: body-right direction
- `+z`: forward direction

This axis convention is baked into the time-varying complementary filter used to compute `roll_cf` and `pitch_cf`.

## Which Checkpoint To Use

This package does **not** contain a single pooled model trained on all subjects together.
Instead, it contains the 8 checkpoints generated during full LOSO evaluation:

- `exp_NA_A8_D6_full_loso_noassist_part1_Test-m2_S001_seed42`
- `exp_NA_A8_D6_full_loso_noassist_part1_Test-m2_S002_seed42`
- `exp_NA_A8_D6_full_loso_noassist_part1_Test-m2_S003_seed42`
- `exp_NA_A8_D6_full_loso_noassist_part1_Test-m2_S004_seed42`
- `exp_NA_A8_D6_full_loso_noassist_part2_Test-m2_S005_seed42`
- `exp_NA_A8_D6_full_loso_noassist_part2_Test-m2_S006_seed42`
- `exp_NA_A8_D6_full_loso_noassist_part2_Test-m2_S007_seed42`
- `exp_NA_A8_D6_full_loso_noassist_part2_Test-m2_S008_seed42`

Use `results/archive_8/fast_eval_metrics.csv` to inspect fold-wise quality.

## Quick Start

```bash
python scripts/d6_inference.py \
  --bundle-dir checkpoints/full_loso/exp_NA_A8_D6_full_loso_noassist_part1_Test-m2_S001_seed42 \
  --input-npz your_input_stream.npz \
  --output-npz d6_predictions.npz \
  --device cpu
```

The script:

1. builds the 18 input features causally from raw signals
2. normalizes them with `scaler.npz`
3. runs the checkpoint on sliding 300-sample windows
4. writes predictions aligned to the training-time `+50 ms` target convention

## Full-LOSO Reference Performance

8-fold average from `results/archive_8/fast_eval_metrics.csv`:

- `MAE = 0.0605`
- `RMSE = 0.0852`
- `R² = 0.9345`

## Notes For Jetson Deployment

- The reference script is correctness-first, not the most optimized implementation.
- It recomputes the full feature stream from arrays, which is useful for validation.
- For true streaming deployment on Jetson, convert the helper logic in `scripts/d6_inference.py` into a stateful online processor.
- All preprocessing used here is causal or online-friendly.

