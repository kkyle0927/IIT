This package is a lightweight export of SpeedEstimation_robot for off-server review.
Included:
- source code, configs, docs, analyses
- experiment-level metadata: config.yaml, metrics.json, train_log.csv, env.txt
- compare_result CSV summaries only
Excluded to keep size small:
- logs/*.out, *.err, worker *.log
- model.pt, scaler.npz, any large binary artifacts
- png figures
- h5 datasets
- __pycache__ and git metadata
