import argparse
import json
from pathlib import Path

import pandas as pd


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--archive-dir", required=True)
    parser.add_argument("--output-dir", required=True)
    parser.add_argument("--prefixes", nargs="+", required=True)
    args = parser.parse_args()

    archive_dir = Path(args.archive_dir)
    output_dir = Path(args.output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)

    rows = []
    for prefix in args.prefixes:
        for metrics_path in sorted(archive_dir.glob(f"{prefix}_Test-*_seed*/metrics.json")):
            exp_dir = metrics_path.parent
            fold = exp_dir.name.split("_Test-")[-1].split("_seed")[0]
            with open(metrics_path, "r", encoding="utf-8") as f:
                m = json.load(f)
            rows.append(
                {
                    "experiment": prefix,
                    "fold": fold,
                    "test_mae": m.get("test_mae"),
                    "test_rmse": m.get("test_rmse"),
                    "test_huber": m.get("test_huber"),
                    "n_params": m.get("n_params"),
                    "model_size_mb": m.get("model_size_mb"),
                    "host_latency_ms": m.get("host_latency_ms"),
                    "host_max_freq_hz": m.get("host_max_freq_hz"),
                    "metric_source": "metrics_json_fallback",
                }
            )

    df = pd.DataFrame(rows)
    if df.empty:
        raise SystemExit("No full LOSO metrics found")

    df.to_csv(output_dir / "fold_metrics.csv", index=False)

    summary = (
        df.groupby("experiment", as_index=False)
        .agg(
            n_folds=("fold", "count"),
            mae_mean=("test_mae", "mean"),
            mae_std=("test_mae", "std"),
            rmse_mean=("test_rmse", "mean"),
            rmse_std=("test_rmse", "std"),
            huber_mean=("test_huber", "mean"),
            huber_std=("test_huber", "std"),
            params_mean=("n_params", "mean"),
            model_size_mb_mean=("model_size_mb", "mean"),
            host_latency_ms_mean=("host_latency_ms", "mean"),
            host_max_freq_hz_mean=("host_max_freq_hz", "mean"),
        )
        .sort_values("mae_mean")
    )
    summary.to_csv(output_dir / "summary_mean_std.csv", index=False)
    print(summary.to_string(index=False))


if __name__ == "__main__":
    main()
