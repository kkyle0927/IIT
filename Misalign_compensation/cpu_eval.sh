#!/bin/bash
#SBATCH -J misalign_cpu_eval
#SBATCH -N 1
#SBATCH -n 1
#SBATCH --cpus-per-task=4
#SBATCH --mem=5G
#SBATCH -t 12:00:00
#SBATCH -o logs/%x_%j.out
#SBATCH -e logs/%x_%j.err

export CUDA_VISIBLE_DEVICES=""
export OMP_NUM_THREADS=4
export MKL_NUM_THREADS=4
export NUMEXPR_NUM_THREADS=4
export MALLOC_ARENA_MAX=2
export MPLCONFIGDIR="/tmp/mpl-${SLURM_JOB_ID:-manual}"
export XDG_CACHE_HOME="/tmp/xdg-cache-${SLURM_JOB_ID:-manual}"
mkdir -p "$MPLCONFIGDIR" "$XDG_CACHE_HOME"

FILTER_PATTERN="${1:-exp_S2A1_*}"

echo "[INFO] Running compare_results.py on CPU-only job $(hostname) with filter ${FILTER_PATTERN}"
workers="${EVAL_PARALLEL_WORKERS:-1}"
if ! [[ "$workers" =~ ^[0-9]+$ ]] || [ "$workers" -lt 1 ]; then
    echo "[ERROR] EVAL_PARALLEL_WORKERS must be a positive integer"
    exit 2
fi

total_cpus="${SLURM_CPUS_PER_TASK:-4}"
threads_per_worker="${EVAL_THREADS_PER_WORKER:-$(( total_cpus / workers ))}"
if [ "$threads_per_worker" -lt 1 ]; then
    threads_per_worker=1
fi

skip_feature_importance="${EVAL_SKIP_FEATURE_IMPORTANCE:-}"
if [ -z "$skip_feature_importance" ]; then
    skip_feature_importance=0
fi

ram_per_worker_gb="${EVAL_RAM_PER_WORKER_GB:-}"
if [ -z "$ram_per_worker_gb" ]; then
    if [ "$workers" -gt 1 ]; then
        ram_per_worker_gb=2
    else
        ram_per_worker_gb=5
    fi
fi

python_bin="/home/chanyoungko/.conda/envs/IIT/bin/python"
top_k="${EVAL_FULL_TOPK:-1}"
full_threads="${EVAL_FULL_THREADS:-4}"
full_ram_gb="${EVAL_FULL_RAM_GB:-5}"
ablation_ram_gb="${EVAL_ABLATION_RAM_GB:-4}"
tmp_dir="/tmp/misalign-cpu-eval-${SLURM_JOB_ID:-manual}"
mkdir -p "$tmp_dir"
merged_summary="${tmp_dir}/automated_eval_metrics_summary.csv"

echo "[INFO] Stage 1/3: fast metrics-only eval for all experiments"

if [ "$workers" -eq 1 ]; then
    COMPARE_RESULTS_THREADS="$threads_per_worker" \
    COMPARE_RESULTS_RAM_GB="$ram_per_worker_gb" \
    "$python_bin" -u compare_results.py \
        --auto \
        --filter "${FILTER_PATTERN}" \
        --metrics-only \
        --summary-output "${merged_summary}" \
        --skip-ablation
    status=$?
    if [ "$status" -ne 0 ]; then
        exit "$status"
    fi
else
    echo "[INFO] Parallel fast CPU eval. workers=${workers}, threads_per_worker=${threads_per_worker}, ram_limit_gb=${ram_per_worker_gb}"

    pids=()
    shard_outputs=()
    for (( shard_index=0; shard_index<workers; shard_index++ )); do
        shard_csv="${tmp_dir}/metrics_shard_${shard_index}.csv"
        shard_outputs+=("$shard_csv")
        (
            echo "[WORKER ${shard_index}] Starting fast shard $(( shard_index + 1 ))/${workers}"
            COMPARE_RESULTS_THREADS="$threads_per_worker" \
            COMPARE_RESULTS_RAM_GB="$ram_per_worker_gb" \
            "$python_bin" -u compare_results.py \
                --auto \
                --filter "${FILTER_PATTERN}" \
                --num-shards "$workers" \
                --shard-index "$shard_index" \
                --metrics-only \
                --summary-output "$shard_csv" \
                --skip-ablation
        ) &
        pids+=("$!")
    done

    failed=0
    for pid in "${pids[@]}"; do
        if ! wait "$pid"; then
            failed=1
        fi
    done

    if [ "$failed" -ne 0 ]; then
        echo "[ERROR] One or more fast eval workers failed."
        exit 1
    fi

    export MERGED_SUMMARY_PATH="${merged_summary}"
    export SHARD_OUTPUTS="$(IFS=:; echo "${shard_outputs[*]}")"
    "$python_bin" - <<'PY'
import os
import pandas as pd
from pathlib import Path

target = Path(os.environ["MERGED_SUMMARY_PATH"])
paths = [Path(p) for p in os.environ["SHARD_OUTPUTS"].split(":") if p]
dfs = []
for path in paths:
    if path.exists():
        df = pd.read_csv(path)
        if not df.empty:
            dfs.append(df)
if not dfs:
    raise SystemExit("No shard metrics summaries found.")
merged = pd.concat(dfs, ignore_index=True).sort_values(["mae", "rmse", "name"], ascending=[True, True, True])
merged.to_csv(target, index=False)
print(f"[INFO] Merged metrics summary saved to {target}")
PY
fi

if [ ! -f "$merged_summary" ]; then
    echo "[ERROR] Missing merged metrics summary: $merged_summary"
    exit 1
fi

export MERGED_SUMMARY_PATH="${merged_summary}"
export EVAL_FULL_TOPK="${top_k}"
persistent_summary="$("$python_bin" - <<'PY'
import os
from pathlib import Path
import pandas as pd

csv_path = Path(os.environ["MERGED_SUMMARY_PATH"])
df = pd.read_csv(csv_path)
out = Path("compare_result") / "automated_eval_metrics_summary.csv"
if not df.empty and "path" in df.columns:
    paths = [Path(p) for p in df["path"].dropna().astype(str).tolist()]
    if paths:
        try:
            rel_paths = [p.relative_to(Path("experiments")) for p in paths]
            common_parent = Path(os.path.commonpath([str(p.parent) for p in rel_paths]))
            out = Path("compare_result") / common_parent / "automated_eval_metrics_summary.csv"
        except Exception:
            pass
print(out)
PY
)"
mkdir -p "$(dirname "$persistent_summary")"
cp "$merged_summary" "$persistent_summary"
merged_summary="$persistent_summary"
export MERGED_SUMMARY_PATH="${merged_summary}"
top_patterns="$("$python_bin" - <<'PY'
import os
import pandas as pd

csv_path = os.environ["MERGED_SUMMARY_PATH"]
top_k = max(0, int(os.environ["EVAL_FULL_TOPK"]))
df = pd.read_csv(csv_path)
if df.empty or top_k == 0:
    print("")
else:
    top = df.sort_values(["mae", "rmse", "name"], ascending=[True, True, True]).head(top_k)
    print(",".join(top["name"].astype(str).tolist()))
PY
)"

if [ -n "$top_patterns" ]; then
    echo "[INFO] Stage 2/3: full plot eval for best-${top_k}: ${top_patterns}"
    extra_args=()
    if [ "$skip_feature_importance" = "1" ]; then
        extra_args+=(--skip-feature-importance)
    fi
    COMPARE_RESULTS_THREADS="$full_threads" \
    COMPARE_RESULTS_RAM_GB="$full_ram_gb" \
    "$python_bin" -u compare_results.py \
        --auto \
        --filter "${top_patterns}" \
        --skip-ablation \
        "${extra_args[@]}"
else
    echo "[INFO] Stage 2/3: skipping full plot eval because no best-model candidate was found."
fi

echo "[INFO] Stage 3/3: ablation summary from fast metrics"
COMPARE_RESULTS_THREADS=1 \
COMPARE_RESULTS_RAM_GB="$ablation_ram_gb" \
"$python_bin" -u compare_results.py --ablation --metrics-summary-csv "$merged_summary"
