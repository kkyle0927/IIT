#!/bin/bash
#SBATCH -J speedest_eval
#SBATCH -N 1
#SBATCH -n 1
#SBATCH -p idx1
#SBATCH --gres=gpu:idx1:1
#SBATCH --cpus-per-task=8
#SBATCH --mem=15G
#SBATCH -t 12:00:00
#SBATCH -o logs/%x_%j.out
#SBATCH -e logs/%x_%j.err

export OMP_NUM_THREADS=4
export MALLOC_ARENA_MAX=2
export PYTORCH_CUDA_ALLOC_CONF=max_split_size_mb:128
export MPLCONFIGDIR="/tmp/mpl-${SLURM_JOB_ID:-manual}"
export XDG_CACHE_HOME="/tmp/xdg-cache-${SLURM_JOB_ID:-manual}"
mkdir -p "$MPLCONFIGDIR" "$XDG_CACHE_HOME"

echo "[INFO] Running compare_results.py on $(hostname) with GPU 1"
echo "[INFO] Arguments: $@"
workers="${EVAL_PARALLEL_WORKERS:-1}"
if ! [[ "$workers" =~ ^[0-9]+$ ]] || [ "$workers" -lt 1 ]; then
    echo "[ERROR] EVAL_PARALLEL_WORKERS must be a positive integer"
    exit 2
fi

if [ "$workers" -gt 1 ] && [[ " $* " != *" --auto "* ]]; then
    echo "[ERROR] Parallel workers require --auto mode."
    exit 2
fi

total_cpus="${SLURM_CPUS_PER_TASK:-8}"
threads_per_worker="${EVAL_THREADS_PER_WORKER:-$(( total_cpus / workers ))}"
if [ "$threads_per_worker" -lt 1 ]; then
    threads_per_worker=1
fi

skip_feature_importance="${EVAL_SKIP_FEATURE_IMPORTANCE:-}"
if [ -z "$skip_feature_importance" ]; then
    if [ "$workers" -gt 1 ]; then
        skip_feature_importance=1
    else
        skip_feature_importance=0
    fi
fi

ram_per_worker_gb="${EVAL_RAM_PER_WORKER_GB:-}"
if [ -z "$ram_per_worker_gb" ]; then
    if [ "$workers" -gt 1 ]; then
        ram_per_worker_gb=$(( 12 / workers ))
        if [ "$ram_per_worker_gb" -lt 4 ]; then
            ram_per_worker_gb=4
        fi
    else
        ram_per_worker_gb=15
    fi
fi

python_bin="/home/chanyoungko/.conda/envs/IIT/bin/python"
shared_args=("$@")
skip_trajectory_plots="${EVAL_SKIP_TRAJECTORY_PLOTS:-0}"
skip_detailed_plots="${EVAL_SKIP_DETAILED_PLOTS:-0}"
eval_stride="${EVAL_STRIDE:-1}"

if [ "$workers" -eq 1 ]; then
    extra_args=()
    if [ "$skip_feature_importance" = "1" ]; then
        extra_args+=(--skip-feature-importance)
    fi
    if [ "$skip_trajectory_plots" = "1" ]; then
        extra_args+=(--skip-trajectory-plots)
    fi
    if [ "$skip_detailed_plots" = "1" ]; then
        extra_args+=(--skip-detailed-plots)
    fi
    if [ "$eval_stride" != "1" ]; then
        extra_args+=(--eval-stride "$eval_stride")
    fi
    echo "[INFO] Single-worker eval. threads=${threads_per_worker}, ram_limit_gb=${ram_per_worker_gb}, skip_fi=${skip_feature_importance}"
    COMPARE_RESULTS_THREADS="$threads_per_worker" \
    COMPARE_RESULTS_RAM_GB="$ram_per_worker_gb" \
    "$python_bin" -u compare_results.py "${shared_args[@]}" "${extra_args[@]}"
    exit $?
fi

echo "[INFO] Parallel eval on one GPU. workers=${workers}, threads_per_worker=${threads_per_worker}, ram_limit_gb=${ram_per_worker_gb}, skip_fi=${skip_feature_importance}"

pids=()
for (( shard_index=0; shard_index<workers; shard_index++ )); do
    worker_args=("${shared_args[@]}" --num-shards "$workers" --shard-index "$shard_index" --skip-ablation)
    if [ "$skip_feature_importance" = "1" ]; then
        worker_args+=(--skip-feature-importance)
    fi
    if [ "$skip_trajectory_plots" = "1" ]; then
        worker_args+=(--skip-trajectory-plots)
    fi
    if [ "$skip_detailed_plots" = "1" ]; then
        worker_args+=(--skip-detailed-plots)
    fi
    if [ "$eval_stride" != "1" ]; then
        worker_args+=(--eval-stride "$eval_stride")
    fi
    (
        echo "[WORKER ${shard_index}] Starting shard $(( shard_index + 1 ))/${workers}"
        COMPARE_RESULTS_THREADS="$threads_per_worker" \
        COMPARE_RESULTS_RAM_GB="$ram_per_worker_gb" \
        "$python_bin" -u compare_results.py "${worker_args[@]}"
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
    echo "[ERROR] One or more eval workers failed."
    exit 1
fi

echo "[INFO] All worker shards finished. Running final ablation summary."
summary_args=("${shared_args[@]}" --ablation)
if [ "$skip_trajectory_plots" = "1" ]; then
    summary_args+=(--skip-trajectory-plots)
fi
if [ "$skip_detailed_plots" = "1" ]; then
    summary_args+=(--skip-detailed-plots)
fi
if [ "$eval_stride" != "1" ]; then
    summary_args+=(--eval-stride "$eval_stride")
fi
COMPARE_RESULTS_THREADS=1 \
COMPARE_RESULTS_RAM_GB=6 \
"$python_bin" -u compare_results.py "${summary_args[@]}"
