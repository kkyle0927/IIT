#!/bin/bash
#SBATCH -J speedest_cpu_eval
#SBATCH -N 1
#SBATCH -n 1
#SBATCH --cpus-per-task=4
#SBATCH --mem=5G
#SBATCH -t 12:00:00
#SBATCH -o logs/%x_%j.out
#SBATCH -e logs/%x_%j.err

export OMP_NUM_THREADS=4
export MKL_NUM_THREADS=4
export NUMEXPR_NUM_THREADS=4
export MALLOC_ARENA_MAX=2
export CUDA_VISIBLE_DEVICES=""
export COMPARE_RESULTS_CUDA_VISIBLE_DEVICES=""
export MPLCONFIGDIR="/tmp/mpl-cpu-${SLURM_JOB_ID:-manual}"
export XDG_CACHE_HOME="/tmp/xdg-cache-cpu-${SLURM_JOB_ID:-manual}"
mkdir -p "$MPLCONFIGDIR" "$XDG_CACHE_HOME"

echo "[INFO] Running CPU-only compare_results.py on $(hostname)"
echo "[INFO] CUDA_VISIBLE_DEVICES='${CUDA_VISIBLE_DEVICES}'"
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

total_cpus="${SLURM_CPUS_PER_TASK:-4}"
threads_per_worker="${EVAL_THREADS_PER_WORKER:-$(( total_cpus / workers ))}"
if [ "$threads_per_worker" -lt 1 ]; then
    threads_per_worker=1
fi

skip_feature_importance="${EVAL_SKIP_FEATURE_IMPORTANCE:-1}"
fast_eval="${EVAL_FAST_EVAL:-1}"
full_plot_topk="${EVAL_FULL_PLOT_TOPK:-1}"
fast_eval_stride="${EVAL_FAST_EVAL_STRIDE:-3}"
eval_batch_size="${EVAL_BATCH_SIZE:-128}"
detail_batch_size="${EVAL_DETAIL_BATCH_SIZE:-64}"
fi_batch_size="${EVAL_FI_BATCH_SIZE:-64}"

ram_per_worker_gb="${EVAL_RAM_PER_WORKER_GB:-}"
if [ -z "$ram_per_worker_gb" ]; then
    if [ "$workers" -gt 1 ]; then
        ram_per_worker_gb=2
    else
        ram_per_worker_gb=5
    fi
fi

summary_ram_gb="${EVAL_SUMMARY_RAM_GB:-4}"
python_bin="/home/chanyoungko/.conda/envs/IIT/bin/python"
shared_args=("$@")

if [ "$workers" -eq 1 ]; then
    extra_args=()
    if [ "$fast_eval" = "1" ]; then
        extra_args+=(--fast-eval)
        extra_args+=(--fast-eval-stride "$fast_eval_stride")
        if [ "$full_plot_topk" -gt 0 ]; then
            extra_args+=(--full-plot-topk "$full_plot_topk")
        fi
    fi
    if [ "$skip_feature_importance" = "1" ]; then
        extra_args+=(--skip-feature-importance)
    fi
    echo "[INFO] Single-worker CPU eval. threads=${threads_per_worker}, ram_limit_gb=${ram_per_worker_gb}, skip_fi=${skip_feature_importance}, fast_eval=${fast_eval}, fast_stride=${fast_eval_stride}, topk=${full_plot_topk}, eval_bs=${eval_batch_size}, detail_bs=${detail_batch_size}, fi_bs=${fi_batch_size}"
    COMPARE_RESULTS_THREADS="$threads_per_worker" \
    COMPARE_RESULTS_RAM_GB="$ram_per_worker_gb" \
    COMPARE_RESULTS_EVAL_BATCH_SIZE="$eval_batch_size" \
    COMPARE_RESULTS_DETAIL_BATCH_SIZE="$detail_batch_size" \
    COMPARE_RESULTS_FI_BATCH_SIZE="$fi_batch_size" \
    "$python_bin" -u compare_results.py "${shared_args[@]}" "${extra_args[@]}"
    exit $?
fi

echo "[INFO] Parallel CPU eval. workers=${workers}, threads_per_worker=${threads_per_worker}, ram_limit_gb=${ram_per_worker_gb}, skip_fi=${skip_feature_importance}, fast_eval=${fast_eval}, fast_stride=${fast_eval_stride}, topk=${full_plot_topk}, eval_bs=${eval_batch_size}, detail_bs=${detail_batch_size}, fi_bs=${fi_batch_size}"

pids=()
for (( shard_index=0; shard_index<workers; shard_index++ )); do
    worker_args=("${shared_args[@]}" --num-shards "$workers" --shard-index "$shard_index" --skip-ablation)
    if [ "$fast_eval" = "1" ]; then
        worker_args+=(--fast-eval)
        worker_args+=(--fast-eval-stride "$fast_eval_stride")
    fi
    if [ "$skip_feature_importance" = "1" ]; then
        worker_args+=(--skip-feature-importance)
    fi
    (
        echo "[WORKER ${shard_index}] Starting shard $(( shard_index + 1 ))/${workers}"
        COMPARE_RESULTS_THREADS="$threads_per_worker" \
        COMPARE_RESULTS_RAM_GB="$ram_per_worker_gb" \
        COMPARE_RESULTS_EVAL_BATCH_SIZE="$eval_batch_size" \
        COMPARE_RESULTS_DETAIL_BATCH_SIZE="$detail_batch_size" \
        COMPARE_RESULTS_FI_BATCH_SIZE="$fi_batch_size" \
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
    echo "[ERROR] One or more CPU eval workers failed."
    exit 1
fi

if [ "$fast_eval" = "1" ] && [ "$full_plot_topk" -gt 0 ]; then
    echo "[INFO] Fast CPU eval shards finished. Rendering full plots for top ${full_plot_topk} experiments."
    topk_args=(--render-topk-only --full-plot-topk "$full_plot_topk")
    if [ "$skip_feature_importance" = "1" ]; then
        topk_args+=(--skip-feature-importance)
    fi
    COMPARE_RESULTS_THREADS="${SLURM_CPUS_PER_TASK:-4}" \
    COMPARE_RESULTS_RAM_GB=5 \
    COMPARE_RESULTS_EVAL_BATCH_SIZE="$eval_batch_size" \
    COMPARE_RESULTS_DETAIL_BATCH_SIZE="$detail_batch_size" \
    COMPARE_RESULTS_FI_BATCH_SIZE="$fi_batch_size" \
    "$python_bin" -u compare_results.py "${shared_args[@]}" "${topk_args[@]}"
fi

echo "[INFO] All CPU worker shards finished. Running final ablation summary."
COMPARE_RESULTS_THREADS=1 \
COMPARE_RESULTS_RAM_GB="$summary_ram_gb" \
"$python_bin" -u compare_results.py "${shared_args[@]}" --ablation
