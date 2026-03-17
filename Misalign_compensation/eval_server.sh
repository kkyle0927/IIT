#!/bin/bash
#SBATCH -J misalign_eval
#SBATCH -N 1
#SBATCH -n 1
#SBATCH --gres=gpu:idx0:1
#SBATCH --cpus-per-task=8
#SBATCH --mem=15G
#SBATCH -t 12:00:00
#SBATCH -o logs/%x_%j.out
#SBATCH -e logs/%x_%j.err

export OMP_NUM_THREADS=4
export MALLOC_ARENA_MAX=2
export PYTORCH_CUDA_ALLOC_CONF=max_split_size_mb:128

FILTER_PATTERN="${1:-exp_S2A1_*}"

echo "[INFO] Running compare_results.py on $(hostname) with filter ${FILTER_PATTERN}"
/home/chanyoungko/.conda/envs/IIT/bin/python -u compare_results.py --auto --filter "${FILTER_PATTERN}"
