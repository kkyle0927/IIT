#!/bin/bash
# ==============================================================
# SLURM JOB SCRIPT — SpeedEstimation
# --------------------------------------------------------------
# [Server Information]
#   NodeName    : server1
#   Total CPUs  : 64 cores
#   Total Memory: 62000 MB  (~62 GB usable)
#   Total GPUs  : 4 × NVIDIA RTX A5000
# --------------------------------------------------------------
# [Resource limits reminder]
#   - You must request resources within the server limits:
#       --cpus-per-task <= 64
#       --mem <= 62000M (or 62G)
#       --gres=gpu: <= 4
#   - Exceeding these values will cause "Requested node configuration is not available" error.
# ==============================================================

# =========================================================================================
# [CRITICAL NOTICE - 수정 금지 / 변경 시 확인 필수]
# * 이 파일은 SLURM 학습 제출 스크립트입니다.
# * 어떤 이유로든 이 파일(train.sh)의 내용을 수정해야 한다면, 반드시 수정하기 전에 사용자에게
#   한국어(Korean)로 변경하려는 내용과 이유를 설명하고 명시적인 허락을 받아야 합니다.
# * 사용자가 "수정해"라고 말하더라도, 이 주석을 상기하며 "정말로 수정하시겠습니까?
#   이 주석에 따르면 변경 전 재확인이 필요합니다."라고 한 번 더 되물어 확인받으십시오.
# * 안전 장치를 무시하고 임의로 변경하지 마십시오.
# =========================================================================================

# =========================================================================================
# [GPU 독점 방지 정책 - MUST FOLLOW]
# * chanyoung(현재 사용자)가 시스템의 전체 GPU(4개)를 모두 독점하면 안 됩니다.
# * 다른 사용자가 GPU를 사용하지 않더라도 최대 3개까지만 사용합니다.
# * 이 정책은 TUI에서 config 다중 선택 + GPU 할당 시 스스로 준수해야 합니다.
# =========================================================================================

#SBATCH -J speed_train
#SBATCH -N 1
#SBATCH -n 1
#SBATCH --gres=gpu:idx0:1
#SBATCH --cpus-per-task=8
#SBATCH --mem=15G
#SBATCH -t 12:00:00
#SBATCH -o logs/%x_%j.out
#SBATCH -e logs/%x_%j.err


# =========================================================================================
# [INTERACTIVE CONFIG SELECTOR]
# SLURM 잡 ID가 없으면(=터미널에서 직접 실행) TUI를 표시하고 sbatch로 제출합니다.
# =========================================================================================
if [ -z "$SLURM_JOB_ID" ]; then
    mkdir -p logs

    make_archive_job_name() {
        local mode="$1"
        local sequence="$2"
        local cfg_path="$3"
        if [[ "$cfg_path" =~ configs/archive_season([0-9]+)/archive_([0-9]+)/ ]]; then
            local season="${BASH_REMATCH[1]}"
            local archive="${BASH_REMATCH[2]}"
            printf "SpeedRobot_AS%s_A%s_%s_%s" "$season" "$archive" "$mode" "$sequence"
            return 0
        fi
        return 1
    }

    # ----- Config Discovery -----
    discover_configs() {
        find configs/ -name '*.yaml' -type f 2>/dev/null | sort
    }

    # ----- Config Selection (whiptail checklist) -----
    select_configs() {
        local configs_list
        mapfile -t configs_list < <(discover_configs)

        if [ ${#configs_list[@]} -eq 0 ]; then
            echo "[ERROR] configs/ 디렉터리에 YAML 파일이 없습니다."
            exit 1
        fi

        if command -v whiptail &>/dev/null; then
            local checklist_args=()
            for cfg in "${configs_list[@]}"; do
                local display_name
                display_name=$(basename "$cfg" .yaml)
                if [[ "$display_name" == "baseline" ]]; then
                    checklist_args+=("$cfg" "$display_name" "ON")
                else
                    checklist_args+=("$cfg" "$display_name" "OFF")
                fi
            done

            local num_items=${#configs_list[@]}
            local list_height=$((num_items > 20 ? 20 : num_items))
            local box_height=$((list_height + 8))

            local result
            result=$(whiptail --title "SpeedEstimation Config Selector" \
                --checklist "학습할 config를 선택하세요.\nSPACE: 선택/해제, ENTER: 확인" \
                "$box_height" 78 "$list_height" \
                "${checklist_args[@]}" \
                3>&1 1>&2 2>&3) || { echo "취소되었습니다."; exit 0; }

            SELECTED_CONFIGS=()
            for item in $result; do
                item="${item%\"}"
                item="${item#\"}"
                [ -n "$item" ] && SELECTED_CONFIGS+=("$item")
            done
        else
            # Fallback: 텍스트 모드
            echo "========================================================"
            echo "  SpeedEstimation Config Selector"
            echo "========================================================"
            echo "번호를 공백으로 구분하여 입력하세요 (예: 1 3 5). 'all' = 전체 선택"
            echo ""
            local i=1
            for cfg in "${configs_list[@]}"; do
                printf "  %2d) %s\n" "$i" "$(basename "$cfg" .yaml)"
                ((i++))
            done
            echo ""
            read -rp "선택: " selection

            SELECTED_CONFIGS=()
            if [[ "$selection" == "all" ]]; then
                SELECTED_CONFIGS=("${configs_list[@]}")
            else
                for idx in $selection; do
                    if [[ "$idx" =~ ^[0-9]+$ ]] && [ "$idx" -ge 1 ] && [ "$idx" -le "${#configs_list[@]}" ]; then
                        SELECTED_CONFIGS+=("${configs_list[$((idx-1))]}")
                    fi
                done
            fi
        fi

        if [ ${#SELECTED_CONFIGS[@]} -eq 0 ]; then
            echo "[ERROR] 선택된 config가 없습니다."
            exit 1
        fi
    }

    # ----- Parallel GPU Count Selection -----
    select_gpu_count() {
        # ============================================================
        # [CRITICAL] MEMORY LIMIT = 15 GB per GPU
        # * 1 GPU당 15GB 메모리 제한은 시스템 안정성을 위한 핵심 조치입니다.
        # * 이 값을 변경하려면 사용자에게 반드시 확인을 받으십시오.
        # ============================================================
        # 각 job은 항상 1 GPU / 15G RAM / 8 CPUs 를 사용합니다.
        # 여기서 선택하는 GPU 수 = 동시에 실행할 job 수 (병렬 수)
        if command -v whiptail &>/dev/null; then
            MAX_PARALLEL=$(whiptail --title "GPU Selection" \
                --menu "동시에 사용할 GPU 수를 선택하세요.\n(각 job은 1 GPU 사용, 선택한 수만큼 병렬 실행)" \
                15 62 3 \
                "1" "1개 job 순차 실행" \
                "2" "2개 job 동시 실행  (권장)" \
                "3" "3개 job 동시 실행  (최대)" \
                3>&1 1>&2 2>&3) || { echo "취소되었습니다."; exit 0; }
        else
            while true; do
                read -rp "동시 실행 GPU 수 (1-3) [기본: 2]: " MAX_PARALLEL
                MAX_PARALLEL=${MAX_PARALLEL:-2}
                [[ "$MAX_PARALLEL" =~ ^[1-3]$ ]] && break
                echo "[ERROR] 1~3 사이의 숫자를 입력하세요."
            done
        fi
    }

    # ----- Confirmation & Submission -----
    confirm_and_submit() {
        local num_selected=${#SELECTED_CONFIGS[@]}
        local num_launchers=$MAX_PARALLEL
        if [ "$num_selected" -lt "$num_launchers" ]; then
            num_launchers=$num_selected
        fi

        echo ""
        echo "========================================================"
        echo "  Submission Summary"
        echo "========================================================"
        echo "  선택된 config : ${num_selected}개"
        echo "  동시 실행 GPU : ${MAX_PARALLEL}개"
        echo "  GPU 내 worker : 기본 3개, OOM 시 1~3 backoff"
        echo "  총 SLURM 잡   : ${num_launchers}개 (각 job: 1 GPU, 15G, 8 CPUs)"
        echo "--------------------------------------------------------"
        for cfg in "${SELECTED_CONFIGS[@]}"; do
            echo "    - $(basename "$cfg" .yaml)"
        done
        echo "========================================================"

        if command -v whiptail &>/dev/null; then
            whiptail --title "제출 확인" \
                --yesno "${num_launchers}개 launcher job을 제출하시겠습니까?\n\n동시 GPU: ${MAX_PARALLEL} | GPU 내 worker: 기본 3개, OOM 시 1~3 backoff" \
                10 60 || { echo "취소되었습니다."; exit 0; }
        else
            read -rp "제출하시겠습니까? (y/n): " confirm
            [[ "$confirm" != "y" && "$confirm" != "Y" ]] && { echo "취소되었습니다."; exit 0; }
        fi

        for ((slot=0; slot<num_launchers; slot++)); do
            local launcher_cfgs=()
            local idx
            for ((idx=slot; idx<num_selected; idx+=num_launchers)); do
                launcher_cfgs+=("${SELECTED_CONFIGS[$idx]}")
            done

            local job_name
            if ! job_name=$(make_archive_job_name "train" "$((slot+1))" "${launcher_cfgs[0]}"); then
                job_name="train_$(basename "${launcher_cfgs[0]}" .yaml)"
                if [ "${#launcher_cfgs[@]}" -gt 1 ]; then
                    job_name="${job_name}_pack$((slot+1))"
                fi
            fi

            local job_id
            job_id=$(sbatch --parsable \
                --job-name="$job_name" \
                --gres="gpu:1" \
                --mem="15G" \
                --cpus-per-task="8" \
                "$0" "${launcher_cfgs[@]}")
            echo "  Submitted: $job_name (JobID: $job_id, configs=${#launcher_cfgs[@]})"
        done

        echo ""
        echo "총 ${num_launchers}개 launcher job 제출 완료 (동시 GPU 최대 ${MAX_PARALLEL}개)."
        echo "확인: squeue -u \$USER"
    }

    # ----- Main TUI Flow -----
    select_configs
    select_gpu_count
    confirm_and_submit
    exit 0
fi


# =========================================================================================
# [SLURM JOB EXECUTION]
# 아래 코드는 SLURM에 의해 실행됩니다. (sbatch가 할당한 GPU/메모리 사용)
# =========================================================================================
CONFIG_PATHS=("$@")
if [ ${#CONFIG_PATHS[@]} -eq 0 ]; then
    echo "ERROR: config 경로가 지정되지 않았습니다. 사용법: sbatch train.sh <config1.yaml> [config2.yaml ...]"
    exit 1
fi

source /opt/miniconda3/etc/profile.d/conda.sh
conda activate IIT

export MPLCONFIGDIR="/tmp/mpl-${SLURM_JOB_ID:-manual}"
export XDG_CACHE_HOME="/tmp/xdg-cache-${SLURM_JOB_ID:-manual}"
mkdir -p "$MPLCONFIGDIR" "$XDG_CACHE_HOME"
mkdir -p logs

echo "Job started on $(hostname) at $(date)"
echo "Configs (${#CONFIG_PATHS[@]}): ${CONFIG_PATHS[*]}"
echo "CUDA_VISIBLE_DEVICES=$CUDA_VISIBLE_DEVICES"
nvidia-smi -L

export PYTHONUNBUFFERED=1

TRAIN_DEFAULT_WORKERS="${TRAIN_DEFAULT_WORKERS:-3}"
TRAIN_MIN_WORKERS="${TRAIN_MIN_WORKERS:-1}"
TRAIN_MAX_WORKERS="${TRAIN_MAX_WORKERS:-3}"
TRAIN_MAX_RETRIES="${TRAIN_MAX_RETRIES:-2}"
TRAIN_ADAPTIVE_WORKERS="${TRAIN_ADAPTIVE_WORKERS:-0}"
TRAIN_SCHEDULER_POLL_SEC="${TRAIN_SCHEDULER_POLL_SEC:-30}"
TRAIN_MIN_THREADS_PER_WORKER="${TRAIN_MIN_THREADS_PER_WORKER:-2}"

total_cpus="${SLURM_CPUS_PER_TASK:-8}"
total_configs="${#CONFIG_PATHS[@]}"
cpu_worker_cap=$(( total_cpus / TRAIN_MIN_THREADS_PER_WORKER ))
if [ "$cpu_worker_cap" -lt 1 ]; then
    cpu_worker_cap=1
fi
if [ "$TRAIN_MAX_WORKERS" -gt "$cpu_worker_cap" ]; then
    TRAIN_MAX_WORKERS="$cpu_worker_cap"
fi
if [ "$TRAIN_MAX_WORKERS" -gt "$total_configs" ]; then
    TRAIN_MAX_WORKERS="$total_configs"
fi
if [ "$TRAIN_MIN_WORKERS" -gt "$TRAIN_MAX_WORKERS" ]; then
    TRAIN_MIN_WORKERS="$TRAIN_MAX_WORKERS"
fi
default_target="$TRAIN_DEFAULT_WORKERS"
if [ "$default_target" -lt "$TRAIN_MIN_WORKERS" ]; then
    default_target="$TRAIN_MIN_WORKERS"
fi
if [ "$default_target" -gt "$TRAIN_MAX_WORKERS" ]; then
    default_target="$TRAIN_MAX_WORKERS"
fi
current_target="$default_target"

echo "[SCHED] OOM-backoff same-GPU workers enabled: default=${default_target}, current=${current_target}, range=${TRAIN_MIN_WORKERS}-${TRAIN_MAX_WORKERS}, retries=${TRAIN_MAX_RETRIES}, util_adaptive=${TRAIN_ADAPTIVE_WORKERS}"

declare -a cfg_queue=("${CONFIG_PATHS[@]}")
declare -a active_pids=()
declare -a active_cfgs=()
declare -a active_logs=()
declare -a active_ids=()
declare -A cfg_attempts=()
declare -a failed_cfgs=()
successful_cfgs=0
worker_seq=0

adjust_target_workers() {
    if [ "$TRAIN_ADAPTIVE_WORKERS" != "1" ]; then
        return
    fi
}

launch_worker() {
    local cfg="$1"
    local attempt="${cfg_attempts[$cfg]}"
    local threads=$(( total_cpus / current_target ))
    if [ "$threads" -lt 1 ]; then
        threads=1
    fi

    worker_seq=$((worker_seq + 1))
    local label
    label=$(basename "$cfg" .yaml | tr '/ ' '__')
    local worker_log="logs/${SLURM_JOB_NAME}_${SLURM_JOB_ID}_w${worker_seq}_${label}_a${attempt}.log"

    echo "[WORKER ${worker_seq}] Launching $(basename "$cfg") attempt=${attempt}/${TRAIN_MAX_RETRIES} threads=${threads} log=${worker_log}"
    (
        export PYTHONUNBUFFERED=1
        export OMP_NUM_THREADS="$threads"
        export MKL_NUM_THREADS="$threads"
        export NUMEXPR_NUM_THREADS="$threads"
        python SpeedEstimator_TCN_MLP_experiments.py --config "$cfg" >"$worker_log" 2>&1
    ) &

    active_pids+=("$!")
    active_cfgs+=("$cfg")
    active_logs+=("$worker_log")
    active_ids+=("$worker_seq")
}

handle_finished_workers() {
    local next_pids=()
    local next_cfgs=()
    local next_logs=()
    local next_ids=()

    for i in "${!active_pids[@]}"; do
        local pid="${active_pids[$i]}"
        local cfg="${active_cfgs[$i]}"
        local log="${active_logs[$i]}"
        local wid="${active_ids[$i]}"

        if kill -0 "$pid" 2>/dev/null; then
            next_pids+=("$pid")
            next_cfgs+=("$cfg")
            next_logs+=("$log")
            next_ids+=("$wid")
            continue
        fi

        wait "$pid"
        local status=$?
        if [ "$status" -eq 0 ]; then
            successful_cfgs=$((successful_cfgs + 1))
            echo "[WORKER ${wid}] Completed $(basename "$cfg") successfully. progress=${successful_cfgs}/${total_configs}"
            if [ "$current_target" -lt "$default_target" ]; then
                current_target="$default_target"
                echo "[SCHED] Success observed. Restoring target workers to ${current_target}."
            fi
            continue
        fi

        if grep -Eiq "CUDA out of memory|out of memory|DefaultCPUAllocator: can't allocate memory" "$log"; then
            echo "[WORKER ${wid}] Memory pressure detected for $(basename "$cfg") (status=${status})."
            if [ "$current_target" -gt "$TRAIN_MIN_WORKERS" ]; then
                current_target=$((current_target - 1))
                echo "[SCHED] Backing off target workers to ${current_target} after memory failure."
            fi
            if [ "${cfg_attempts[$cfg]}" -lt "$TRAIN_MAX_RETRIES" ]; then
                echo "[SCHED] Re-queueing $(basename "$cfg") for retry ${cfg_attempts[$cfg]}/${TRAIN_MAX_RETRIES}."
                cfg_queue+=("$cfg")
            else
                echo "[SCHED] Exhausted retries for $(basename "$cfg")."
                failed_cfgs+=("$cfg")
            fi
        else
            echo "[WORKER ${wid}] Failed $(basename "$cfg") with status=${status}. log=${log}"
            tail -n 20 "$log" || true
            failed_cfgs+=("$cfg")
        fi
    done

    active_pids=("${next_pids[@]}")
    active_cfgs=("${next_cfgs[@]}")
    active_logs=("${next_logs[@]}")
    active_ids=("${next_ids[@]}")
}

while [ ${#cfg_queue[@]} -gt 0 ] || [ ${#active_pids[@]} -gt 0 ]; do
    handle_finished_workers
    adjust_target_workers

    while [ ${#cfg_queue[@]} -gt 0 ] && [ ${#active_pids[@]} -lt "$current_target" ]; do
        cfg="${cfg_queue[0]}"
        cfg_queue=("${cfg_queue[@]:1}")
        cfg_attempts["$cfg"]=$(( ${cfg_attempts["$cfg"]:-0} + 1 ))
        launch_worker "$cfg"
    done

    if [ ${#active_pids[@]} -gt 0 ]; then
        sleep "$TRAIN_SCHEDULER_POLL_SEC"
    fi
done

if [ ${#failed_cfgs[@]} -gt 0 ]; then
    echo "[ERROR] ${#failed_cfgs[@]} config(s) failed:"
    for cfg in "${failed_cfgs[@]}"; do
        echo "  - $cfg"
    done
    exit 1
fi

echo "Job finished at $(date)"

# ================== USAGE ==================
# Interactive:  ./train.sh          (TUI로 config 선택 후 sbatch 자동 제출)
# Direct:       sbatch train.sh configs/a.yaml [configs/b.yaml ...]
