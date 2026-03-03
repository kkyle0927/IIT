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
#SBATCH --gres=gpu:1
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

        echo ""
        echo "========================================================"
        echo "  Submission Summary"
        echo "========================================================"
        echo "  선택된 config : ${num_selected}개"
        echo "  동시 실행 GPU : ${MAX_PARALLEL}개 (각 job: 1 GPU, 15G, 8 CPUs)"
        echo "  총 SLURM 잡   : ${num_selected}개"
        echo "--------------------------------------------------------"
        for cfg in "${SELECTED_CONFIGS[@]}"; do
            echo "    - $(basename "$cfg" .yaml)"
        done
        echo "========================================================"

        if command -v whiptail &>/dev/null; then
            whiptail --title "제출 확인" \
                --yesno "${num_selected}개 잡을 제출하시겠습니까?\n\n동시 GPU: ${MAX_PARALLEL} | 각 job: 1 GPU, 15G RAM, 8 CPUs" \
                10 60 || { echo "취소되었습니다."; exit 0; }
        else
            read -rp "제출하시겠습니까? (y/n): " confirm
            [[ "$confirm" != "y" && "$confirm" != "Y" ]] && { echo "취소되었습니다."; exit 0; }
        fi

        # 첫 MAX_PARALLEL개 job은 즉시 제출, 나머지는 이전 job에 의존하여 순차 대기
        local submitted_ids=()
        local slot=0

        for cfg in "${SELECTED_CONFIGS[@]}"; do
            local job_name
            job_name="train_$(basename "$cfg" .yaml)"

            local dep_arg=""
            if [ "$slot" -ge "$MAX_PARALLEL" ]; then
                # slot번째 이전 job이 끝나면 시작 (슬라이딩 윈도우 방식)
                local dep_idx=$((slot - MAX_PARALLEL))
                dep_arg="--dependency=afterany:${submitted_ids[$dep_idx]}"
            fi

            local job_id
            job_id=$(sbatch --parsable \
                --job-name="$job_name" \
                --gres="gpu:1" \
                --mem="15G" \
                --cpus-per-task="8" \
                $dep_arg \
                "$0" "$cfg")
            submitted_ids+=("$job_id")
            echo "  Submitted: $job_name (JobID: $job_id)"
            ((slot++))
        done

        echo ""
        echo "총 ${num_selected}개 잡 제출 완료 (동시 최대 ${MAX_PARALLEL}개)."
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
CONFIG_PATH="${1:?ERROR: config 경로가 지정되지 않았습니다. 사용법: sbatch train.sh <config.yaml>}"

source /opt/miniconda3/etc/profile.d/conda.sh
conda activate IIT

echo "Job started on $(hostname) at $(date)"
echo "Config: $CONFIG_PATH"
echo "CUDA_VISIBLE_DEVICES=$CUDA_VISIBLE_DEVICES"
nvidia-smi -L

export PYTHONUNBUFFERED=1

python SpeedEstimator_TCN_MLP_experiments.py --config "$CONFIG_PATH"

echo "Job finished at $(date)"

# ================== USAGE ==================
# Interactive:  ./train.sh          (TUI로 config 선택 후 sbatch 자동 제출)
# Direct:       sbatch train.sh configs/baseline.yaml
