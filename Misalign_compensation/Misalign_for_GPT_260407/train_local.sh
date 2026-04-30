#!/bin/bash
# ==============================================================
# LOCAL TRAINING SCRIPT - Misalign_compensation
# ==============================================================

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
        result=$(whiptail --title "Misalign_compensation Config Selector" \
            --checklist "로컬에서 학습할 config를 선택하세요.\nSPACE: 선택/해제, ENTER: 확인" \
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
        # Fallback
        echo "========================================================"
        echo "  Misalign_compensation Config Selector"
        echo "========================================================"
        echo "번호를 공백으로 구분하여 입력하세요. 'all' = 전체 선택"
        local i=1
        for cfg in "${configs_list[@]}"; do
            printf "  %2d) %s\n" "$i" "$(basename "$cfg" .yaml)"
            ((i++))
        done
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

# ----- GPU Selection -----
select_gpu() {
    if command -v whiptail &>/dev/null; then
        GPU_ID=$(whiptail --title "GPU Selection" \
            --inputbox "사용할 GPU ID를 입력하세요 (예: 0, 1 등)\n현재 로컬 시스템 타겟입니다." \
            10 60 "0" \
            3>&1 1>&2 2>&3) || { echo "취소되었습니다."; exit 0; }
    else
        read -rp "사용할 GPU ID를 입력하세요 (기본: 0): " GPU_ID
        GPU_ID=${GPU_ID:-0}
    fi
}

# ----- Execution -----
execute_local() {
    echo ""
    echo "========================================================"
    echo "  Local Execution Summary"
    echo "========================================================"
    echo "  선택된 config : ${#SELECTED_CONFIGS[@]}개"
    echo "  할당된 GPU ID : ${GPU_ID}"
    echo "--------------------------------------------------------"
    for cfg in "${SELECTED_CONFIGS[@]}"; do
        echo "    - $(basename "$cfg" .yaml)"
    done
    echo "========================================================"

    if command -v whiptail &>/dev/null; then
        whiptail --title "학습 시작 확인" \
            --yesno "로컬 환경에서 순차적으로 학습을 시작하시겠습니까?" \
            10 60 || { echo "취소되었습니다."; exit 0; }
    else
        read -rp "시작하시겠습니까? (y/n): " confirm
        [[ "$confirm" != "y" && "$confirm" != "Y" ]] && { echo "취소되었습니다."; exit 0; }
    fi

    echo "================ ENVIRONMENT SETUP ================="
    # Source local conda path optimally for Git Bash on Windows
    source "C:/Users/ChanyoungKo/anaconda3/etc/profile.d/conda.sh" 2>/dev/null || true
    conda activate IIT 2>/dev/null || true

    export CUDA_VISIBLE_DEVICES="$GPU_ID"
    export PYTHONUNBUFFERED=1

    for cfg in "${SELECTED_CONFIGS[@]}"; do
        cfg_name=$(basename "$cfg" .yaml)
        log_file="logs/${cfg_name}.log"
        echo ""
        echo ">>> [RUNNING] $cfg_name (Log: $log_file)"
        # Use python directly
        python model_training.py --config "$cfg" 2>&1 | tee "$log_file"
        
        # Check if the process failed
        if [ ${PIPESTATUS[0]} -ne 0 ]; then
            echo ">>> [ERROR] $cfg_name 학습 중 오류 발생!"
            # Optional: break on error
            # break
        fi
    done

    echo ""
    echo ">>> [FINISHED] 모든 학습 루프가 종료되었습니다."
    echo ">>> [EVAL] 결과를 비교합니다..."
    python compare_results.py --auto
}

select_configs
select_gpu
execute_local
