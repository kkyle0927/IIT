#!/bin/bash
# ==============================================================
# Server Status Quick Check - SpeedEstimation
# Usage: ./status.sh          (터미널 요약)
#        ./status.sh --web    (웹 대시보드 열기)
#        ./status.sh --gpu    (GPU만 보기)
#        ./status.sh --jobs   (SLURM 잡만 보기)
# ==============================================================

RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
CYAN='\033[0;36m'
BOLD='\033[1m'
NC='\033[0m'

DASHBOARD_URL="http://143.248.65.114:5050"
PROJECT_NAME="SpeedEstimation"

show_header() {
    echo ""
    echo -e "${BOLD}========================================${NC}"
    echo -e "${BOLD}  Server Status — ${CYAN}${PROJECT_NAME}${NC}"
    echo -e "${BOLD}  $(date '+%Y-%m-%d %H:%M:%S')${NC}"
    echo -e "${BOLD}========================================${NC}"
}

show_gpu() {
    echo ""
    echo -e "${YELLOW}[GPU Status]${NC}"
    nvidia-smi --query-gpu=index,name,memory.used,memory.total,utilization.gpu,temperature.gpu \
        --format=csv,noheader,nounits 2>/dev/null | while IFS=',' read -r idx name mem_used mem_total util temp; do
        idx=$(echo "$idx" | xargs)
        name=$(echo "$name" | xargs)
        mem_used=$(echo "$mem_used" | xargs)
        mem_total=$(echo "$mem_total" | xargs)
        util=$(echo "$util" | xargs)
        temp=$(echo "$temp" | xargs)
        pct=$((mem_used * 100 / mem_total))
        if [ "$pct" -lt 10 ]; then
            color=$GREEN
            state="Free"
        else
            color=$RED
            state="In Use"
        fi
        printf "  GPU %s: ${color}%-7s${NC}  %5s/%5s MB (%2d%%)  Util: %3s%%  Temp: %s°C\n" \
            "$idx" "$state" "$mem_used" "$mem_total" "$pct" "$util" "$temp"
    done

    # GPU 사용자 정보
    echo ""
    echo -e "${YELLOW}[GPU Users]${NC}"
    nvidia-smi --query-compute-apps=pid,gpu_bus_id,used_memory --format=csv,noheader 2>/dev/null | while IFS=',' read -r pid gpu mem; do
        pid=$(echo "$pid" | xargs)
        gpu=$(echo "$gpu" | xargs)
        mem=$(echo "$mem" | xargs)
        user=$(ps -o user= -p "$pid" 2>/dev/null | xargs)
        cmd=$(ps -o args= -p "$pid" 2>/dev/null | head -c 60)
        if [ -n "$user" ]; then
            printf "  PID %-7s  User: %-12s  GPU: %s  Mem: %s\n" "$pid" "$user" "$gpu" "$mem"
        fi
    done
    gpu_proc_count=$(nvidia-smi --query-compute-apps=pid --format=csv,noheader 2>/dev/null | wc -l)
    if [ "$gpu_proc_count" -eq 0 ]; then
        echo -e "  ${GREEN}No GPU processes running${NC}"
    fi
}

show_system() {
    echo ""
    echo -e "${YELLOW}[System Resources]${NC}"

    # CPU
    cpu_usage=$(top -bn1 | grep "Cpu(s)" | awk '{print $2}' 2>/dev/null)
    load=$(uptime | awk -F'load average:' '{print $2}' | xargs)
    printf "  CPU Usage: %s%%  Load Avg: %s\n" "$cpu_usage" "$load"

    # RAM
    read -r total used free <<< $(free -g | awk '/^Mem:/{print $2, $3, $4}')
    ram_pct=$((used * 100 / total))
    if [ "$ram_pct" -gt 80 ]; then
        ram_color=$RED
    elif [ "$ram_pct" -gt 50 ]; then
        ram_color=$YELLOW
    else
        ram_color=$GREEN
    fi
    printf "  RAM: ${ram_color}%dG / %dG (%d%%)${NC}  Free: %dG\n" "$used" "$total" "$ram_pct" "$free"

    # Disk
    disk_info=$(df -h /home 2>/dev/null | awk 'NR==2{printf "%s / %s (%s)", $3, $2, $5}')
    printf "  Disk (/home): %s\n" "$disk_info"
}

show_jobs() {
    echo ""
    echo -e "${YELLOW}[SLURM Jobs]${NC}"

    # My jobs
    my_jobs=$(squeue -u "$USER" -o "%.10i %.20j %.8T %.10M %.20R" --noheader 2>/dev/null)
    if [ -n "$my_jobs" ]; then
        echo -e "  ${CYAN}My Jobs:${NC}"
        echo "  $(squeue -u "$USER" -o "%.10i %.20j %.8T %.10M" --noheader 2>/dev/null | sed 's/^/  /')"
    else
        echo -e "  ${GREEN}No running jobs for $USER${NC}"
    fi

    # All jobs
    echo ""
    echo -e "  ${CYAN}All Jobs:${NC}"
    all_jobs=$(squeue -o "%.10i %.10u %.20j %.8T %.10M %.20R" --noheader 2>/dev/null)
    if [ -n "$all_jobs" ]; then
        squeue -o "%.10i %.10u %.20j %.8T %.10M %.20R" --noheader 2>/dev/null | sed 's/^/  /'
    else
        echo -e "  ${GREEN}No jobs in queue${NC}"
    fi
}

show_project() {
    echo ""
    echo -e "${YELLOW}[Project: ${PROJECT_NAME}]${NC}"

    # Running experiments
    running_logs=$(find logs/ -name "*.log" -newer logs/ -mmin -60 2>/dev/null | head -5)
    if [ -n "$running_logs" ]; then
        echo -e "  ${CYAN}Recent logs (last 1h):${NC}"
        for log in $running_logs; do
            size=$(du -h "$log" 2>/dev/null | cut -f1)
            mod=$(stat -c '%Y' "$log" 2>/dev/null)
            now=$(date +%s)
            ago=$(( (now - mod) / 60 ))
            printf "    %-50s  %s  (%d min ago)\n" "$log" "$size" "$ago"
        done
    fi

    # Pending configs
    pending=$(find configs/ -name "*.yaml" -not -path "*/archive/*" 2>/dev/null | wc -l)
    completed=$(ls -d experiments/*/ 2>/dev/null | wc -l)
    printf "  Configs: %d  |  Completed experiments: %d\n" "$pending" "$completed"
}

# ==================== Main ====================
case "${1:-}" in
    --web)
        echo "Opening dashboard: $DASHBOARD_URL"
        if command -v xdg-open &>/dev/null; then
            xdg-open "$DASHBOARD_URL" 2>/dev/null
        elif command -v curl &>/dev/null; then
            echo "No browser available. Fetching with curl..."
            curl -s --connect-timeout 3 "$DASHBOARD_URL" | python3 -c "
import sys, re
html = sys.stdin.read()
text = re.sub('<[^>]+>', ' ', html)
text = re.sub(r'\s+', ' ', text).strip()
print(text[:2000])
" 2>/dev/null || echo "Dashboard unreachable."
        fi
        ;;
    --gpu)
        show_gpu
        ;;
    --jobs)
        show_jobs
        ;;
    *)
        show_header
        show_gpu
        show_system
        show_jobs
        show_project
        echo ""
        echo -e "${BOLD}Dashboard: ${CYAN}${DASHBOARD_URL}${NC}"
        echo ""
        ;;
esac
