#!/bin/bash
# ==============================================================================
# ZAV Quadrotor SITL Split-Dashboard Runner (tmux 기반 2분할 대시보드)
# ==============================================================================
# 기능:
#   1. DDS Agent(ROS 2 브리지) 자동 기동
#   2. 상하 2분할 (상단: PX4 실시간 콘솔, 하단: Pegasus 파이썬 로그)
#   3. 마우스 휠 스크롤 지원 + 마우스 드래그 시 윈도우/리눅스 클립보드 자동 복사
#   4. 상단 또는 하단 창에서 Ctrl+C 입력 시 세션 전체(tmux) 즉시 폭파 및 단일 터미널 복귀
# ==============================================================================

export PATH="/mnt/c/Windows/System32:$PATH"
CMD_BIN="/mnt/c/Windows/System32/cmd.exe"
TASKKILL_BIN="/mnt/c/Windows/System32/taskkill.exe"
CLIP_BIN="/mnt/c/Windows/System32/clip.exe"

SESSION_NAME="zav_sim"
ISAAC_LOG="/tmp/zav_isaac_sim.log"
rm -f "$ISAAC_LOG"

# 1. 이전 잔여 프로세스 및 기존 tmux 세션 정리
tmux kill-session -t $SESSION_NAME 2>/dev/null || true
pkill -9 -f MicroXRCEAgent 2>/dev/null || true
pkill -9 -f px4_sitl 2>/dev/null || true
pkill -9 -f "bin/px4" 2>/dev/null || true
pkill -9 -f "zav_sim.py" 2>/dev/null || true
if grep -qi microsoft /proc/version 2>/dev/null && [ -f "$TASKKILL_BIN" ]; then
    "$TASKKILL_BIN" /F /IM kit.exe 2>/dev/null || true
fi
sleep 0.5

# 2. 세션 종료 시 전체 프로세스 강제 회수(Trap) 및 비정상 종료 에러 출력
cleanup() {
    echo ""
    echo "=========================================================="
    echo " [ZAV Simulator] 대시보드 종료: 모든 프로세스를 정리합니다..."
    echo "=========================================================="
    tmux kill-session -t $SESSION_NAME 2>/dev/null || true
    pkill -9 -f MicroXRCEAgent 2>/dev/null || true
    pkill -9 -f px4_sitl 2>/dev/null || true
    pkill -9 -f "bin/px4" 2>/dev/null || true
    pkill -9 -f "zav_sim.py" 2>/dev/null || true
    if grep -qi microsoft /proc/version 2>/dev/null && [ -f "$TASKKILL_BIN" ]; then
        "$TASKKILL_BIN" /F /IM kit.exe 2>/dev/null || true
    fi
    if [ -f "$ISAAC_LOG" ] && grep -qE "Traceback|ImportError|KeyError|FileNotFoundError|ModuleNotFoundError" "$ISAAC_LOG" 2>/dev/null; then
        echo "----------------------------------------------------------"
        echo " ⚠️ [진단 로그] Isaac Sim (zav_sim.py) 실행 중 발생한 에러:"
        tail -n 25 "$ISAAC_LOG"
        echo "----------------------------------------------------------"
    fi
    echo " [ZAV Simulator] 모든 프로세스 및 포트 정리 완료!"
}
trap cleanup EXIT SIGINT SIGTERM

# 3. Micro-XRCE-DDS Agent 백그라운드 기동
echo " [1/3] Micro-XRCE-DDS Agent 기동 [UDP 8888]..."
MicroXRCEAgent udp4 -p 8888 > /tmp/micro_xrce_agent.log 2>&1 &
sleep 1

# 4. tmux 세션 생성 (마우스 휠 스크롤 + 자동 클립보드 복사 바인딩)
echo " [2/3] tmux 분할 대시보드 생성 중..."
tmux new-session -d -s $SESSION_NAME -n "ZAV_SITL"
tmux set -t $SESSION_NAME -g mouse on
tmux set -t $SESSION_NAME -g pane-border-status top
tmux set -t $SESSION_NAME -g pane-border-format " #{pane_title} "

# 클립보드 자동 복사 바인딩: 드래그 종료 시 윈도우 clip.exe로 즉시 복사
if grep -qi microsoft /proc/version 2>/dev/null && [ -f "$CLIP_BIN" ]; then
    tmux bind-key -t $SESSION_NAME -T copy-mode MouseDragEnd1Pane send-keys -X copy-pipe-and-cancel "$CLIP_BIN" 2>/dev/null || true
    tmux bind-key -t $SESSION_NAME -T copy-mode-vi MouseDragEnd1Pane send-keys -X copy-pipe-and-cancel "$CLIP_BIN" 2>/dev/null || true
fi

# 5. 상단 팬 (Pane 0): PX4 SITL 실행 (Ctrl+C 시 tmux kill-session 연쇄 실행으로 즉시 폭파)
tmux select-pane -t $SESSION_NAME:0.0 -T "🔴 [상단] PX4 비행 콘솔 (종료: Ctrl+C)"
tmux send-keys -t $SESSION_NAME:0.0 "cd ~/PX4-Autopilot && make px4_sitl isaac_zav; tmux kill-session -t $SESSION_NAME" C-m

# 6. 하단 팬 (Pane 1): 40% 크기로 분할 후 Isaac Sim 기동
echo " [3/3] Pegasus / Isaac Sim 3D 뷰어 연결 중..."
tmux split-window -v -p 40 -t $SESSION_NAME:0.0
tmux select-pane -t $SESSION_NAME:0.1 -T "🟢 [하단] Pegasus / Isaac Sim 파이썬 로그"

    SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
    RUNNER_SCRIPT="/tmp/run_isaac_sim.sh"
    rm -f "$RUNNER_SCRIPT"

    if grep -qi microsoft /proc/version 2>/dev/null; then
        # --------------------------------------------------------------------------
        # [WSL2 환경]: Windows 호스트의 Isaac Sim 탐색 및 실행
        # 원리:
        #   1. C:\isaac-sim 또는 C:\issac-sim 등 윈도우 배치 파일(python.bat) 자동 탐색
        #   2. wslpath -w 로 리눅스 경로를 윈도우 UNC 경로(\\wsl.localhost\...)로 변환
        #   3. Windows cmd.exe는 홑따옴표(')를 인용부호로 인식하지 못하므로, call 문법 및
        #      쌍따옴표("") 표준 규격을 적용하여 안전하게 호출
        #   4. 실행 로그를 $ISAAC_LOG에 실시간 기록하고, 에러 발생 시 창이 바로 닫히지 않고
        #      키 입력을 대기하도록 안전장치(read -n 1) 마련
        # --------------------------------------------------------------------------
        WIN_ISAAC_BAT=""
        if [ -f "/mnt/c/isaac-sim/python.bat" ]; then
            WIN_ISAAC_BAT=$(wslpath -w "/mnt/c/isaac-sim/python.bat")
        elif [ -f "/mnt/c/issac-sim/python.bat" ]; then
            WIN_ISAAC_BAT=$(wslpath -w "/mnt/c/issac-sim/python.bat")
        else
            # 윈도우 바탕화면 또는 옴니버스 기본 경로 자동 탐색
            DETECTED=$(find /mnt/c/Users/*/Desktop /mnt/c/Users/*/AppData/Local/ov/pkg -maxdepth 3 -name "python.bat" 2>/dev/null | grep -i "isaac" | head -n 1)
            if [ -n "$DETECTED" ]; then
                WIN_ISAAC_BAT=$(wslpath -w "$DETECTED")
            fi
        fi

        # zav_sim.py 윈도우 경로 산출 (C:\UAVProject 우선, 없으면 WSL 경로 자동 변환)
        if [ -f "/mnt/c/UAVProject/PegasusSimulator/examples/zav_sim.py" ]; then
            WIN_SIM_SCRIPT="C:\\UAVProject\\PegasusSimulator\\examples\\zav_sim.py"
        elif [ -f "$HOME/PegasusSimulator/examples/zav_sim.py" ]; then
            WIN_SIM_SCRIPT=$(wslpath -w "$HOME/PegasusSimulator/examples/zav_sim.py")
        elif [ -f "$SCRIPT_DIR/../pegasus/zav_sim.py" ]; then
            WIN_SIM_SCRIPT=$(wslpath -w "$SCRIPT_DIR/../pegasus/zav_sim.py")
        else
            WIN_SIM_SCRIPT=""
        fi

        cat << RUNNER_EOF > "$RUNNER_SCRIPT"
#!/bin/bash
echo " [Isaac Sim] PX4 SITL 네트워크 소켓(TCP 4560) 대기 중 (3초)..."
sleep 3
echo " [Isaac Sim] Windows Isaac Sim을 실행합니다..."
echo "  - Python Batch: $WIN_ISAAC_BAT"
echo "  - Sim Script:   $WIN_SIM_SCRIPT"
echo "----------------------------------------------------------"

if [ -n "$WIN_ISAAC_BAT" ] && [ -n "$WIN_SIM_SCRIPT" ]; then
    /mnt/c/Windows/System32/cmd.exe /c call "$WIN_ISAAC_BAT" "$WIN_SIM_SCRIPT" 2>&1 | tee "$ISAAC_LOG"
    EXIT_CODE=\${PIPESTATUS[0]}
else
    echo "⚠️ Windows Isaac Sim 또는 zav_sim.py 경로를 찾지 못했습니다. WSL 파이썬으로 대체 실행합니다."
    python3 ~/PegasusSimulator/examples/zav_sim.py 2>&1 | tee "$ISAAC_LOG"
    EXIT_CODE=\${PIPESTATUS[0]}
fi

if [ "\$EXIT_CODE" -ne 0 ]; then
    echo ""
    echo "=========================================================="
    echo " ⚠️ Isaac Sim 실행 중 오류 발생 (종료 코드: \$EXIT_CODE)"
    echo " 상세 에러 로그: $ISAAC_LOG"
    echo "=========================================================="
    echo " 터미널을 종료하려면 아무 키나 누르세요 (자동 종료 방지)..."
    read -n 1 -s
fi

tmux kill-session -t "$SESSION_NAME" 2>/dev/null || true
RUNNER_EOF

    else
        # --------------------------------------------------------------------------
        # [Native Linux 환경 (팀원 PC)]: 가상환경 및 Standalone python.sh 자동 감지
        # --------------------------------------------------------------------------
        if [ -n "$VIRTUAL_ENV" ] && [ -f "$VIRTUAL_ENV/bin/python" ]; then
            ISAAC_PY="$VIRTUAL_ENV/bin/python"
        elif [ -f "$HOME/isaac_env/bin/python" ]; then
            ISAAC_PY="$HOME/isaac_env/bin/python"
        elif [ -f "$HOME/isaacsim_env/bin/python" ]; then
            ISAAC_PY="$HOME/isaac_env/bin/python"
        elif [ -f "$HOME/isaacsim/python.sh" ]; then
            ISAAC_PY="$HOME/isaacsim/python.sh"
        elif [ -f "$HOME/isaac-sim/python.sh" ]; then
            ISAAC_PY="$HOME/isaac-sim/python.sh"
        else
            ISAAC_PY="python3"
        fi

        cat << RUNNER_EOF > "$RUNNER_SCRIPT"
#!/bin/bash
echo " [Isaac Sim] PX4 SITL 네트워크 소켓(TCP 4560) 대기 중 (3초)..."
sleep 3
echo " [Isaac Sim] Pegasus / Isaac Sim을 실행합니다..."
echo "  - Python Bin: $ISAAC_PY"
echo "----------------------------------------------------------"

$ISAAC_PY ~/PegasusSimulator/examples/zav_sim.py 2>&1 | tee "$ISAAC_LOG"
EXIT_CODE=\${PIPESTATUS[0]}

if [ "\$EXIT_CODE" -ne 0 ]; then
    echo ""
    echo "=========================================================="
    echo " ⚠️ Isaac Sim 실행 중 오류 발생 (종료 코드: \$EXIT_CODE)"
    echo " 상세 에러 로그: $ISAAC_LOG"
    echo "=========================================================="
    echo " 터미널을 종료하려면 아무 키나 누르세요 (자동 종료 방지)..."
    read -n 1 -s
fi

tmux kill-session -t "$SESSION_NAME" 2>/dev/null || true
RUNNER_EOF
    fi

    chmod +x "$RUNNER_SCRIPT"
    tmux send-keys -t $SESSION_NAME:0.1 "$RUNNER_SCRIPT" C-m

# 7. 상단 콘솔로 포커스 맞추고 대시보드 화면으로 전환
tmux select-pane -t $SESSION_NAME:0.0
tmux attach-session -t $SESSION_NAME
