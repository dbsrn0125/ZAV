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

# 5. 상단 팬 (Pane 0): PX4 SITL 실행 (Ctrl+C 시 trap으로 tmux 세션 즉시 폭파)
tmux select-pane -t $SESSION_NAME:0.0 -T "🔴 [상단] PX4 비행 콘솔 (종료: Ctrl+C)"
tmux send-keys -t $SESSION_NAME:0.0 "trap 'tmux kill-session -t $SESSION_NAME 2>/dev/null' INT TERM EXIT; cd ~/PX4-Autopilot && make px4_sitl isaac_zav; tmux kill-session -t $SESSION_NAME 2>/dev/null" C-m

# 6. 하단 팬 (Pane 1): 40% 크기로 분할 후 Isaac Sim 기동
echo " [3/3] Pegasus / Isaac Sim 3D 뷰어 연결 중..."
tmux split-window -v -p 40 -t $SESSION_NAME:0.0
tmux select-pane -t $SESSION_NAME:0.1 -T "🟢 [하단] Pegasus / Isaac Sim 파이썬 로그 (종료: Ctrl+C)"

    SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
    RUNNER_SCRIPT="$SCRIPT_DIR/run_isaac_sim.sh"
    if [ ! -f "$RUNNER_SCRIPT" ]; then
        RUNNER_SCRIPT="$HOME/zenith_ws/simulation/scripts/run_isaac_sim.sh"
    fi
    chmod +x "$RUNNER_SCRIPT"
    tmux send-keys -t $SESSION_NAME:0.1 "trap 'tmux kill-session -t $SESSION_NAME 2>/dev/null' INT TERM EXIT; $RUNNER_SCRIPT; tmux kill-session -t $SESSION_NAME 2>/dev/null" C-m

# 7. 상단 콘솔로 포커스 맞추고 대시보드 화면으로 전환
tmux select-pane -t $SESSION_NAME:0.0
tmux attach-session -t $SESSION_NAME
