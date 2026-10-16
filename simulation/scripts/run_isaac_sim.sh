#!/bin/bash
# ==============================================================================
# Pegasus / Isaac Sim 런처 스크립트 (WSL2 및 Native Linux 자동 감지)
# ==============================================================================
# 기능:
#   1. PX4 SITL 소켓(TCP 4560) 대기 (3초)
#   2. WSL2 vs Native Linux 환경 자동 감지 및 최적화된 파이썬 실행
#   3. Isaac Sim 내장 ROS 2 Bridge (humble, rmw_fastrtps_cpp) 라이브러리 경로 자동 로드
#   4. Windows 호스트의 Isaac Sim 실행 시 wslpath -m (포워드 슬래시 /) 변환을 통해
#      백슬래시(\) 이스케이프 왜곡 및 UNC 경로 해석 오류 원천 방지
#   5. 실시간 에러 로깅($ISAAC_LOG) 및 비정상 종료 시 대시보드 자동 닫힘 방지(read -n 1)
# ==============================================================================

SESSION_NAME="zav_sim"
ISAAC_LOG="/tmp/zav_isaac_sim.log"

trap 'tmux kill-session -t "$SESSION_NAME" 2>/dev/null; exit 0' INT TERM

echo " [Isaac Sim] PX4 SITL 네트워크 소켓(TCP 4560) 대기 중 (3초)..."
sleep 3

if grep -qi microsoft /proc/version 2>/dev/null; then
    # --------------------------------------------------------------------------
    # [WSL2 환경]: Windows 호스트의 Isaac Sim 탐색 및 실행
    # --------------------------------------------------------------------------
    WIN_ISAAC_BAT=""
    WIN_ROS2_LIB=""
    for cand in "/mnt/c/isaac-sim/python.bat" "/mnt/c/issac-sim/python.bat" /mnt/c/Users/*/AppData/Local/ov/pkg/isaac*sim*/python.bat /mnt/c/Users/*/Desktop/*isaac*/python.bat; do
        if [ -f "$cand" ]; then
            WIN_ISAAC_BAT=$(wslpath -m "$cand")
            cand_dir=$(dirname "$cand")
            if [ -d "$cand_dir/exts/isaacsim.ros2.bridge/humble/lib" ]; then
                WIN_ROS2_LIB=$(wslpath -w "$cand_dir/exts/isaacsim.ros2.bridge/humble/lib")
            fi
            break
        fi
    done

    # zav_sim.py 탐색 (C:\UAVProject 우선, 없으면 WSL 내부 PegasusSimulator 탐색)
    if [ -f "/mnt/c/UAVProject/PegasusSimulator/examples/zav_sim.py" ]; then
        WIN_SIM_SCRIPT="C:/UAVProject/PegasusSimulator/examples/zav_sim.py"
    elif [ -f "$HOME/PegasusSimulator/examples/zav_sim.py" ]; then
        WIN_SIM_SCRIPT=$(wslpath -m "$HOME/PegasusSimulator/examples/zav_sim.py")
    else
        SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
        WIN_SIM_SCRIPT=$(wslpath -m "$SCRIPT_DIR/../pegasus/zav_sim.py")
    fi

    echo " [Isaac Sim] Windows Isaac Sim을 실행합니다..."
    echo "  - Python Batch: $WIN_ISAAC_BAT"
    echo "  - Sim Script:   $WIN_SIM_SCRIPT"
    echo "  - ROS2 Bridge:  humble (rmw_fastrtps_cpp)"
    echo "----------------------------------------------------------"

    if [ -n "$WIN_ISAAC_BAT" ] && [ -n "$WIN_SIM_SCRIPT" ]; then
        (cd /mnt/c && /mnt/c/Windows/System32/cmd.exe /c "set ROS_DISTRO=humble&& set RMW_IMPLEMENTATION=rmw_fastrtps_cpp&& set PATH=%PATH%;$WIN_ROS2_LIB&& call \"$WIN_ISAAC_BAT\" \"$WIN_SIM_SCRIPT\"") 2>&1 | tee "$ISAAC_LOG"
        EXIT_CODE=${PIPESTATUS[0]}
    else
        echo "⚠️ Windows Isaac Sim 또는 zav_sim.py 경로를 찾지 못했습니다. WSL 파이썬으로 대체 실행합니다."
        python3 ~/PegasusSimulator/examples/zav_sim.py 2>&1 | tee "$ISAAC_LOG"
        EXIT_CODE=${PIPESTATUS[0]}
    fi

else
    # --------------------------------------------------------------------------
    # [Native Linux 환경 (팀원 PC)]: 가상환경 및 Standalone python.sh 자동 감지
    # --------------------------------------------------------------------------
    if [ -n "$VIRTUAL_ENV" ] && [ -f "$VIRTUAL_ENV/bin/python" ]; then
        ISAAC_PY="$VIRTUAL_ENV/bin/python"
    elif [ -f "$HOME/isaac_env/bin/python" ]; then
        ISAAC_PY="$HOME/isaac_env/bin/python"
    elif [ -f "$HOME/isaacsim_env/bin/python" ]; then
        ISAAC_PY="$HOME/isaacsim_env/bin/python"
    elif [ -f "$HOME/isaacsim/python.sh" ]; then
        ISAAC_PY="$HOME/isaacsim/python.sh"
    elif [ -f "$HOME/isaac-sim/python.sh" ]; then
        ISAAC_PY="$HOME/isaac-sim/python.sh"
    else
        ISAAC_PY="python3"
    fi

    # Isaac Sim 내장 ROS 2 Bridge (humble) 환경변수 및 LD_LIBRARY_PATH 자동 구성
    export ROS_DISTRO="${ROS_DISTRO:-humble}"
    export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"
    ROS2_BRIDGE_LIB=$("$ISAAC_PY" -c "import os, isaacsim; p=os.path.join(os.path.dirname(isaacsim.__file__), 'exts', 'isaacsim.ros2.bridge', os.environ.get('ROS_DISTRO','humble'), 'lib'); print(p if os.path.isdir(p) else '')" 2>/dev/null || true)
    if [ -z "$ROS2_BRIDGE_LIB" ]; then
        ISAAC_ROOT="$(cd "$(dirname "$ISAAC_PY")" && pwd)"
        if [ -d "$ISAAC_ROOT/exts/isaacsim.ros2.bridge/$ROS_DISTRO/lib" ]; then
            ROS2_BRIDGE_LIB="$ISAAC_ROOT/exts/isaacsim.ros2.bridge/$ROS_DISTRO/lib"
        fi
    fi
    if [ -n "$ROS2_BRIDGE_LIB" ]; then
        export LD_LIBRARY_PATH="$ROS2_BRIDGE_LIB${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
    fi

    echo " [Isaac Sim] Pegasus / Isaac Sim을 실행합니다..."
    echo "  - Python Bin:  $ISAAC_PY"
    echo "  - ROS2 Bridge: $ROS_DISTRO ($RMW_IMPLEMENTATION)"
    echo "----------------------------------------------------------"

    $ISAAC_PY ~/PegasusSimulator/examples/zav_sim.py 2>&1 | tee "$ISAAC_LOG"
    EXIT_CODE=${PIPESTATUS[0]}
fi

if [ "$EXIT_CODE" -ne 0 ]; then
    echo ""
    echo "=========================================================="
    echo " ⚠️ Isaac Sim 실행 중 오류 발생 (종료 코드: $EXIT_CODE)"
    echo " 상세 에러 로그: $ISAAC_LOG"
    echo "=========================================================="
    echo " 터미널을 종료하려면 아무 키나 누르세요 (자동 종료 방지)..."
    read -n 1 -s
fi

tmux kill-session -t "$SESSION_NAME" 2>/dev/null || true
