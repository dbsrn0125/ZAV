#!/usr/bin/env bash
# ==============================================================================
# [Zenith Aerial Vehicle] ZAV Simulation 자동 환경 구축 스크립트
#
# 팀원(Ubuntu/WSL2) 환경에서 PX4 SITL 에어프레임/CMake 타겟(isaac_zav) 등록,
# Pegasus 기체 모델 배치, params.py / __init__.py 자동 등록, 런처 등록 및
# ROS 2 제어기 빌드를 한 번에 수행합니다.
# ==============================================================================

set -e

# 터미널 출력 색상 정의
GREEN='\033[0;32m'
BLUE='\033[0;34m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
NC='\033[0m'

echo -e "${BLUE}==========================================================${NC}"
echo -e "${BLUE}   [ZAV Simulation] 시뮬레이션 환경 자동 설정 시작...   ${NC}"
echo -e "${BLUE}==========================================================${NC}"

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_DIR="$(dirname "$SCRIPT_DIR")"

PX4_DIR="${PX4_DIR:-$HOME/PX4-Autopilot}"
PEGASUS_DIR="${PEGASUS_DIR:-$HOME/PegasusSimulator}"

# 1. PX4-Autopilot 에어프레임 및 CMake 타겟(isaac_zav) 설정
echo -e "\n${YELLOW}[1/4] PX4 에어프레임 및 CMake 타겟(isaac_zav) 설정 중...${NC}"
if [ -d "$PX4_DIR" ]; then
    AIRFRAME_DIR="$PX4_DIR/ROMFS/px4fmu_common/init.d-posix/airframes"
    mkdir -p "$AIRFRAME_DIR"
    cp "$SCRIPT_DIR/px4/10020_isaac_zav" "$AIRFRAME_DIR/"
    echo -e "  - 10020_isaac_zav 에어프레임 복사 완료: $AIRFRAME_DIR"

    # 이미 빌드된 ROMFS 디렉토리가 있으면 즉시 동기화 및 이전 파라미터 캐시 초기화
    BUILD_AIRFRAME_DIR="$PX4_DIR/build/px4_sitl_default/etc/init.d-posix/airframes"
    if [ -d "$BUILD_AIRFRAME_DIR" ]; then
        cp "$SCRIPT_DIR/px4/10020_isaac_zav" "$BUILD_AIRFRAME_DIR/"
        rm -f "$PX4_DIR/build/px4_sitl_default/rootfs/parameters.bson" \
              "$PX4_DIR/build/px4_sitl_default/rootfs/parameters_backup.bson"
        echo -e "  - 빌드된 ROMFS 에어프레임 즉시 갱신 및 구버전 파라미터 캐시 초기화 완료"
    fi

    # (A) ROMFS airframes/CMakeLists.txt 에 10020_isaac_zav 등록
    CMAKE_FILE="$AIRFRAME_DIR/CMakeLists.txt"
    if [ -f "$CMAKE_FILE" ]; then
        if ! grep -q "10020_isaac_zav" "$CMAKE_FILE"; then
            if grep -q "10016_none_iris" "$CMAKE_FILE"; then
                sed -i '/10016_none_iris/a \\t10020_isaac_zav' "$CMAKE_FILE"
            else
                sed -i '/px4_add_romfs_files(/a \\t10020_isaac_zav' "$CMAKE_FILE"
            fi
            echo -e "  - airframes/CMakeLists.txt 에 10020_isaac_zav 등록 완료"
        else
            echo -e "  - airframes/CMakeLists.txt 에 이미 등록되어 있습니다."
        fi
    fi

    # (B) simulator_mavlink/CMakeLists.txt 에 'isaac_zav' 빌드/실행 타겟 등록
    MAVLINK_CMAKE="$PX4_DIR/src/modules/simulation/simulator_mavlink/CMakeLists.txt"
    if [ -f "$MAVLINK_CMAKE" ]; then
        if ! grep -q "add_custom_target(isaac_zav" "$MAVLINK_CMAKE"; then
            cat << 'CMAKEEOF' >> "$MAVLINK_CMAKE"

# isaac_zav (ZAV Custom Quadcopter for Isaac Sim / Pegasus)
add_custom_target(isaac_zav
	COMMAND ${CMAKE_COMMAND} -E env PX4_SYS_AUTOSTART=10020 $<TARGET_FILE:px4>
	WORKING_DIRECTORY ${SITL_WORKING_DIR}
	USES_TERMINAL
	DEPENDS
		px4
		${PX4_SOURCE_DIR}/ROMFS/px4fmu_common/init.d-posix/airframes/10020_isaac_zav
	COMMENT "launching px4 isaac_zav (SYS_AUTOSTART=10020)"
)
CMAKEEOF
            echo -e "  - simulator_mavlink/CMakeLists.txt 에 'isaac_zav' 타겟 등록 완료"
        else
            echo -e "  - simulator_mavlink/CMakeLists.txt 에 'isaac_zav' 타겟이 이미 등록되어 있습니다."
        fi
    fi

    # (C) 기존 PX4 빌드 캐시가 존재할 경우 CMake 재구성하여 ninja 타겟 즉시 반영
    if [ -f "$PX4_DIR/build/px4_sitl_default/CMakeCache.txt" ]; then
        echo -e "  - 기존 PX4 빌드 디렉토리에 isaac_zav 타겟 갱신 중..."
        cmake "$PX4_DIR/build/px4_sitl_default" > /dev/null
        echo -e "  - PX4 Ninja 빌드 타겟(isaac_zav) 갱신 완료!"
    fi
else
    echo -e "${RED}  [경고] $PX4_DIR 경로를 찾을 수 없습니다.${NC}"
fi

# 2. Pegasus Simulator 기체 및 USD 에셋 설정 + 파이썬 모듈 자동 등록
echo -e "\n${YELLOW}[2/4] Pegasus 기체, USD 에셋 및 파이썬 모듈 등록 중...${NC}"
if [ -d "$PEGASUS_DIR" ]; then
    SIM_ROOT="$PEGASUS_DIR/extensions/pegasus.simulator/pegasus/simulator"
    VEHICLE_DIR="$SIM_ROOT/logic/vehicles/multirotors"
    VEHICLES_INIT="$SIM_ROOT/logic/vehicles/__init__.py"
    MULTIROTORS_INIT="$VEHICLE_DIR/__init__.py"
    PARAMS_FILE="$SIM_ROOT/params.py"
    ASSET_DIR="$SIM_ROOT/assets/Robots/ZAV_v1"
    EXAMPLE_DIR="$PEGASUS_DIR/examples"

    mkdir -p "$VEHICLE_DIR" "$ASSET_DIR" "$EXAMPLE_DIR"
    cp "$SCRIPT_DIR/pegasus/zav_v1.py" "$VEHICLE_DIR/"
    cp "$SCRIPT_DIR/pegasus/assets/ZAV_v1/ZAV_v1.usd" "$ASSET_DIR/"
    cp "$SCRIPT_DIR/pegasus/zav_sim.py" "$EXAMPLE_DIR/"
    echo -e "  - zav_v1.py (공력 모델), ZAV_v1.usd (3D 모델), zav_sim.py 복사 완료"

    # (A) multirotors/__init__.py 에 ZAV_v1 클래스 등록
    if [ ! -f "$MULTIROTORS_INIT" ]; then
        cat << 'PYEOF' > "$MULTIROTORS_INIT"
from .iris import Iris
from .zav_v1 import ZAV_v1
PYEOF
        echo -e "  - multirotors/__init__.py 생성 및 ZAV_v1 등록 완료"
    elif ! grep -q "ZAV_v1" "$MULTIROTORS_INIT"; then
        echo "from .zav_v1 import ZAV_v1" >> "$MULTIROTORS_INIT"
        echo -e "  - multirotors/__init__.py 에 ZAV_v1 추가 완료"
    else
        echo -e "  - multirotors/__init__.py 에 ZAV_v1이 이미 등록되어 있습니다."
    fi

    # (B) vehicles/__init__.py 에 ZAV_v1 클래스 등록
    if [ -f "$VEHICLES_INIT" ]; then
        if ! grep -q "ZAV_v1" "$VEHICLES_INIT"; then
            echo "from .multirotors import Iris, ZAV_v1" >> "$VEHICLES_INIT"
            echo -e "  - vehicles/__init__.py 에 ZAV_v1 등록 완료"
        else
            echo -e "  - vehicles/__init__.py 에 ZAV_v1이 이미 등록되어 있습니다."
        fi
    fi

    # (C) params.py 의 ROBOTS 딕셔너리에 ZAV_v1 USD 경로 자동 등록
    if [ -f "$PARAMS_FILE" ]; then
        if ! grep -q '"ZAV_v1"' "$PARAMS_FILE"; then
            python3 - "$PARAMS_FILE" << 'PYEOF'
import sys, pathlib
p = pathlib.Path(sys.argv[1])
text = p.read_text(encoding="utf-8")
target = '"PegasusOptimized": ROBOTS_ASSETS + "/Pegasus/pegasus_optimized.usdc"'
replacement = target + ',\n    "ZAV_v1": ROBOTS_ASSETS + "/ZAV_v1/ZAV_v1.usd"'
if target in text:
    text = text.replace(target, replacement, 1)
    p.write_text(text, encoding="utf-8")
    print("  - params.py ROBOTS 딕셔너리에 ZAV_v1 자동 등록 완료")
else:
    print("  - [경고] params.py 에서 ROBOTS 기본 패턴을 찾지 못했습니다.")
PYEOF
        else
            echo -e "  - params.py 에 ZAV_v1이 이미 등록되어 있습니다."
        fi
    fi

    # (D) Native Linux Isaac Sim 가상환경(isaac_env, isaacsim_env 등)에 필수 패키지 및 pegasus 경로 등록
    for ENV_PY in "${VIRTUAL_ENV:+$VIRTUAL_ENV/bin/python}" "$HOME/isaac_env/bin/python" "$HOME/isaacsim_env/bin/python"; do
        if [ -n "$ENV_PY" ] && [ -f "$ENV_PY" ]; then
            echo -e "  - Isaac Sim 파이썬 환경 감지 ($ENV_PY): 필수 의존성(pymavlink, scipy, pyyaml) 및 pegasus 경로 등록 중..."
            "$ENV_PY" -m pip install --quiet numpy pymavlink scipy pyyaml || true
            "$ENV_PY" - "$PEGASUS_DIR/extensions/pegasus.simulator" << 'PYEOF'
import sys, site, os
ext_path = sys.argv[1]
for sp in site.getsitepackages():
    if os.path.isdir(sp):
        pth_file = os.path.join(sp, "pegasus_simulator.pth")
        with open(pth_file, "w", encoding="utf-8") as f:
            f.write(ext_path + "\n")
        break
PYEOF
            echo -e "  - $ENV_PY 에 pegasus.simulator 모듈 경로(.pth) 등록 완료!"
            break
        fi
    done

    # (E) Windows + WSL2 환경: 윈도우 Isaac Sim 감지 시 pip 의존성(pymavlink, scipy, pyyaml) 자동 동기화 설치
    if grep -qi microsoft /proc/version 2>/dev/null && [ -f "/mnt/c/Windows/System32/cmd.exe" ]; then
        WIN_ISAAC_BAT=""
        for CANDIDATE in "/mnt/c/isaac-sim/python.bat" "/mnt/c/issac-sim/python.bat" /mnt/c/Users/*/AppData/Local/ov/pkg/isaac*sim*/python.bat; do
            if [ -f "$CANDIDATE" ]; then
                WIN_ISAAC_BAT=$(wslpath -w "$CANDIDATE")
                break
            fi
        done
        if [ -n "$WIN_ISAAC_BAT" ]; then
            echo -e "  - Windows Isaac Sim 감지 ($WIN_ISAAC_BAT): 필수 패키지(pymavlink, scipy, pyyaml) 동기화 설치 중..."
            /mnt/c/Windows/System32/cmd.exe /c call "$WIN_ISAAC_BAT" -m pip install --quiet pymavlink scipy pyyaml || true
            echo -e "  - Windows Isaac Sim 필수 의존성(pymavlink, scipy, pyyaml) 설치 완료!"
        fi
    fi
else
    echo -e "${RED}  [경고] $PEGASUS_DIR 경로를 찾을 수 없습니다.${NC}"
fi

# 3. 2분할 대시보드 런처 및 단축어 등록
echo -e "\n${YELLOW}[3/4] 2분할 대시보드 런처 및 bashrc 단축어 등록...${NC}"
cp "$SCRIPT_DIR/scripts/run_zav_sim.sh" "$HOME/run_zav_sim.sh"
chmod +x "$HOME/run_zav_sim.sh"

if ! grep -q "alias zav_sim=" "$HOME/.bashrc"; then
    echo "alias zav_sim='$HOME/run_zav_sim.sh'" >> "$HOME/.bashrc"
    echo -e "  - ~/.bashrc에 'alias zav_sim' 등록 완료"
else
    echo -e "  - ~/.bashrc에 'alias zav_sim'이 이미 존재합니다."
fi

# 4. ROS 2 패키지 다운로드 및 PX4 원본 메시지 동기화 빌드 (px4_msgs, zav_control)
echo -e "\n${YELLOW}[4/4] ROS 2 제어 패키지 빌드 중...${NC}"
if [ ! -d "$WS_DIR/src/px4_msgs" ]; then
    echo -e "  - px4_msgs 기본 저장소 클론 중..."
    git clone -b release/1.14 https://github.com/PX4/px4_msgs.git "$WS_DIR/src/px4_msgs"
fi

# 설치된 PX4-Autopilot의 원본 .msg 파일들과 100% 바이트 단위로 일치하도록 동기화
if [ -d "$PX4_DIR/msg" ] && [ -d "$WS_DIR/src/px4_msgs/msg" ]; then
    rm -f "$WS_DIR/src/px4_msgs/msg/"*.msg
    cp "$PX4_DIR/msg/"*.msg "$WS_DIR/src/px4_msgs/msg/"
    if [ -d "$PX4_DIR/msg/versioned" ]; then
        cp "$PX4_DIR/msg/versioned/"*.msg "$WS_DIR/src/px4_msgs/msg/"
    fi
    echo -e "  - PX4-Autopilot 원본 .msg 정의를 px4_msgs에 100% 동기화 완료"
fi

if [ -d "$WS_DIR/src/zav_control" ]; then
    cd "$WS_DIR"
    if [ -f /opt/ros/humble/setup.bash ]; then
        source /opt/ros/humble/setup.bash
    fi
    colcon build --symlink-install --packages-select px4_msgs zav_control
    echo -e "${GREEN}  - ROS 2 패키지 빌드 완료!${NC}"
fi

echo -e "\n${GREEN}==========================================================${NC}"
echo -e "${GREEN}   [ZAV Simulation] 모든 환경 설정이 성공적으로 완료되었습니다!   ${NC}"
echo -e "${GREEN}   새 터미널을 열거나 'source ~/.bashrc' 실행 후,         ${NC}"
echo -e "${GREEN}   'zav_sim' 명령어를 입력하면 시뮬레이터가 기동됩니다.   ${NC}"
echo -e "${GREEN}==========================================================${NC}"
