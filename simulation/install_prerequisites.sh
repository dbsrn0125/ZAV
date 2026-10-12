#!/usr/bin/env bash
# ==============================================================================
# [Zenith Aerial Vehicle] 시뮬레이션 기반 환경 자동 설치 스크립트
#
# 팀원의 Ubuntu 22.04 / WSL2 환경에서 PX4-Autopilot, PegasusSimulator,
# Micro-XRCE-DDS-Agent 및 필수 시스템 패키지를 자동으로 설치 및 빌드합니다.
# ==============================================================================

set -e

GREEN='\033[0;32m'
BLUE='\033[0;34m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
NC='\033[0m'

echo -e "${BLUE}================================================================${NC}"
echo -e "${BLUE}   [ZAV Simulation] 사전 필수 환경(Prerequisites) 자동 설치     ${NC}"
echo -e "${BLUE}================================================================${NC}"

# 1. 필수 시스템 유틸리티 설치
echo -e "\n${YELLOW}[1/4] 시스템 필수 도구(tmux, git, cmake, build-essential) 설치...${NC}"
sudo apt-get update
sudo apt-get install -y git cmake build-essential python3-pip tmux net-tools lsof curl

# 2. Micro-XRCE-DDS-Agent 설치 (PX4 <-> ROS 2 통신 브릿지)
echo -e "\n${YELLOW}[2/4] Micro-XRCE-DDS-Agent 빌드 및 시스템 설치...${NC}"
if ! command -v MicroXRCEAgent &> /dev/null; then
    cd "$HOME"
    if [ ! -d "Micro-XRCE-DDS-Agent" ]; then
        git clone https://github.com/eProsima/Micro-XRCE-DDS-Agent.git
    fi
    cd Micro-XRCE-DDS-Agent
    mkdir -p build && cd build
    cmake ..
    make -j$(nproc)
    sudo make install
    sudo ldconfig /usr/local/lib/
    echo -e "${GREEN}  - MicroXRCEAgent 설치 완료!${NC}"
else
    echo -e "${GREEN}  - MicroXRCEAgent가 이미 설치되어 있습니다.${NC}"
fi

# 3. PX4-Autopilot 저장소 클론 (v1.14.3 LTS)
echo -e "\n${YELLOW}[3/4] PX4-Autopilot (v1.14.3) 저장소 다운로드...${NC}"
if [ ! -d "$HOME/PX4-Autopilot" ]; then
    cd "$HOME"
    echo -e "  - PX4-Autopilot 서브모듈 포함 클론 중 (수 분 소요될 수 있습니다)..."
    git clone -b v1.14.3 --recursive https://github.com/PX4/PX4-Autopilot.git "$HOME/PX4-Autopilot"
    echo -e "${GREEN}  - PX4-Autopilot 클론 완료!${NC}"
    
    # PX4 의존성 스크립트 실행 안내
    echo -e "  - PX4 우분투 기본 의존성 설치 스크립트 실행 중..."
    bash "$HOME/PX4-Autopilot/Tools/setup/ubuntu.sh" --no-nuttx --no-sim-tools || true
else
    echo -e "${GREEN}  - PX4-Autopilot 디렉토리가 이미 존재합니다 ($HOME/PX4-Autopilot).${NC}"
fi

# 4. PegasusSimulator 저장소 클론
echo -e "\n${YELLOW}[4/4] PegasusSimulator 저장소 다운로드...${NC}"
if [ ! -d "$HOME/PegasusSimulator" ]; then
    cd "$HOME"
    git clone https://github.com/PegasusSimulator/PegasusSimulator.git "$HOME/PegasusSimulator"
    echo -e "${GREEN}  - PegasusSimulator 클론 완료!${NC}"
else
    echo -e "${GREEN}  - PegasusSimulator 디렉토리가 이미 존재합니다 ($HOME/PegasusSimulator).${NC}"
fi

echo -e "\n${GREEN}================================================================${NC}"
echo -e "${GREEN}   [ZAV Simulation] 사전 필수 환경 설치가 완료되었습니다!       ${NC}"
echo -e "${GREEN}   1. NVIDIA Isaac Sim이 설치되어 있는지 확인하세요.             ${NC}"
echo -e "${GREEN}   2. 이제 './setup_sim.sh'를 실행하여 ZAV 환경을 등록하세요!   ${NC}"
echo -e "${GREEN}================================================================${NC}"
