# ZAV Simulation Environment

Pegasus Simulator (NVIDIA Isaac Sim) 및 PX4 SITL 기반 **Zenith Aerial Vehicle (ZAV)** 쿼드콥터 시뮬레이션 환경입니다.

---

## 📌 주요 특징
1. **정밀 물리/공력 모델**:
   - T-Motor F90 1500KV 모터 + 7인치 프롭 6S 실측 데이터 기반 $\omega_{\max}=1937.4\ \text{rad/s}$, $C_T=5.4935\times 10^{-6}$, $C_M=9.888\times 10^{-8}$
   - 호버 스로틀 33% 매칭 (`zav_v1.py` & `10020_isaac_zav`)
2. **착륙 완충 지면**:
   - 지면 반발 계수 `restitution=0.0` 적용으로 착륙 바운싱 없는 안정적 안착 및 착륙 감지
3. **올인원 2분할 대시보드 런처**:
   - `tmux` 기반으로 상단 PX4 SITL 콘솔(60%), 하단 Isaac Sim 백엔드 로그(40%) 분할 출력
   - `Ctrl+C` 한 번으로 백그라운드 프로세스 일괄 정리
4. **ROS 2 FPV 키보드 제어기**:
   - `zav_control` 패키지 제공 (`teleop_keyboard.py`)
   - 기체 방향(Yaw) 기준 1인칭 FPV 직관적 조종 지원

---

## 🛠️ 팀원용 단계별 설정 가이드 (Setup Guide)

처음 환경을 구축하는 팀원은 아래 순서대로 진행해 주세요.

### [0단계] NVIDIA Isaac Sim 설치 (사전 준비)
Isaac Sim은 용량이 크므로 [NVIDIA Omniverse Launcher](https://www.nvidia.com/ko-kr/omniverse/)를 통해 직접 설치해야 합니다.
- **권장 버전**: Isaac Sim 2023.1.1 또는 4.x
- **설치 위치**: 기본 경로 (Ubuntu: `~/.local/share/ov/pkg/isaac-sim-...` / Windows: `C:\isaac-sim...`)

---

### [1단계] 기반 환경 자동 설치 (PX4, Pegasus, DDS)
저장소 클론 후 아래 명령어를 실행하여 필수 패키지를 한 번에 설치합니다:
```bash
cd ~/zenith_ws/simulation
./install_prerequisites.sh
```
> **자동 수행 항목**:
> - `tmux`, `cmake`, `build-essential` 등 필수 시스템 도구 설치
> - `Micro-XRCE-DDS-Agent` 자동 빌드 및 시스템 설치 (PX4 <-> ROS 2 통신 브릿지)
> - `PX4-Autopilot` (v1.14.3 LTS) 소스코드 클론 및 환경 설정
> - `PegasusSimulator` 확장 패키지 클론

---

### [2단계] ZAV 전용 기체 및 런처 등록
기반 환경이 준비되었으면, ZAV 전용 설정을 등록합니다:
```bash
cd ~/zenith_ws/simulation
./setup_sim.sh
```
> **자동 수행 항목**:
> - PX4 에어프레임(`10020_isaac_zav`) 복사 및 `CMakeLists.txt` 등록
> - Pegasus 기체 공력 모델(`zav_v1.py`), 3D 로봇 USD 모델(`ZAV_v1.usd`), 런처(`run_zav_plane.py`) 복사
> - `~/.bashrc`에 단축어 `zav_sim` 등록
> - ROS 2 `px4_msgs`, `zav_control` 패키지 자동 빌드

---

## 🚀 시뮬레이션 실행 및 비행 테스트

### 1. 시뮬레이션 대시보드 기동
터미널을 새로 열거나 `source ~/.bashrc` 실행 후:
```bash
zav_sim
```
- 상단(60%): PX4 콘솔 화면 (GPS 고정, 시동, 비행 모드 출력)
- 하단(40%): Pegasus / Isaac Sim 렌더링 로그
- 마우스 드래그로 텍스트 선택 시 윈도우/시스템 클립보드에 자동 복사됩니다.
- 종료 시 아무 창에서나 **`Ctrl+C`**를 누르면 모든 프로세스가 깔끔하게 자동 종료됩니다.

---

### 2. ROS 2 FPV 키보드 조종
새 터미널을 열고 조종 노드를 실행합니다:
```bash
source ~/zenith_ws/install/setup.bash
ros2 run zav_control teleop_keyboard
```

#### 키 조작 매뉴얼:
| 키 | 기능 | 설명 |
|---|---|---|
| **`T`** | 자동 이륙 (Takeoff) | 시동(Arm) 후 고도 2.0m 칼호버링 상태로 대기 |
| **`L`** | 자동 착륙 (Land) | 현재 위치에서 수직 하강 후 지면 안착 시 시동 자동 꺼짐 |
| **`W / S`** | 전진 / 후진 | 기체 머리가 바라보는 방향 기준 1인칭 전진/후진 (0.2 m/s 단위) |
| **`A / D`** | 좌 / 우 이동 | 기체 머리 기준 좌측/우측 수평 이동 (0.2 m/s 단위) |
| **`Space / C`** | 상승 / 하강 | 고도 상승 / 하강 (0.2 m/s 단위) |
| **`Q / E`** | 좌회전 / 우회전 | Yaw 각속도 회전 (0.2 rad/s 단위) |
| **`X`** | 제자리 정지 | 모든 수평/수직 속도 0으로 급제동 및 위치 유지 |
