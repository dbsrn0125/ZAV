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

## 🚀 빠른 시작 (Quick Start)

### 1. 원클릭 환경 설정
이 저장소를 클론한 후 아래 스크립트를 실행합니다:
```bash
cd ~/zenith_ws/simulation
./setup_sim.sh
```
> **자동 수행 작업**:
> - PX4 에어프레임(`10020_isaac_zav`) 등록 및 CMakeLists.txt 추가
> - Pegasus 기체 모델(`zav_v1.py`), 3D USD 모델(`ZAV_v1.usd`), 런처(`run_zav_plane.py`) 복사
> - `~/.bashrc`에 단축어 `zav_sim` 등록
> - ROS 2 `px4_msgs`, `zav_control` 패키지 빌드

### 2. 시뮬레이션 실행
터미널을 새로 열거나 `source ~/.bashrc` 후 실행합니다:
```bash
zav_sim
```

### 3. 키보드로 조종하기
새 터미널 창을 열고 FPV 키보드 조종 노드를 실행합니다:
```bash
cd ~/zenith_ws
source install/setup.bash
ros2 run zav_control teleop_keyboard
```

#### 키 조작 매뉴얼:
- **`T`**: 자동 이륙 (2.0m 칼호버링)
- **`L`**: 자동 착륙 및 모터 정지
- **`W / S`**: 전진 / 후진 (1인칭 기체 기준)
- **`A / D`**: 좌측 / 우측 이동 (1인칭 기체 기준)
- **`Space / C`**: 고도 상승 / 하강
- **`Q / E`**: 좌회전 / 우회전 (Yaw 조작)
- **`X`**: 제자리 정지
