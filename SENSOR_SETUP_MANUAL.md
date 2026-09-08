# Zenith 드론 3D 스캔 모듈 센서 세팅 매뉴얼 (Sensor Bring-up Runbook)

이 문서는 **Radxa Dragon Q6A** 보드 기반의 3D 공간 스캔 및 자율비행 드론 개발 환경에서 **카메라(Hikrobot MV-CU013)**와 **라이다(Livox Mid-360S)**를 밑바닥(Zero)부터 직접 연결하고 ROS 2 드라이버를 구축하는 전 과정을 기록한 엔지니어링 가이드입니다.

---

## 1. 하드웨어 아키텍처 및 4단계 센서 공식

모든 로보틱스 센서는 아래 **4계층 구조**를 순서대로 거쳐 가동됩니다.

```text
┌─────────────────────────────────────────────────────────────┐
│ [4계층: ROS 2 통역사]  토픽 발행 (/camera/image_raw, /livox/lidar) │
├─────────────────────────────────────────────────────────────┤
│ [3계층: 제조사 SDK]    MVS SDK (C++ .so), Livox-SDK2        │
├─────────────────────────────────────────────────────────────┤
│ [2계층: OS 통신/권한]  USB udev 규칙, 유선 LAN 고정 IP (192.168.1.50) │
├─────────────────────────────────────────────────────────────┤
│ [1계층: 물리/전원 결선] USB 3.0 (5V), 외부 독립 DC 전원 (12V)  │
└─────────────────────────────────────────────────────────────┘
```

---

## 2. 하이크로봇 카메라 세팅 가이드 (Hikrobot MV-CU013-A0UC)

### 2.1 하드웨어 및 렌즈 특징
* **센서**: 1.3MP 글로벌 셔터 (Global Shutter) - 고속 비행 시 젤로(Jello) 왜곡 없음
* **렌즈**: MV-0316 3.16mm 고정 초점 수동 렌즈
  * ⚠️ **주의**: 자율비행 SLAM용 카메라는 오토포커스를 절대 쓰면 안 됩니다 (초점 거리 $f_x, f_y$가 변하면 캘리브레이션이 깨짐). 약 1.5m~2m 거리에 초점을 맞추고 조리개를 조인 뒤 고정 나사를 잠급니다.
* **전원/통신**: USB 3.0 케이블 (보드의 5V로 전원 및 5Gbps 데이터 동시 처리)

### 2.2 OS 인식 및 udev 권한 설정 (호스트)
일반 사용자가 루트 권한 없이 USB 카메라에 고속 접근할 수 있도록 규칙 등록:
```bash
# 카메라 인식 확인
lsusb | grep -i hik

# 권한 규칙 등록 (MVS SDK 설치 시 자동 구성됨)
# /etc/udev/rules.d/88-hikrobot-usb.rules
```

### 2.3 MVS SDK 설치
제조사 공식 Linux ARM64 SDK 설치:
```bash
sudo dpkg -i MVS-5.0.2_aarch64_20260728.deb
# 설치 위치: /opt/MVS
```

### 2.4 ROS 2 드라이버 구동
* **패키지**: `mvs_ros_driver2`
* **설정 파일**: `src/mvs_ros_driver2/config/zenith_camera.yaml`
  * `Topic_Name`: `/camera/image_raw`
  * `TriggerEnable`: `0` (자체 연속 촬영 모드)
* **실행 명령어**:
  ```bash
  ros2 launch mvs_ros_driver2 zenith_camera.launch.py
  ```
* **검증**: `ros2 topic hz /camera/image_raw` (약 30 Hz 확인)

---

## 3. 리복스 라이다 세팅 가이드 (Livox Mid-360S)

### 3.1 하드웨어 및 전원 결선
* **센서**: Livox Mid-360S (비반복 스캐닝 360° x 59° FoV, 고정밀 6축 IMU 내장)
* **전원**: **독립 DC 12V 2~3A 전원 필수** (보드 랜포트는 PoE가 아니므로 반드시 파워선을 배터리/파워서플라이에 직결)
* **통신**: RJ-45 유선 랜선을 Radxa 유선 랜포트(`enp1s0`)에 직결

### 3.2 네트워크 IP 설정 (호스트 보드)
라이다는 자체 IP를 가진 독립 네트워크 장비입니다.

1. **라이다 기본 IP 확인 (14자리 Broadcast Code)**:
   * 센서 라벨의 QR 코드/시리얼 확인 (예: `ARMCP710031873`)
   * 끝 2자리 숫자(`73`) 확인 $\rightarrow$ 라이다 기본 IP는 **`192.168.1.173`**
2. **보드 유선 랜포트에 고정 IP 할당**:
   ```bash
   # 유선 연결 프로필에 192.168.1.50/24 고정 할당
   nmcli connection modify "Wired connection 1" connection.interface-name enp1s0 ipv4.method manual ipv4.addresses 192.168.1.50/24
   nmcli connection up "Wired connection 1"
   ```
3. **물리 통신 핑(Ping) 테스트**:
   ```bash
   ping -c 3 192.168.1.173
   # 0% packet loss 및 1ms 미만 지연시간 확인
   ```

### 3.3 Livox-SDK2 라이브러리 (도커 내 사전 설치됨)
* 경로: `/usr/local/lib/liblivox_lidar_sdk_shared.so`
* 헤더: `/usr/local/include/livox_lidar_api.h`, `livox_lidar_def.h`

### 3.4 ROS 2 드라이버 (`livox_ros_driver2`)
* **패키지 위치**: `src/livox_ros_driver2`
* **ROS 2 모드 전환**:
  ```bash
  cp src/livox_ros_driver2/package_ROS2.xml src/livox_ros_driver2/package.xml
  ```
* **설정 파일 (`config/MID360s_config.json`)**:
  ```json
  {
    "lidar_summary_info" : { "lidar_type": 8 },
    "Mid360s": {
      "host_net_info" : [
        {
          "host_ip"        : "192.168.1.50",
          "cmd_data_port"  : 56101,
          "push_msg_port"  : 56201,
          "point_data_port": 56301,
          "imu_data_port"  : 56401,
          "log_data_port"  : 56501
        }
      ]
    },
    "lidar_configs" : [
      {
        "ip" : "192.168.1.173",
        "pcl_data_type" : 1,
        "pattern_mode" : 0
      }
    ]
  }
  ```
* **빌드**:
  ```bash
  colcon build --symlink-install --packages-select livox_ros_driver2 --cmake-args -DROS_EDITION=ROS2 -DDISTRO_ROS=humble
  ```
* **실행 명령어**:
  ```bash
  ros2 launch livox_ros_driver2 msg_MID360s_launch.py
  ```
* **검증**:
  * `ros2 topic hz /livox/lidar` $\rightarrow$ **10.0 Hz**
  * `ros2 topic hz /livox/imu` $\rightarrow$ **200.0 Hz**

---

## 4. 네트워크 FAQ 및 시스템 동작 원리

* **Q. 야외에서 인터넷이 없으면 작동 안 하나요?**
  * **A**: 완벽하게 100% 작동합니다. 이더넷 랜선은 라이다와 보드 둘만 잇는 1:1 폐쇄 고속도로(LAN)이며, 외부 인터넷과는 완전히 무관합니다.
* **Q. 나중에 LTE 모뎀이나 지상 GCS 통신을 붙이면 라이다 IP가 충돌하지 않나요?**
  * **A**: 충돌하지 않습니다. 리눅스는 포트마다 IP를 따로 관리(다중 인터페이스)하므로, 라이다는 유선 랜포트(`enp1s0`, `192.168.1.50`), 인터넷은 Wi-Fi(`wlan0`, `192.168.0.x`), 원격 제어는 LTE(`wwan0`)로 완전히 분리되어 공존합니다.

---

## 5. 최종 데이터 스트림 확인 요약

| 센서 | 프로토콜 | ROS 2 토픽 | 메시지 규격 | 정상 주기 | 비고 |
| :--- | :--- | :--- | :--- | :--- | :--- |
| **카메라** | USB 3.0 | `/camera/image_raw` | `sensor_msgs/msg/Image` | **10.0 Hz** | 640x512 다운샘플, 메모리 무할당 패치 완료 |
| **라이다 점군** | Ethernet UDP | `/livox/lidar` | `livox_ros_driver2/msg/CustomMsg` | **10.0 Hz** | Mid-360S 스캔 주기 |
| **라이다 IMU** | Ethernet UDP | `/livox/imu` | `sensor_msgs/msg/Imu` | **200.0 Hz** | 내장 IMU 고주파수 계측 |

---

## 6. FAST-LIVO2 구동 가이드

1. **카메라 노드**:
   ```bash
   ros2 launch mvs_ros2_driver zenith_camera.launch.py
   ```
2. **라이다 노드**:
   ```bash
   ros2 launch livox_ros_driver2 msg_MID360s_launch.py
   ```
3. **FAST-LIVO2 매핑 노드**:
   > ⚠️ **필수 주의**: 매핑 노드 기동 시 센서 모듈을 **책상이나 바닥에 완전히 평평하게 올려놓고 손을 뗀 상태**에서 실행해야 합니다. 최초 1~2초간 IMU 중력 벡터($g$) 초기화가 완료된 후 손으로 들어 올려 스캔합니다.
   ```bash
   ros2 launch fast_livo zenith_mapping.launch.py
   ```
