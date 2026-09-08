# [DevLog] 2026-09-07: FAST-LIVO2 센서 융합 안정화 및 치명적 메모리 누수 해결

- **작성자**: Zenith 드론 연구팀
- **플랫폼**: Radxa Dragon Q6A (Qualcomm QCS6490, 8-Core ARM64, 12GB LPDDR5)
- **센서 구성**: 
  - LiDAR: Livox Mid-360S (Ethernet UDP)
  - Camera: Hikrobot MV-CU013-A0UC (1.3MP Global Shutter, USB 3.0)
  - Lens: Hikrobot MV-0316 (3.16mm 고정 초점 렌즈)
- **주요 알고리즘**: FAST-LIVO2 (Direct LiDAR-Inertial-Visual Odometry)

---

## 1. 당일 주요 이슈 및 증상 요약

FAST-LIVO2와 카메라/라이다 드라이버를 결합하여 핸드헬드 3D 매핑을 시도하던 중 3가지 치명적인 장애가 발생함:
1. **극심한 시스템 멈춤 및 메모리 고갈 (11.3GB 100% 포화)**:
   - 노드 실행 30초~1분 만에 가용 램 11.3GB 중 10.4GB 이상이 잠식되며 마우스 커서가 굳고 전체 OS가 스왑 지옥에 빠짐.
2. **오도메트리 발산 ("바닥 꺼짐 현상")**:
   - 실행 직후 오도메트리 축이 아래로 곤두박질치며 궤적이 지하로 끝없이 추락함.
3. **ARM CPU 연산 과부하 및 프레임 랙**:
   - 카메라 영상이 수십 Hz로 무제한 쏟아져 들어와 보드 전체가 버벅거림.

---

## 2. 심층 원인 분석 (Root Cause Analysis)

### 원인 1. 카메라 드라이버의 치명적인 힙 메모리 누수 (150~300 MB/sec)
- **발견 위치**: `src/mvs_ros_driver2/src/grab_trigger.cc` 내부 `WorkThread` 루프
- **코드 결함**:
  ```cpp
  // 매 프레임마다 약 5.2MB를 힙(Heap)에 할당
  pDataForRGB = (unsigned char *)malloc(
      stImageInfo.stFrameInfo.nExtendWidth *
      stImageInfo.stFrameInfo.nExtendHeight * 4 + 2048);
  ...
  // 프레임 발행 후 free(pDataForRGB)를 전혀 호출하지 않음!
  ```
- **영향**:
  - 카메라가 30~60 FPS로 이미지를 뿜어내며 **초당 150MB~310MB의 메모리가 실시간으로 유출**됨.
  - 60초 만에 9.4GB의 메모리가 증발하여 11.3GB 램이 100% 가득 차고, 커널이 페이징/메모리 회수 루프에 빠져 CPU를 독점함.

### 원인 2. 카메라 프레임레이트 제한 부재와 ARM 프로세서 한계
- **코드 결함**: `MV_CC_SetBoolValue(handle, "AcquisitionFrameRateEnable", false);`로 하드코딩되어 프레임 제한이 해제된 상태였음.
- **논문 분석 (FAST-LIVO2, IEEE T-RO 2024)**:
  - HKU MaRS 연구실의 논문 실측 데이터에 따르면, **ARM 프로세서에서 FAST-LIVO2의 1프레임당 비선형 최적화(Direct Photometric Alignment) 연산 시간은 평균 78.44ms**임.
  - 10 Hz(주기 100ms)로 구동해야 78ms 연산 후 22ms의 안전 마진이 남아 실시간성이 보장됨.
  - 30~60 FPS로 이미지가 들어오면 33ms 주기 안에 78ms짜리 연산을 끝낼 수 없어 EKF 큐가 기하급수적으로 밀려 시스템 랙과 드리프트가 발생함.

### 원인 3. 기동 초기 IMU 정지 미흡 및 중력 벡터 오차 누적
- **메커니즘**:
  - FAST-LIVO2는 기동 직후 최초 N개 IMU 프레임(`imu_int_frame`) 동안 정지 상태의 가속도를 평균 내어 중력 벡터 $\mathbf{g} \approx [0, 0, -9.81]^T$를 추정함.
  - 초기 설정이 50프레임(200Hz 기준 0.25초)으로 너무 짧았고, 센서를 손에 쥐고 흔들리는 상태에서 켜면서 손의 선가속도가 중력 벡터에 혼입됨.
  - EKF 상태 적분 시 $(a_{\text{meas}} - g_{\text{est}})$ 오차가 시간에 따라 2차 함수($\iint \Delta a \, dt^2$)로 누적되어 오도메트리가 지하로 수직 추락함.
  - 추가로 `gravity_align_en: true`로 인해 초기 좌표계 회전 변환 급변이 발생함.

---

## 3. 엔지니어링 해결 및 최적화 내역

### 1) 카메라 메모리 무할당(Zero-allocation) 구조로 전면 리팩토링
- **파일**: [src/mvs_ros_driver2/src/grab_trigger.cc](file:///home/radxa/zenith_ws/src/mvs_ros_driver2/src/grab_trigger.cc)
- **개선 코드**:
  ```cpp
  // 스레드 시작 시 1회만 할당되는 단일 재사용 버퍼 생성
  std::vector<unsigned char> rgb_buffer;
  ...
  // 루프 내부: 용량이 부족할 때만 확장하고, 기존 버퍼 재사용
  size_t needed_size = (size_t)stImageInfo.stFrameInfo.nExtendWidth *
                       stImageInfo.stFrameInfo.nExtendHeight * 4 + 2048;
  if (rgb_buffer.size() < needed_size) {
    rgb_buffer.resize(needed_size);
  }
  pDataForRGB = rgb_buffer.data();
  stConvertParam.pDstBuffer = pDataForRGB;
  stConvertParam.nDstBufferSize = rgb_buffer.size();
  ```
- **결과**: **메모리 누수 0 Byte/s 달성!** 연속 스트리밍 중 프로세스 RSS 메모리가 **74MB**로 완벽 고정됨.

### 2) 10.0 Hz 프레임레이트 제한 파라미터 연동
- **파일**: `grab_trigger.cc` 및 [src/mvs_ros_driver2/config/zenith_camera.yaml](file:///home/radxa/zenith_ws/src/mvs_ros_driver2/config/zenith_camera.yaml)
- **내용**:
  - `setParams()` 함수에서 yaml의 `FrameRate` 값을 읽어 `AcquisitionFrameRateEnable: true` 및 `AcquisitionFrameRate: 10.0f` 적용.
  - 라이다(Mid-360) 10Hz와 정확히 1:1로 주기를 일치시켜 ARM CPU 사용률 대폭 절감.

### 3) 슬라이딩 윈도우 맵 (Sliding Window Voxel Map) 활성화
- **파일**: [src/FAST-LIVO2/config/zenith_mid360s.yaml](file:///home/radxa/zenith_ws/src/FAST-LIVO2/config/zenith_mid360s.yaml)
- **내용**:
  ```yaml
  local_map:
    map_sliding_en: true   # 슬라이딩 윈도우 활성화
    half_map_size: 50      # 복셀 0.5m 기준 반경 25m 큐브 유지
    sliding_thresh: 5      # 5m 이동할 때마다 오래된 복셀 메모리 해제
  ```
- **효과**: 장시간 비행/매핑 시에도 전체 복셀 맵 메모리가 일정 한도(약 2~3GB) 내에서 바운딩됨.

### 4) 중력 벡터 초기화 안정화
- **파일**: `zenith_mid360s.yaml`
- **내용**:
  - `imu_int_frame`: 50 $\rightarrow$ 100 (초기 0.5초간 충분한 평균 샘플 확보)
  - `gravity_align_en`: `false` (공식 기본값 복원)
  - 운용 수칙: 기동 초기 1~2초간 센서를 바닥에 평평하게 정지시키는 절차 확립.

---

## 4. 검증 결과 (Milestone Achieved)

1. **메모리 안정성**:
   - SLAM 풀 파이프라인 구동 시 전체 시스템 메모리가 **2.7GB 내외로 칼같이 고정**됨 (이전 11GB 폭증 완벽 해결).
2. **오도메트리 안정성**:
   - 지하 추락 현상 완전 소멸. 손으로 움직인 3D 곡선 궤적이 부드럽게 추종됨.
3. **RGB 컬러링 매핑**:
   - 복셀 포인트클라우드에 주변 사물의 실제 색상이 정상적으로 입혀지며 실내 구조가 3D 형태로 입체 형성됨을 RViz2에서 확인.

---

## 5. 다음 작업 계획 (Next Action Items)

1. **무선/배터리 단독 구동 & Headless 자동 실행 (더러럭)**:
   - HDMI와 유선 전원 제거 후 배터리 단독 구동.
   - 부팅 시 센서 및 매핑 노드가 자동 실행되고, 맵 파일(`.pcd`)과 궤적이 안전하게 백그라운드에 저장되는 서비스 구성 (복도 보행 스캔 준비).
2. **하드웨어 타임 동기화 (Temporal Sync)**:
   - 라이다: 유선 랜선 기반 PTP (IEEE 1588v2) 활성화
   - 카메라: 항공잭 2가닥 케이블을 통한 10Hz 하드웨어 셔터 펄스 트리거 (`TriggerEnable: 1`)
3. **카메라-라이다 캘리브레이션 (Spatial Sync)**:
   - 체커보드 기반 렌즈 왜곡($d_0, d_1$) 및 정밀 초점 거리 측정
   - 라이다-카메라 간의 6-DoF 외재적 파라미터($R_{cl}, P_{cl}$) 정밀 보정
