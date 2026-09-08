# Zenith 드론 연구 개발 일지 (Engineering DevLogs)

이 디렉토리는 Zenith 드론 프로젝트의 모든 시스템 엔지니어링, 센서 드라이버 구축, SLAM 알고리즘 최적화, 비행 제어 및 트러블슈팅 내역을 일자별로 상세히 기록하는 연구 일지 아카이브입니다.

---

## 📅 일자별 개발 로그 목차

| 날짜 | 문서 링크 | 핵심 작업 및 주요 성과 |
| :--- | :--- | :--- |
| **2026-09-07** | [2026-09-07_fast_livo2_bringup_and_optimization.md](file:///home/radxa/zenith_ws/devlog/2026-09-07_fast_livo2_bringup_and_optimization.md) | **FAST-LIVO2 핸드헬드 3D 매핑 최초 성공**<br>- 카메라 드라이버 치명적 메모리 누수(150MB/s) 0B로 완전 해결<br>- 카메라 10Hz 프레임레이트 제한(ARM 실시간성 확보)<br>- 슬라이딩 윈도우 복셀 맵 활성화로 RAM 2.7GB 안정화<br>- 오도메트리 바닥 추락 버그 원인 분석 및 해결 |
| **2026-09-08 (예정)** | *2026-09-08_hardware_sync_and_headless_scan.md* | - 무선/배터리 단독 구동 원클릭(더러럭) 자동 실행 환경 구축<br>- 라이다 PTPv2 + 카메라 10Hz 하드웨어 트리거 동기화<br>- 카메라-라이다 캘리브레이션 툴 구동 (복도 매핑) |
