#!/usr/bin/env python3
import asyncio
from mavsdk import System

# =============================================================
# ⚙️ 사용자 파라미터 설정 (Parameter Settings)
# =============================================================
TARGET_ALTITUDE = 3.0  # 이륙 목표 고도 (미터 단위, m)
WAIT_SEC = 2.0         # 각 단계별 대기 시간 (초 단위, sec)
# =============================================================

async def run_drone_mission():
    drone = System()
    
    # MAVLink 포트 바인딩 충돌을 방지하는 클라이언트 접속 포맷 (udp://:14540)
    print("🚁 드론 가상 FC(PX4) MAVLink 하트비트 신호 연결 중...")
    await drone.connect(system_address="udp://:14540")

    async for state in drone.core.connection_state():
        if state.is_connected:
            print("✅ 가상 FC(PX4) 연결 성공 및 하트비트 수신 완료!")
            break

    # Step 1: 시동 걸기 (Arming)
    print("▶ [1단계] 시동 걸기 (Arming)...")
    await drone.action.arm()

    # Step 2: 2초 대기
    print(f"⏳ {WAIT_SEC}초간 대기 중...")
    await asyncio.sleep(WAIT_SEC)

    # Step 3: 이륙 고도 파라미터 설정 및 이륙 (Takeoff)
    print(f"▶ [2단계] 이륙 시작 (목표 고도: {TARGET_ALTITUDE}m)...")
    await drone.action.set_takeoff_altitude(TARGET_ALTITUDE)
    await drone.action.takeoff()

    # 이륙 상승 시간 포함 8초 대기 후 정점에서 2초 대기
    await asyncio.sleep(8.0)
    print(f"▶ [3단계] 정상 고도({TARGET_ALTITUDE}m) 도달! {WAIT_SEC}초간 대기...")
    await asyncio.sleep(WAIT_SEC)

    # Step 4: 착륙 (Land)
    print("▶ [4단계] 착륙 시작 (Landing)...")
    await drone.action.land()

    # 착륙 하강 시간 대기
    await asyncio.sleep(8.0)
    print(f"▶ [5단계] 착륙 완료! {WAIT_SEC}초 대기 후 시동 끄기...")
    await asyncio.sleep(WAIT_SEC)

    # Step 5: 시동 끄기 (Disarming)
    print("▶ [6단계] 시동 끄기 (Disarming)...")
    await drone.action.disarm()
    print("🎉 [완료] 시동 끄기 및 시나리오 미션 성공적으로 완수!")

if __name__ == '__main__':
    asyncio.run(run_drone_mission())
