#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
================================================================================
ZAV Drone Simulation on Lightweight Ground Plane with PX4 SITL & Pegasus
================================================================================
설명:
  대용량 야외 맵(지형/텍스처) 로딩으로 인한 디스크 I/O 및 GPU/CPU 병목을 제거하고,
  Isaac Sim 내장 기본 평면(Default Ground Plane) 환경에서 커스텀 드론 ZAV (1.99 kg)를
  초경량·초고속(60+ FPS)으로 시뮬레이션 및 비행 제어하는 런처 스크립트입니다.

물리 제원:
  - 기체: Fusion 360 기반 커스텀 ZAV 쿼드콥터 (프레임, 가드, 카메라, 배터리 포함)
  - 총 질량: 1.99 kg
  - 무게중심(CoM): Z = -0.0182 m
  - 모터 축 간격: 반경 140.0 mm (Wheelbase 280.0 mm, dx = ±99.0 mm, dy = ±99.0 mm)
  - 통신: PX4 MAVLink Backend (UDP 14560 포트)
================================================================================
"""

from isaacsim import SimulationApp

# Isaac Sim GUI 앱 인스턴스 초기화 (단독 실행 모드)
simulation_app = SimulationApp({"headless": False})

import omni.timeline
from omni.isaac.core.world import World
import omni.isaac.core.utils.prims as prim_utils

# Pegasus Simulator 핵심 API
from pegasus.simulator.logic.interface.pegasus_interface import PegasusInterface
from pegasus.simulator.logic.vehicles.multirotors import ZAV_v1

class ZAVPlaneApp:
    def __init__(self):
        # 1. 시뮬레이션 타임라인 인터페이스 획득
        self.timeline = omni.timeline.get_timeline_interface()
        
        # 2. Pegasus 인터페이스 및 월드 초기화 (기본 물리 주기 250Hz, 렌더링 60Hz)
        self.pg = PegasusInterface()
        self.pg._world = World(**self.pg._world_settings)
        self.world = self.pg.world

        # 3. 초경량 기본 평면 바닥(Ground Plane) 및 완충 물리 물성 설정
        #    - 원리 (충돌 역학 / Rule 2 상세 주석):
        #      restitution=0.0 (완전 비탄성 충돌): 바닥 충돌 에너지를 흡수하여 착륙 시 기체가 위로 튀는 현상(Bouncing) 원천 방지
        #      static_friction=0.8, dynamic_friction=0.6: 접지 시 미끄러짐 방지 및 안정적인 그립 확보
        print("[Terrain] 충격 흡수 완충 평면(Damped Ground Plane) 생성 중...")
        self.world.scene.add_default_ground_plane(
            static_friction=0.8,
            dynamic_friction=0.6,
            restitution=0.0
        )

        # 시각적 가시성을 위한 직사광(Distant Light) 생성
        stage = self.world.stage
        if not stage.GetPrimAtPath("/World/defaultDistantLight"):
            prim_utils.create_prim(
                "/World/defaultDistantLight",
                "DistantLight",
                attributes={
                    "inputs:intensity": 3000.0,
                    "inputs:angle": 0.53,
                    "inputs:color": (1.0, 1.0, 1.0)
                }
            )

        # 4. ZAV 커스텀 쿼드콥터 스폰 (ZAV_v1 공식 프리셋 클래스 사용)
        #    - 내부적으로 ZAV_v1.usd 자동 바인딩
        #    - PX4 MAVLink Backend (UDP 14560) 자동 장착
        #    - PX4 북쪽(NED North) 정렬을 위한 Yaw=+90도 자동 적용
        print("[Spawn] ZAV_v1 프리셋으로 기체 생성 중...")
        ZAV_v1(stage_prefix="/World/quadrotor", vehicle_id=0, init_pos=[0.0, 0.0, 0.10])

        # 7. 물리 시뮬레이션 환경 리셋 (Articulation 및 관절 초기화)
        self.world.reset()
        print("=" * 60)
        print("[Ready] 초경량 완충 평면 시뮬레이션 준비 완료!")
        print("=" * 60)

    def run(self):
        """시뮬레이션 메인 루프 (물리 스텝 및 렌더링 갱신)"""
        self.timeline.play()
        while simulation_app.is_running():
            self.world.step(render=True)
        
        self.timeline.stop()
        simulation_app.close()

if __name__ == '__main__':
    app = ZAVPlaneApp()
    app.run()
