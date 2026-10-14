#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
==============================================================================
    ZAV Flight Manager (Bridge Node & Safety Gateway)
==============================================================================
[시스템 아키텍처 및 제어 원리 설명]

1. 역할 분담 원칙 (Role Separation):
   - 본 노드는 비행 물리(제동, 감속 곡선, 호버링 위치 제어 등)에 직접 개입하지 않습니다.
   - 실제 비행 역학(자세 제어, 감속 브레이크, 제자리 호버링)은 PX4 Autopilot 내부의
     5단계 캐스케이드 제어기(mc_pos_control, mc_att_control 등)가 100% 전담합니다.
   - 본 노드의 역할은 오직 다음 2가지로 한정됩니다:
     ① [RC 조종기 스틱 모사 (Input Shaping)]:
        - 물리적 스프링과 댐핑이 있는 실제 RC 조종기 짐벌(Gimbal)과 달리,
          키보드는 0/1로 즉각 변하는 불연속 디지털 스위치입니다.
        - 키를 누를 때는 손가락으로 스틱을 미는 동작(Ramp Up: 5.0 m/s^2)을 모사하고,
        - 키에서 손을 뗄 때는 센터링 스프링(Centering Spring)이 스틱을 중앙(0)으로
          부드럽게 당겨 복귀시키는 동작(Spring Return: 7.0 m/s^2)을 모사합니다.
        - 이를 통해 PX4 제어기에 무한 저크(Jerk 충격) 없이 부드러운 연속 속도 지령을 공급합니다.
     ② [안전 게이트웨이 (Safety Switch & Mode Gateway)]:
        - 1번(MANUAL_POS): 키보드 스틱 지령을 PX4 Velocity Trajectory로 전달.
        - 3번(OFFBOARD_TRAJ): 상위 경로 플래너(SUPER, Fast-Planner 등)의 /zav/cmd_vel 전달.
          * 자율비행 중 사용자의 수동 키보드 입력은 안전을 위해 자동 차단(Lock).
        - 4번(OFFBOARD_RATE): 고속 기동 제어기(OMMPC 등)의 /zav/cmd_rate 전달.
        - Space / E 키: 언제든 비상 디스암 및 모터 전원 차단 최우선 개입.

2. 3차원 좌표계 회전 변환 (FLU Body -> NED World):
   - 기체 고유 좌표계 (Body FLU):
     * Front (+X): 전진
     * Left  (+Y): 좌측
     * Up    (+Z): 상승
   - 항법 좌표계 (World NED):
     * North (+X), East (+Y), Down (+Z)
   - 기체의 현재 헤딩 요 각도를 psi (rad)라 할 때의 변환 수식:
     ┌ vn ┐   ┌ cos(psi)  -sin(psi)   0 ┐ ┌ vx_body ┐
     │ ve │ = │ sin(psi)   cos(psi)   0 │ │ vy_body │
     └ vd ┘   └    0          0      -1 ┘ └ vz_body ┘
     * vn = vx * cos(psi) - vy * sin(psi)
     * ve = vx * sin(psi) + vy * cos(psi)
     * vd = -vz (Body 상향 속도가 월드 하향 속도의 역수)

3. PX4 통신 및 페일세이프 방지 규격:
   - 속도 제어 모드일 때는 OffboardControlMode.position=False, velocity=True로 선언.
   - TrajectorySetpoint의 position=[NaN, NaN, NaN], velocity=[vn, ve, vd] 설정.
   - 불필요한 yaw/yawspeed 동시 입력으로 인한 'invalid setpoints' 경고 원천 차단.
   - E-Stop 시 시뮬레이션 소켓을 파괴하는 TERMINATION 대신 강제 디스암(21196.0) 사용.
==============================================================================
"""

import os
import sys
import base64
import termios
import tty
import select
import time
import math
import subprocess
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

# ROS 2 표준 메시지
from geometry_msgs.msg import Twist

# PX4 uORB 메시지 인터페이스
from px4_msgs.msg import (
    OffboardControlMode,
    TrajectorySetpoint,
    VehicleRatesSetpoint,
    VehicleCommand,
    VehicleStatus,
    VehicleLocalPosition,
)

# 비트마스크 상수 정의
KEY_UP    = 1 << 0  # 1   (전진 ↑)
KEY_DOWN  = 1 << 1  # 2   (후진 ↓)
KEY_LEFT  = 1 << 2  # 4   (좌측 ←)
KEY_RIGHT = 1 << 3  # 8   (우측 →)
KEY_W     = 1 << 4  # 16  (상승 W)
KEY_S     = 1 << 5  # 32  (하강 S)
KEY_A     = 1 << 6  # 64  (좌회전 A)
KEY_D     = 1 << 7  # 128 (우회전 D)
KEY_SPACE = 1 << 8  # 256 (Spacebar)


def wrap_pi(angle: float) -> float:
    """각도를 [-pi, pi] 범위로 정규화하는 함수"""
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


def ramp_stick_value(current: float, target: float, push_rate: float, return_rate: float, dt: float) -> float:
    """
    [조종기 짐벌 스틱 물리 모사 필터 (Stick Input Shaping)]
    - 스틱을 밀 때(push): 조종사의 손가락 반응 속도(push_rate)로 선형 증가
    - 스틱을 놓을 때(return): 센터링 스프링의 복원력(return_rate)으로 0을 향해 선형 감소
    """
    if abs(target) < 1e-5 or (current * target < 0.0):
        rate = return_rate
    else:
        rate = push_rate

    step = rate * dt
    if current < target:
        return min(target, current + step)
    elif current > target:
        return max(target, current - step)
    return target


class Win32MultiKeyReader:
    """
    WSL2 Linux 환경에서 Windows 호스트의 GetAsyncKeyState API를 호출하여
    여러 키의 동시 입력 및 실시간 눌림 상태를 20ms 주기로 읽어오는 초고속 비동기 리더
    """
    def __init__(self):
        self.key_mask = 0
        self.active = False
        self._proc = None
        self._pipe_fd = None
        self._recv_buf = b""

        ps_exe = "/mnt/c/Windows/System32/WindowsPowerShell/v1.0/powershell.exe"
        if os.path.exists(ps_exe):
            ps_code = (
                "Add-Type -TypeDefinition '"
                "using System; using System.Runtime.InteropServices; "
                "public class KS { [DllImport(\"user32.dll\")] public static extern short GetAsyncKeyState(int vKey); }"
                "'; "
                "while ($true) { "
                "$m = 0; "
                "if (([int][KS]::GetAsyncKeyState(0x26) -band 0x8000) -ne 0) { $m = $m -bor 1 }; "
                "if (([int][KS]::GetAsyncKeyState(0x28) -band 0x8000) -ne 0) { $m = $m -bor 2 }; "
                "if (([int][KS]::GetAsyncKeyState(0x25) -band 0x8000) -ne 0) { $m = $m -bor 4 }; "
                "if (([int][KS]::GetAsyncKeyState(0x27) -band 0x8000) -ne 0) { $m = $m -bor 8 }; "
                "if (([int][KS]::GetAsyncKeyState(0x57) -band 0x8000) -ne 0) { $m = $m -bor 16 }; "
                "if (([int][KS]::GetAsyncKeyState(0x53) -band 0x8000) -ne 0) { $m = $m -bor 32 }; "
                "if (([int][KS]::GetAsyncKeyState(0x41) -band 0x8000) -ne 0) { $m = $m -bor 64 }; "
                "if (([int][KS]::GetAsyncKeyState(0x44) -band 0x8000) -ne 0) { $m = $m -bor 128 }; "
                "if (([int][KS]::GetAsyncKeyState(0x20) -band 0x8000) -ne 0) { $m = $m -bor 256 }; "
                "[Console]::Out.WriteLine($m); "
                "[Console]::Out.Flush(); "
                "Start-Sleep -Milliseconds 20 "
                "}"
            )
            encoded = base64.b64encode(ps_code.encode('utf-16le')).decode('ascii')
            try:
                self._proc = subprocess.Popen(
                    [ps_exe, "-NoProfile", "-NonInteractive", "-EncodedCommand", encoded],
                    stdin=subprocess.DEVNULL,
                    stdout=subprocess.PIPE,
                    stderr=subprocess.DEVNULL,
                    bufsize=0
                )
                self._pipe_fd = self._proc.stdout.fileno()
                os.set_blocking(self._pipe_fd, False)
            except Exception:
                self.active = False

    def poll_latest(self) -> int:
        if self._pipe_fd is None:
            return 0
        try:
            while True:
                data = os.read(self._pipe_fd, 4096)
                if not data:
                    break
                self._recv_buf += data
        except BlockingIOError:
            pass
        except Exception:
            return self.key_mask

        if b"\n" in self._recv_buf:
            lines = self._recv_buf.split(b"\n")
            self._recv_buf = lines[-1]
            for raw_line in reversed(lines[:-1]):
                s = raw_line.strip()
                if s.isdigit():
                    self.key_mask = int(s)
                    self.active = True
                    break
        return self.key_mask

    def stop(self):
        if self._proc:
            try:
                self._proc.kill()
            except Exception:
                pass


def read_key_nonblocking(fd: int) -> str:
    """단일 문자 및 단축키(1, 3, 4, L, R, E 등) 수신용 논블로킹 키보드 리더"""
    rlist, _, _ = select.select([fd], [], [], 0.005)
    if not rlist:
        return ''
    try:
        raw_bytes = os.read(fd, 32)
        return raw_bytes.decode('utf-8', errors='ignore')
    except Exception:
        return ''


class ZAVFlightManager(Node):
    def __init__(self):
        super().__init__('zav_flight_manager')

        # PX4 uORB 토픽 통신 QoS 설정
        qos_pub = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        qos_sub = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # 1. PX4 제어 인터페이스 퍼블리셔
        self.cmd_pub = self.create_publisher(VehicleCommand, '/fmu/in/vehicle_command', qos_pub)
        self.offboard_mode_pub = self.create_publisher(OffboardControlMode, '/fmu/in/offboard_control_mode', qos_pub)
        self.traj_setpoint_pub = self.create_publisher(TrajectorySetpoint, '/fmu/in/trajectory_setpoint', qos_pub)
        self.rate_setpoint_pub = self.create_publisher(VehicleRatesSetpoint, '/fmu/in/vehicle_rates_setpoint', qos_pub)

        # 2. PX4 상태 피드백 구독자
        self.create_subscription(VehicleStatus, '/fmu/out/vehicle_status', self._on_vehicle_status, qos_sub)
        self.create_subscription(VehicleLocalPosition, '/fmu/out/vehicle_local_position', self._on_local_position, qos_sub)

        # 3. 상위 자율비행 제어기 입력 인터페이스 (SUPER / Fast-LIO / OMMPC 연동)
        self.create_subscription(Twist, '/zav/cmd_vel', self._on_external_cmd_vel, 10)
        self.create_subscription(Twist, '/zav/cmd_rate', self._on_external_cmd_rate, 10)

        # 4. 비행 상태 관리 변수
        self.is_armed = False
        self.nav_state = 0
        self.flight_mode = "MANUAL_POS"  # 기본: 키보드 수동 위치/호버링 모드

        # 기체 센서 피드백 저장
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_z_ned = 0.0
        self.current_alt_enu = 0.0
        self.current_yaw = 0.0
        self.yaw_initialized = False
        self.target_yaw = 0.0

        # 자동 이륙 상태 관리
        self.is_auto_takeoff = False
        self.auto_takeoff_start_time = 0.0
        self.takeoff_hover_x = 0.0
        self.takeoff_hover_y = 0.0

        # 5. 가상 RC 스틱 사양 (Stick Dynamics Limits)
        self.max_speed_xy = 8.0        # 최대 수평 비행 속도 (m/s)
        self.max_speed_z_up = 3.0      # 최대 상승 속도 (m/s)
        self.max_speed_z_dn = 1.5      # 최대 하강 속도 (m/s)
        self.max_yaw_rate = 2.0        # 최대 회전 각속도 (rad/s)

        # 스틱 모사 가속도:
        # 손가락으로 스틱을 미는 속도 (Push rate)
        self.stick_push_xy = 5.0       # m/s^2 (약 0.3초만에 최고속도 도달)
        self.stick_push_z = 2.5        # m/s^2
        self.stick_push_yaw = 3.0      # rad/s^2

        # 손을 뗐을 때 센터링 스프링이 스틱을 중앙(0)으로 당기는 속도 (Spring Return rate)
        self.stick_return_xy = 7.0     # m/s^2 (약 0.15~0.2초만에 스틱 0 복귀)
        self.stick_return_z = 4.0      # m/s^2
        self.stick_return_yaw = 5.0    # rad/s^2

        # 실시간 가상 스틱 목표치 (키를 누르면 max_speed, 떼면 0.0)
        self.target_vx_body = 0.0
        self.target_vy_body = 0.0
        self.target_vz_body = 0.0
        self.target_yaw_rate = 0.0

        # 스프링 모사를 거친 실제 출력 속도 지령 (Continuous Output)
        self.out_vx_body = 0.0
        self.out_vy_body = 0.0
        self.out_vz_body = 0.0
        self.out_yaw_rate = 0.0

        # 외부 상위 명령 수신 버퍼
        self.ext_cmd_vel = None
        self.last_ext_vel_time = 0.0
        self.ext_cmd_rate = None
        self.last_ext_rate_time = 0.0

        # 입력 디바이스 초기화
        self.multi_key = Win32MultiKeyReader()
        self.space_latched = False
        self.last_loop_time = time.time()
        self.last_print_time = time.time()

        # 50Hz (20ms) 고속 정주기 제어 루프 타이머
        self.timer = self.create_timer(0.02, self._control_loop)

        print("\n" + "=" * 78)
        print(" 🎮 ZAV Flight Manager 가동 완료 (Clean Input Gateway Architecture)")
        print(" ----------------------------------------------------------------------------")
        print(" [단축키 안내]")
        print("   - [Spacebar] : 시동 켜기/끄기 (Arm / Disarm 토글)")
        print("   - [T]        : 자동 2.0m 이륙 (Auto Takeoff)")
        print("   - [1]        : 수동 위치 모드 (MANUAL_POS - 키 떼면 PX4 자동 호버링)")
        print("   - [2]        : 수동 고도 모드 (ALTCTL - 고도 유지 & 수평 관성 비행)")
        print("   - [3]        : 외부 자율 속도 모드 (OFFBOARD_TRAJ - 키보드 이동 차단 ⛔)")
        print("   - [4]        : 외부 자율 각속도 모드 (OFFBOARD_RATE - OMMPC 등 연동 ⛔)")
        print("   - [L]        : 자동 착륙 (Land)")
        print("   - [R]        : 홈 복귀 (Return-To-Launch)")
        print("   - [E]        : 비상 모터 강제 전원 차단 (Emergency Kill)")
        print("   - [Q / Ctrl+C]: 노드 안전 종료")
        print(" ----------------------------------------------------------------------------")
        print(" [수동 조종키 안내 (1번 및 2번 수동 모드에서 작동)]")
        print("   - [ ↑ / ↓ ]  : 전진(Forward) / 후진(Backward)")
        print("   - [ ← / → ]  : 좌측이동(Left) / 우측이동(Right)")
        print("   - [ W / S ]  : 상승(Up) / 하강(Down)")
        print("   - [ A / D ]  : 좌회전(Turn Left) / 우회전(Turn Right)")
        print("=" * 78 + "\n")

    def _on_vehicle_status(self, msg: VehicleStatus):
        self.is_armed = (msg.arming_state == VehicleStatus.ARMING_STATE_ARMED)
        self.nav_state = msg.nav_state

    def _on_local_position(self, msg: VehicleLocalPosition):
        self.current_x = msg.x
        self.current_y = msg.y
        self.current_z_ned = msg.z
        self.current_alt_enu = -msg.z
        self.current_yaw = msg.heading
        if not self.yaw_initialized:
            self.target_yaw = msg.heading
            self.yaw_initialized = True

    def _on_external_cmd_vel(self, msg: Twist):
        self.ext_cmd_vel = msg
        self.last_ext_vel_time = time.time()

    def _on_external_cmd_rate(self, msg: Twist):
        self.ext_cmd_rate = msg
        self.last_ext_rate_time = time.time()

    def send_vehicle_command(self, command: int, p1=0.0, p2=0.0, p3=0.0, p4=0.0, p5=0.0, p6=0.0, p7=0.0):
        cmd = VehicleCommand()
        cmd.timestamp = self.get_clock().now().nanoseconds // 1000
        cmd.param1 = float(p1)
        cmd.param2 = float(p2)
        cmd.param3 = float(p3)
        cmd.param4 = float(p4)
        cmd.param5 = float(p5)
        cmd.param6 = float(p6)
        cmd.param7 = float(p7)
        cmd.command = command
        cmd.target_system = 1
        cmd.target_component = 1
        cmd.source_system = 1
        cmd.source_component = 1
        cmd.from_external = True
        self.cmd_pub.publish(cmd)

    def set_arm_toggle(self):
        new_arm = not self.is_armed
        self.send_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0 if new_arm else 0.0)
        if new_arm:
            # 시동을 걸 때 PX4를 OFFBOARD 모드(1.0, 6.0)로 즉시 전환하여 W키 이륙 및 조종 보장
            time.sleep(0.05)
            self.send_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0)
        action_name = "시동 켜기(ARM) & OFFBOARD 모드 진입" if new_arm else "시동 끄기(DISARM)"
        print(f"\r\n👉 [명령 전송] {action_name} 완료!\r\n")

    def set_mode_manual(self):
        self.flight_mode = "MANUAL_POS"
        print("\r\n🎯 [모드 변경] 수동 위치 모드 (MANUAL_POS: 키 떼면 PX4 자동 호버링 정지)!\r\n")

    def set_mode_altctl(self):
        self.flight_mode = "ALTCTL"
        print("\r\n🎈 [모드 변경] 수동 고도 모드 (ALTCTL: 고도 고정 & 수평 관성 비행)!\r\n")

    def set_mode_offboard_traj(self):
        self.flight_mode = "OFFBOARD_TRAJ"
        print("\r\n🚀 [모드 변경] Offboard Trajectory Mode (상위 플래너 자율비행, 키보드 이동 완전 차단 ⛔)!\r\n")

    def set_mode_offboard_rate(self):
        self.flight_mode = "OFFBOARD_RATE"
        print("\r\n⚡ [모드 변경] Offboard Rate Mode (OMMPC 고속 각속도 자율비행, 키보드 이동 완전 차단 ⛔)!\r\n")

    def set_land(self):
        self.send_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
        self.flight_mode = "LAND"
        print("\r\n🛬 [명령] 자동 착륙(Land) 시작...\r\n")

    def set_rtl(self):
        self.send_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_RETURN_TO_LAUNCH)
        self.flight_mode = "RTL"
        print("\r\n🏠 [명령] 홈 복귀(RTL) 시작...\r\n")

    def set_emergency_kill(self):
        # SITL 통신을 파괴하는 FLIGHTTERMINATION 대신 강제 Disarm 매직코드(21196.0)를 사용하여 모터 즉시 차단
        self.send_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0, 21196.0)
        self.is_armed = False
        print("\r\n🚨🚨 [EMERGENCY STOP] 모터 전원이 강제 차단되었습니다! (Forced Disarmed) 🚨🚨\r\n")

    def handle_key_command(self, ch: str):
        if ch == ' ':
            self.set_arm_toggle()
        elif ch in ('t', 'T'):
            print("\r\n🚀 [자동이륙] 목표 고도 2.0m 자동 수직 상승 시작!\r\n")
            self.is_auto_takeoff = True
            self.auto_takeoff_start_time = time.time()
            self.takeoff_hover_x = self.current_x
            self.takeoff_hover_y = self.current_y
            self.target_yaw = self.current_yaw
            self.send_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)
            time.sleep(0.05)
            self.send_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0)
        elif ch in ('l', 'L'):
            self.set_land()
        elif ch in ('r', 'R'):
            self.set_rtl()
        elif ch in ('e', 'E'):
            self.set_emergency_kill()
        elif ch == '1':
            self.set_mode_manual()
        elif ch == '2':
            self.set_mode_altctl()
        elif ch == '3':
            self.set_mode_offboard_traj()
        elif ch == '4':
            self.set_mode_offboard_rate()
        elif ch in ('q', 'Q', '\x03'):
            return False
        return True

    def _update_stick_inputs(self):
        """키보드 입력을 가상 조종기 스틱의 목표치(Target)로 변환"""
        mask = self.multi_key.poll_latest()

        # Spacebar 시동 토글 (엣지 트리거)
        if mask & KEY_SPACE:
            if not self.space_latched:
                self.space_latched = True
                self.set_arm_toggle()
        else:
            self.space_latched = False

        # 자율비행 모드(3번, 4번)일 때는 안전을 위해 수동 조종키를 완전 차단!
        if self.flight_mode in ("OFFBOARD_TRAJ", "OFFBOARD_RATE"):
            self.target_vx_body = 0.0
            self.target_vy_body = 0.0
            self.target_vz_body = 0.0
            self.target_yaw_rate = 0.0
            return

        # 1. 전후 축 (Up / Down) -> Body X (전진: +vx, 후진: -vx)
        if (mask & KEY_UP) and not (mask & KEY_DOWN):
            self.target_vx_body = self.max_speed_xy
        elif (mask & KEY_DOWN) and not (mask & KEY_UP):
            self.target_vx_body = -self.max_speed_xy
        else:
            self.target_vx_body = 0.0

        # 2. 좌우 축 (Left / Right) -> Body Y (좌측: +vy, 우측: -vy)
        if (mask & KEY_LEFT) and not (mask & KEY_RIGHT):
            self.target_vy_body = self.max_speed_xy
        elif (mask & KEY_RIGHT) and not (mask & KEY_LEFT):
            self.target_vy_body = -self.max_speed_xy
        else:
            self.target_vy_body = 0.0

        # 3. 수직 스로틀 축 (W / S) -> Body Z (상승: +vz, 하강: -vz)
        if (mask & KEY_W) and not (mask & KEY_S):
            self.target_vz_body = self.max_speed_z_up
            # 시동 상태에서 오프보드가 아니면 W를 누르는 순간 즉시 오프보드로 전환
            if self.is_armed and self.nav_state != 14:
                self.send_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0)
        elif (mask & KEY_S) and not (mask & KEY_W):
            self.target_vz_body = -self.max_speed_z_dn
        else:
            self.target_vz_body = 0.0

        # 4. Yaw 회전 축 (A / D) -> CCW 좌회전: +yaw_rate, CW 우회전: -yaw_rate
        if (mask & KEY_A) and not (mask & KEY_D):
            self.target_yaw_rate = self.max_yaw_rate
        elif (mask & KEY_D) and not (mask & KEY_A):
            self.target_yaw_rate = -self.max_yaw_rate
        else:
            self.target_yaw_rate = 0.0

    def _publish_offboard_heartbeat(self, velocity_mode=True, body_rate_mode=False):
        """PX4 OffboardControlMode 하트비트 전송 (속도 제어 모드 정확 선언)"""
        mode = OffboardControlMode()
        mode.timestamp = self.get_clock().now().nanoseconds // 1000
        mode.position = False
        mode.velocity = velocity_mode
        mode.acceleration = False
        mode.attitude = False
        mode.body_rate = body_rate_mode
        self.offboard_mode_pub.publish(mode)

    def _control_loop(self):
        now = time.time()
        dt = min(0.1, max(0.005, now - self.last_loop_time))
        self.last_loop_time = now

        # 키보드 입력 읽기
        self._update_stick_inputs()

        # [케이스 A] 자동 2m 이륙 시퀀스 (T 키)
        if self.is_auto_takeoff:
            if self.current_alt_enu >= 1.8 or (now - self.auto_takeoff_start_time > 8.0):
                print(f"\r\n✨ 자동 2.0m 이륙 완료! (현재 고도: {self.current_alt_enu:.2f}m) -> 수동/자율 모드 조종 가능!\r\n")
                self.is_auto_takeoff = False
                self.set_mode_manual()
            else:
                # 이륙 중에는 목표 고도로 상승 속도 인가
                self._publish_offboard_heartbeat(velocity_mode=True, body_rate_mode=False)
                sp = TrajectorySetpoint()
                sp.timestamp = self.get_clock().now().nanoseconds // 1000
                sp.position = [float('nan'), float('nan'), float('nan')]
                sp.velocity = [0.0, 0.0, -1.2]  # NED 기준 상향 1.2 m/s
                sp.acceleration = [float('nan'), float('nan'), float('nan')]
                sp.yaw = float(self.target_yaw) if self.yaw_initialized else float('nan')
                sp.yawspeed = float('nan')
                self.traj_setpoint_pub.publish(sp)
                return

        # [케이스 B] 모드 4: OFFBOARD_RATE 자율비행 (OMMPC 연동)
        if self.flight_mode == "OFFBOARD_RATE":
            self._publish_offboard_heartbeat(velocity_mode=False, body_rate_mode=True)
            rate_sp = VehicleRatesSetpoint()
            rate_sp.timestamp = self.get_clock().now().nanoseconds // 1000

            if self.ext_cmd_rate and (now - self.last_ext_rate_time < 0.3):
                rate_sp.roll = float(self.ext_cmd_rate.angular.x)
                rate_sp.pitch = float(self.ext_cmd_rate.angular.y)
                rate_sp.yaw = float(self.ext_cmd_rate.angular.z)
                thrust_val = max(0.0, min(0.8, self.ext_cmd_rate.linear.z))
                rate_sp.thrust_body[2] = float(-thrust_val)
            else:
                rate_sp.roll = 0.0
                rate_sp.pitch = 0.0
                rate_sp.yaw = 0.0
                rate_sp.thrust_body[2] = -0.24  # 호버링 기본 추력

            self.rate_setpoint_pub.publish(rate_sp)
            return

        # [케이스 C] 모드 3: OFFBOARD_TRAJ 자율비행 (SUPER / Fast-Planner 연동)
        if self.flight_mode == "OFFBOARD_TRAJ":
            self._publish_offboard_heartbeat(velocity_mode=True, body_rate_mode=False)
            sp = TrajectorySetpoint()
            sp.timestamp = self.get_clock().now().nanoseconds // 1000

            if self.ext_cmd_vel and (now - self.last_ext_vel_time < 0.3):
                vx = self.ext_cmd_vel.linear.x
                vy = self.ext_cmd_vel.linear.y
                vz = self.ext_cmd_vel.linear.z
                yaw_rate = self.ext_cmd_vel.angular.z

                # FLU Body -> NED World 변환
                psi = self.current_yaw
                vn = vx * math.cos(psi) - vy * math.sin(psi)
                ve = vx * math.sin(psi) + vy * math.cos(psi)
                vd = -vz

                sp.position = [float('nan'), float('nan'), float('nan')]
                sp.velocity = [float(vn), float(ve), float(vd)]
                sp.acceleration = [float('nan'), float('nan'), float('nan')]
                sp.yaw = float('nan')
                sp.yawspeed = float(yaw_rate)
            else:
                sp.position = [float('nan'), float('nan'), float('nan')]
                sp.velocity = [0.0, 0.0, 0.0]
                sp.acceleration = [float('nan'), float('nan'), float('nan')]
                sp.yaw = float('nan')
                sp.yawspeed = 0.0

            self.traj_setpoint_pub.publish(sp)

        # [케이스 D] 모드 1: MANUAL_POS (키보드 수동 조종 - 스틱 모사) 및 모드 2: ALTCTL
        else:
            self._publish_offboard_heartbeat(velocity_mode=True, body_rate_mode=False)

            # 1. 스틱 스프링 복귀 필터 적용 (Input Shaping)
            # 모드 1(MANUAL_POS): return_rate=7.0 m/s^2 (키 떼면 즉시 0 복귀 -> PX4 제동 및 호버링)
            # 모드 2(ALTCTL): return_rate=0.35 m/s^2 (키 떼도 스르륵 미끄러지며 관성 비행 유지)
            cur_return_xy = 0.35 if self.flight_mode == "ALTCTL" else self.stick_return_xy

            self.out_vx_body = ramp_stick_value(self.out_vx_body, self.target_vx_body, self.stick_push_xy, cur_return_xy, dt)
            self.out_vy_body = ramp_stick_value(self.out_vy_body, self.target_vy_body, self.stick_push_xy, cur_return_xy, dt)
            self.out_vz_body = ramp_stick_value(self.out_vz_body, self.target_vz_body, self.stick_push_z, self.stick_return_z, dt)
            self.out_yaw_rate = ramp_stick_value(self.out_yaw_rate, self.target_yaw_rate, self.stick_push_yaw, self.stick_return_yaw, dt)

            # 수평 최대 속도 벡터 크기 제한
            vel_xy_norm = math.hypot(self.out_vx_body, self.out_vy_body)
            if vel_xy_norm > self.max_speed_xy:
                scale = self.max_speed_xy / vel_xy_norm
                vx_eff = self.out_vx_body * scale
                vy_eff = self.out_vy_body * scale
            else:
                vx_eff = self.out_vx_body
                vy_eff = self.out_vy_body

            # 2. 3차원 좌표계 회전 변환 (기존 검증된 teleop_keyboard.py 수식으로 100% 원복)
            # 수식:
            # vn = vx * cos(psi) + vy * sin(psi)
            # ve = vx * sin(psi) - vy * cos(psi)
            # vd = -vz
            psi = self.current_yaw
            vn = vx_eff * math.cos(psi) + vy_eff * math.sin(psi)
            ve = vx_eff * math.sin(psi) - vy_eff * math.cos(psi)
            vd = -self.out_vz_body  # 상승(+vz)은 NED에서 음수(-vd)

            # 3. PX4 TrajectorySetpoint 생성 및 퍼블리시
            sp = TrajectorySetpoint()
            sp.timestamp = self.get_clock().now().nanoseconds // 1000
            sp.position = [float('nan'), float('nan'), float('nan')]
            sp.velocity = [float(vn), float(ve), float(vd)]
            sp.acceleration = [float('nan'), float('nan'), float('nan')]

            # 회전 키를 누르고 있을 때는 yawspeed 지정, 뗐을 때는 헤딩 잠금
            if abs(self.out_yaw_rate) > 0.02:
                sp.yaw = float('nan')
                sp.yawspeed = float(self.out_yaw_rate)
                self.target_yaw = self.current_yaw
            else:
                sp.yaw = float(self.target_yaw) if self.yaw_initialized else float('nan')
                sp.yawspeed = float('nan')

            self.traj_setpoint_pub.publish(sp)

        # 4. 실시간 상태 단일 라인 갱신 출력 (\033[K)
        if now - self.last_print_time > 0.09:
            self.last_print_time = now
            arm_str = "🟢 시동(Armed)" if self.is_armed else "🔴 정지(Disarmed)"
            mode_str = self.flight_mode
            v_xy = math.hypot(self.out_vx_body, self.out_vy_body)
            sys.stdout.write(
                f"\r\033[K[{arm_str}] [{mode_str:13s}] "
                f"고도:{self.current_alt_enu:5.2f}m | "
                f"스틱속도:{v_xy:4.1f}m/s | "
                f"헤딩:{math.degrees(self.current_yaw):+6.1f}°"
            )
            sys.stdout.flush()

    def destroy_node(self):
        self.multi_key.stop()
        super().destroy_node()


def main(args=None):
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    rclpy.init(args=args)
    node = ZAVFlightManager()

    try:
        tty.setraw(fd, termios.TCSANOW)
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.005)
            ch = read_key_nonblocking(fd)
            if ch != '':
                if not node.handle_key_command(ch):
                    break
    except KeyboardInterrupt:
        pass
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        node.destroy_node()
        rclpy.shutdown()
        print("\n\n✅ ZAV Flight Manager가 안전하게 종료되었습니다.")


if __name__ == '__main__':
    main()
