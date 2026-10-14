#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
==============================================================================
ZAV Quadrotor Central Flight Manager & Coordinate Bridge Node
==============================================================================
패키지: zav_control
노드명: zav_flight_manager
설명:
  1. 비행 모드 관리자 (Flight Mode Manager):
     - [수동 모드 1: POSCTL, 2: ALTCTL]: Offboard를 끄고 정식 조종기 스틱(ManualControlSetpoint)으로 수동 비행!
       * 2번 ALTCTL 선택 시 PX4 수평 위치 제어가 완전히 꺼져 바람/관성대로 흐름!
     - [자율 모드 3: OFFBOARD_TRAJ, 4: OFFBOARD_RATE]:
       * 사용자의 이동 키(WASD, 화살표) 100% 차단! 오직 상위 플래너 토픽만 통과.
       * 비상/모드 전환 키(1, 2, Space, L, R, E)만 즉시 개입 가능 (Safety Interlock).
  2. 실시간 3차원 좌표계 변환 브릿지 (Coordinate Transformation Bridge):
     - 상위 플래너(SUPER, 텔레옵)의 ROS 2 표준 ENU/FLU 좌표계를 PX4 항공 표준 NED/FRD로 실시간 변환.
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
    ManualControlSetpoint,
    VehicleCommand,
    VehicleStatus,
    VehicleLocalPosition,
)

# 비트마스크 상수 정의 (무지연 다중 키보드 동시 입력용)
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
    """각도를 [-pi, +pi] 범위로 정규화"""
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


def ramp_value(current: float, target: float, accel_rate: float, decel_rate: float, dt: float) -> float:
    """스틱 선형 가감속 (Ramp) 필터"""
    if abs(target) < 1e-5 or (current * target < 0.0):
        rate = decel_rate
    else:
        rate = accel_rate

    step = rate * dt
    if current < target:
        return min(target, current + step)
    elif current > target:
        return max(target, current - step)
    return target


class Win32MultiKeyReader:
    """WSL2 환경에서 Windows GetAsyncKeyState를 통해 다중 키 상태를 초고속 논블로킹 수신"""
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

    def poll_latest(self):
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

        # 1. PX4 통신 퍼블리셔 (Micro-XRCE-DDS)
        self.cmd_pub = self.create_publisher(VehicleCommand, '/fmu/in/vehicle_command', qos_pub)
        self.offboard_mode_pub = self.create_publisher(OffboardControlMode, '/fmu/in/offboard_control_mode', qos_pub)
        self.traj_setpoint_pub = self.create_publisher(TrajectorySetpoint, '/fmu/in/trajectory_setpoint', qos_pub)
        self.rate_setpoint_pub = self.create_publisher(VehicleRatesSetpoint, '/fmu/in/vehicle_rates_setpoint', qos_pub)
        self.manual_pub = self.create_publisher(ManualControlSetpoint, '/fmu/in/manual_control_setpoint', qos_pub)

        # 2. PX4 상태 구독자
        self.create_subscription(VehicleStatus, '/fmu/out/vehicle_status', self._on_vehicle_status, qos_sub)
        self.create_subscription(VehicleLocalPosition, '/fmu/out/vehicle_local_position', self._on_local_position, qos_sub)

        # 3. 상위 자율비행 제어기 입력 인터페이스
        self.create_subscription(Twist, '/zav/cmd_vel', self._on_external_cmd_vel, 10)
        self.create_subscription(Twist, '/zav/cmd_rate', self._on_external_cmd_rate, 10)

        # 4. 비행 상태 관리 변수
        self.is_armed = False
        self.nav_state = 0
        self.flight_mode = "POSCTL"  # 기본 시작 모드: POSCTL (수동 위치 모드)

        # 센서 피드백
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_z_ned = 0.0
        self.current_alt_enu = 0.0
        self.current_yaw = 0.0
        self.yaw_initialized = False

        self.target_yaw = 0.0
        self.hold_x = 0.0
        self.hold_y = 0.0

        # 속도 및 가감속 스펙
        self.max_speed_xy = 12.0
        self.max_speed_z_up = 3.0
        self.max_speed_z_dn = 1.5
        self.max_yaw_rate = 2.5

        self.accel_xy = 5.0
        self.decel_xy = 14.0
        self.accel_z = 2.5
        self.decel_z = 6.0
        self.accel_yaw = 2.5
        self.decel_yaw = 8.0

        # 실시간 속도 명령 (Body FLU 기준)
        self.cmd_vx_body = 0.0
        self.cmd_vy_body = 0.0
        self.cmd_vz_body = 0.0
        self.cmd_yaw_rate = 0.0

        self.des_vx_body = 0.0
        self.des_vy_body = 0.0
        self.des_vz_body = 0.0
        self.des_yaw_rate = 0.0

        # 외부 상위 명령 수신 버퍼
        self.ext_cmd_vel = None
        self.last_ext_vel_time = 0.0

        self.ext_cmd_rate = None
        self.last_ext_rate_time = 0.0

        # 자동 2m 이륙(T) 모드 상태 플래그
        self.is_auto_takeoff = False
        self.auto_takeoff_start_time = 0.0

        # 다중 키 감지기 시작
        self.multi_key = Win32MultiKeyReader()

        # 주기적 제어 루프 타이머 (50Hz = 20ms 주기)
        self.last_loop_time = time.time()
        self.last_print_time = time.time()
        self.timer = self.create_timer(0.02, self._control_loop)

        self._print_banner()

    def _print_banner(self):
        print("""
==============================================================================
   🚁 [ZAV Quadrotor Central Flight Manager & Coordinate Bridge] 🚁
==============================================================================
  [비행 시동 및 비상 명령]
    Spacebar : 【시동(Arm) / 시동끄기(Disarm) 토글】 (비행 중: 비상 급제동 호버링)
    W        : 【수동 스로틀 상승 / 이륙】 (시동 켠 뒤 W 누르면 부드럽게 지면 박차고 상승!)
    S        : 【수동 스로틀 하강】 (손 떼면 현재 고도에서 딱 고정 호버링)
    L        : 【자동 착륙 (Land)】 (서서히 수직 착륙 후 모터 자동 셧다운)
    R        : 【홈 복귀 후 착륙 (RTL)】 (이륙 지점으로 자동 복귀)
    E        : 🚨【EMERGENCY KILL】 (모터 즉시 강제 전원 차단 / 비상 추락)

  [비행 모드 원클릭 스위칭]
    [1] Position Mode  : GPS 기반 수동 제자리 호버링 모드 (키보드 조종 O)
    [2] Altitude Mode  : 고도 유지 & 관성/바람 따라 흐르는 수동 모드 (키보드 조종 O)
    [3] Offboard Traj  : 상위 플래너(/zav/cmd_vel) 자율비행 (키보드 이동 완전차단 ⛔)
    [4] Offboard Rate  : 미래 OMMPC(/zav/cmd_rate) 고속 자율비행 (키보드 이동 완전차단 ⛔)

  [수동 방향 조종 (1번, 2번 모드에서만 동작)]
    ↑ / ↓    : 전진 / 후진 (선형 가감속)
    ← / →    : 좌측 / 우측 이동 (방향 정확 일치)
    A / D    : 좌회전 / 우회전 (Yaw Rate)
    ★ T 키   : (옵션) 원버튼 자동 2.0m 수직 이륙
==============================================================================
[준비 완료] Spacebar를 눌러 시동(Arm)을 켜고, W 키를 눌러 이륙하세요!
""")

    def _on_vehicle_status(self, msg: VehicleStatus):
        self.is_armed = (msg.arming_state == VehicleStatus.ARMING_STATE_ARMED)
        self.nav_state = msg.nav_state

    def _on_local_position(self, msg: VehicleLocalPosition):
        self.current_x = msg.x
        self.current_y = msg.y
        self.current_z_ned = msg.z
        self.current_alt_enu = -msg.z

        if not self.yaw_initialized and not math.isnan(msg.heading):
            self.current_yaw = msg.heading
            self.target_yaw = msg.heading
            self.yaw_initialized = True
        elif not math.isnan(msg.heading):
            self.current_yaw = msg.heading

    def _on_external_cmd_vel(self, msg: Twist):
        self.ext_cmd_vel = msg
        self.last_ext_vel_time = time.time()

    def _on_external_cmd_rate(self, msg: Twist):
        self.ext_cmd_rate = msg
        self.last_ext_rate_time = time.time()

    def send_vehicle_command(self, command, param1=0.0, param2=0.0, param3=0.0, param7=0.0):
        """PX4 시스템 명령 uORB 송신"""
        cmd = VehicleCommand()
        cmd.command = command
        cmd.param1 = float(param1)
        cmd.param2 = float(param2)
        cmd.param3 = float(param3)
        cmd.param7 = float(param7)
        cmd.target_system = 1
        cmd.target_component = 1
        cmd.source_system = 1
        cmd.source_component = 1
        cmd.from_external = True
        cmd.timestamp = 0
        self.cmd_pub.publish(cmd)

    def set_arm_toggle(self):
        """Spacebar 시동 토글 (지면: 시동 On/Off, 공중: 브레이크)"""
        if self.current_alt_enu < 0.35:
            if not self.is_armed:
                print("\r\n⚡ [시동] 시동(Arm) 인가 중... 모터 아이들링 시작!\r\n")
                # 시동 걸기
                self.send_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)
                time.sleep(0.05)
                # 현재 모드가 자율이 아니면 기본 POSCTL 유지
                if self.flight_mode not in ("OFFBOARD_TRAJ", "OFFBOARD_RATE", "ALTCTL"):
                    self.set_mode_posctl()
            else:
                print("\r\n🛑 [시동] 모터 시동 끄기(Disarm) 완료!\r\n")
                self.send_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0)
                self.flight_mode = "DISARMED"
        else:
            print("\r\n🛑 [급제동] 즉시 제자리 수평 호버링 브레이크!\r\n")
            self.des_vx_body = self.cmd_vx_body = 0.0
            self.des_vy_body = self.cmd_vy_body = 0.0
            self.des_vz_body = self.cmd_vz_body = 0.0
            self.des_yaw_rate = self.cmd_yaw_rate = 0.0
            self.set_mode_posctl()

    def set_mode_posctl(self):
        """Position Mode (PX4 POSCTL) 전환 - Offboard Mute"""
        self.send_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 3.0)  # 3 = POSCTL
        self.flight_mode = "POSCTL"
        print("\r\n🎯 [모드 변경] Position Mode (GPS 제자리 호버링 & 키보드 수동 조종) 활성화!\r\n")

    def set_mode_altctl(self):
        """Altitude Mode (PX4 ALTCTL) 전환 - Offboard Mute & 수평제어 OFF"""
        self.send_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 2.0)  # 2 = ALTCTL
        self.flight_mode = "ALTCTL"
        print("\r\n🎈 [모드 변경] Altitude Mode (고도 유지 & 수평 위치제어 OFF 관성 흐름) 활성화!\r\n")

    def set_mode_offboard_traj(self):
        """Offboard Trajectory Mode (상위 플래너 자율비행 - 키보드 이동 차단)"""
        self.send_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0)  # 6 = OFFBOARD
        self.flight_mode = "OFFBOARD_TRAJ"
        print("\r\n🚀 [모드 변경] Offboard Trajectory Mode (상위 플래너 자율비행, 키보드 이동 완전 차단 ⛔)!\r\n")

    def set_mode_offboard_rate(self):
        """Offboard Rate Mode (OMMPC 고속 각속도 자율비행 - 키보드 이동 차단)"""
        self.send_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0)
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
        self.send_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_FLIGHTTERMINATION, 1.0)
        print("\r\n🚨🚨 [EMERGENCY KILL] 모터 전원이 강제 차단되었습니다! 🚨🚨\r\n")

    def handle_key_command(self, ch: str):
        """모드 전환 및 비상 단축키 처리 (어느 모드에서나 항상 즉시 반응)"""
        if ch == ' ':
            self.set_arm_toggle()
        elif ch in ('t', 'T'):
            print("\r\n🚀 [자동이륙] 목표 고도 2.0m 자동 수직 상승 시작!\r\n")
            self.is_auto_takeoff = True
            self.auto_takeoff_start_time = time.time()
            self.hold_x = self.current_x
            self.hold_y = self.current_y
            self.target_yaw = self.current_yaw
            self.send_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)
            time.sleep(0.05)
            self.send_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0)
            self.flight_mode = "OFFBOARD_TRAJ"
        elif ch in ('l', 'L'):
            self.set_land()
        elif ch in ('r', 'R'):
            self.set_rtl()
        elif ch in ('e', 'E'):
            self.set_emergency_kill()
        elif ch == '1':
            self.set_mode_posctl()
        elif ch == '2':
            self.set_mode_altctl()
        elif ch == '3':
            self.set_mode_offboard_traj()
        elif ch == '4':
            self.set_mode_offboard_rate()

    def _update_stick_inputs(self):
        """수동 모드(1, 2)일 때만 키보드 이동 입력 반영 (3, 4번 자율 모드에서는 완전 차단!)"""
        if self.flight_mode in ("OFFBOARD_TRAJ", "OFFBOARD_RATE"):
            # 자율비행 중에는 사람 손가락 입력 완전 무시!
            self.des_vx_body = 0.0
            self.des_vy_body = 0.0
            self.des_vz_body = 0.0
            self.des_yaw_rate = 0.0
            return

        mask = self.multi_key.poll_latest()

        # 전후 축 (Up / Down)
        if (mask & KEY_UP) and not (mask & KEY_DOWN):
            self.des_vx_body = self.max_speed_xy
        elif (mask & KEY_DOWN) and not (mask & KEY_UP):
            self.des_vx_body = -self.max_speed_xy
        else:
            self.des_vx_body = 0.0

        # 좌우 축 (Left / Right)
        if (mask & KEY_LEFT) and not (mask & KEY_RIGHT):
            self.des_vy_body = self.max_speed_xy
        elif (mask & KEY_RIGHT) and not (mask & KEY_LEFT):
            self.des_vy_body = -self.max_speed_xy
        else:
            self.des_vy_body = 0.0

        # 수직 스로틀 축 (W / S)
        if (mask & KEY_W) and not (mask & KEY_S):
            self.des_vz_body = self.max_speed_z_up
        elif (mask & KEY_S) and not (mask & KEY_W):
            self.des_vz_body = -self.max_speed_z_dn
        else:
            self.des_vz_body = 0.0

        # Yaw 회전 축 (A / D)
        if (mask & KEY_A) and not (mask & KEY_D):
            self.des_yaw_rate = -self.max_yaw_rate
        elif (mask & KEY_D) and not (mask & KEY_A):
            self.des_yaw_rate = self.max_yaw_rate
        else:
            self.des_yaw_rate = 0.0

    def _publish_offboard_heartbeat(self, pos_vel=True, body_rate=False):
        mode = OffboardControlMode()
        mode.position = pos_vel
        mode.velocity = pos_vel
        mode.acceleration = False
        mode.attitude = False
        mode.body_rate = body_rate
        mode.timestamp = 0
        self.offboard_mode_pub.publish(mode)

    def _control_loop(self):
        now = time.time()
        dt = min(0.1, max(0.005, now - self.last_loop_time))
        self.last_loop_time = now

        # 키보드 이동 입력 갱신
        self._update_stick_inputs()

        # -------------------------------------------------------------
        # 1. 자동 2m 이륙 시퀀스 (T 키)
        # -------------------------------------------------------------
        if self.is_auto_takeoff:
            if self.current_alt_enu >= 1.8 or (now - self.auto_takeoff_start_time > 8.0):
                print(f"\r\n✨ 자동 2.0m 이륙 완료! (현재 고도: {self.current_alt_enu:.2f}m) -> 수동/자율 모드 조종 가능!\r\n")
                self.is_auto_takeoff = False
                self.set_mode_posctl()
            else:
                self._publish_offboard_heartbeat(pos_vel=True, body_rate=False)
                sp = TrajectorySetpoint()
                sp.timestamp = 0
                sp.position = [float(self.hold_x), float(self.hold_y), -2.0]
                sp.velocity = [0.0, 0.0, -1.5]
                sp.yaw = float(self.target_yaw) if self.yaw_initialized else float('nan')
                self.traj_setpoint_pub.publish(sp)
                return

        # -------------------------------------------------------------
        # 2. [모드 1 & 2: 수동 비행] POSCTL / ALTCTL
        # -------------------------------------------------------------
        if self.flight_mode in ("POSCTL", "ALTCTL"):
            # ★ 핵심: Offboard 토픽을 완전히 차단(Mute)하여 PX4 본래의 수동 제어기 동작!
            # 조종사 키보드 입력을 PX4 정식 조종기 스틱(ManualControlSetpoint)으로 직접 인가!
            man_sp = ManualControlSetpoint()
            man_sp.timestamp = 0
            man_sp.valid = True
            man_sp.data_source = 2  # MAVLink / 외부 제어기 입력

            # 스틱 가감속 적용
            self.cmd_vx_body = ramp_value(self.cmd_vx_body, self.des_vx_body, self.accel_xy, self.decel_xy, dt)
            self.cmd_vy_body = ramp_value(self.cmd_vy_body, self.des_vy_body, self.accel_xy, self.decel_xy, dt)
            self.cmd_vz_body = ramp_value(self.cmd_vz_body, self.des_vz_body, self.accel_z, self.decel_z, dt)
            self.cmd_yaw_rate = ramp_value(self.cmd_yaw_rate, self.des_yaw_rate, self.accel_yaw, self.decel_yaw, dt)

            # 정규화된 스틱 값 (-1.0 ~ +1.0)
            man_sp.pitch = float(self.cmd_vx_body / self.max_speed_xy)    # 전후 (피치)
            man_sp.roll  = float(-self.cmd_vy_body / self.max_speed_xy)   # 좌우 (롤)
            man_sp.yaw   = float(self.cmd_yaw_rate / self.max_yaw_rate)   # 요 회전

            # 스로틀: 중립(0.5) 기준, W 누르면 상승(>0.5), S 누르면 하강(<0.5)
            if self.cmd_vz_body > 0.05:
                throttle_norm = 0.5 + 0.5 * (self.cmd_vz_body / self.max_speed_z_up)
            elif self.cmd_vz_body < -0.05:
                throttle_norm = 0.5 - 0.5 * (abs(self.cmd_vz_body) / self.max_speed_z_dn)
            else:
                throttle_norm = 0.5  # 고도 유지 (Hover)

            man_sp.throttle = float(max(0.0, min(1.0, throttle_norm)))
            man_sp.sticks_moving = (abs(man_sp.pitch) > 0.02 or abs(man_sp.roll) > 0.02 or
                                    abs(man_sp.throttle - 0.5) > 0.02 or abs(man_sp.yaw) > 0.02)
            self.manual_pub.publish(man_sp)

        # -------------------------------------------------------------
        # 3. [모드 3: OFFBOARD_TRAJ 자율비행] 상위 플래너 명령만 전달 (키보드 이동 차단)
        # -------------------------------------------------------------
        elif self.flight_mode == "OFFBOARD_TRAJ":
            self._publish_offboard_heartbeat(pos_vel=True, body_rate=False)
            sp = TrajectorySetpoint()
            sp.timestamp = 0

            # 상위 플래너(/zav/cmd_vel)가 0.3초 이내에 명령을 주면 추종, 없으면 제자리 호버링
            if self.ext_cmd_vel and (now - self.last_ext_vel_time < 0.3):
                vx = self.ext_cmd_vel.linear.x
                vy = self.ext_cmd_vel.linear.y
                vz = self.ext_cmd_vel.linear.z
                yaw_rate = self.ext_cmd_vel.angular.z

                # FLU -> NED 좌표 회전
                psi = self.current_yaw
                vn = vx * math.cos(psi) + vy * math.sin(psi)
                ve = vx * math.sin(psi) - vy * math.cos(psi)
                vd = -vz

                sp.position = [float('nan'), float('nan'), float('nan')]
                sp.velocity = [float(vn), float(ve), float(vd)]
                sp.yawspeed = float(yaw_rate)
            else:
                # 상위 플래너 명령 없을 시: 안전한 제자리 호버링 유지
                sp.position = [float('nan'), float('nan'), float('nan')]
                sp.velocity = [0.0, 0.0, 0.0]

            self.traj_setpoint_pub.publish(sp)

        # -------------------------------------------------------------
        # 4. [모드 4: OFFBOARD_RATE 자율비행] OMMPC 각속도/추력 제어 (과추력 방지)
        # -------------------------------------------------------------
        elif self.flight_mode == "OFFBOARD_RATE":
            self._publish_offboard_heartbeat(pos_vel=False, body_rate=True)
            rate_sp = VehicleRatesSetpoint()
            rate_sp.timestamp = 0

            if self.ext_cmd_rate and (now - self.last_ext_rate_time < 0.3):
                rate_sp.roll = float(self.ext_cmd_rate.angular.x)
                rate_sp.pitch = float(self.ext_cmd_rate.angular.y)
                rate_sp.yaw = float(self.ext_cmd_rate.angular.z)
                # 정규화된 추력 클램핑 (0.0 ~ 0.8)
                thrust_val = max(0.0, min(0.8, self.ext_cmd_rate.linear.z))
                rate_sp.thrust_body[2] = float(-thrust_val)
            else:
                # OMMPC 미실행 시: 과추력 솟구침 원천 차단! (ZAV 호버 추력 24% 유지)
                rate_sp.roll = 0.0
                rate_sp.pitch = 0.0
                rate_sp.yaw = 0.0
                rate_sp.thrust_body[2] = -0.24

            self.rate_setpoint_pub.publish(rate_sp)

        # -------------------------------------------------------------
        # 5. 실시간 상태 단일 라인 갱신 출력 (\033[K)
        # -------------------------------------------------------------
        if now - self.last_print_time > 0.09:
            self.last_print_time = now
            arm_str = "🟢 시동(Armed)" if self.is_armed else "🔴 정지(Disarmed)"
            mode_str = self.flight_mode
            v_xy = math.hypot(self.cmd_vx_body, self.cmd_vy_body)
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
                if ch in ('\x03', '\x1a'):  # Ctrl+C or Ctrl+Z
                    break
                node.handle_key_command(ch)
    except KeyboardInterrupt:
        pass
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        node.destroy_node()
        rclpy.shutdown()
        print("\n\n✅ ZAV Flight Manager가 안전하게 종료되었습니다.")


if __name__ == '__main__':
    main()
