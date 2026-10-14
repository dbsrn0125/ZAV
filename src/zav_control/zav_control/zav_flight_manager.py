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
     - 조종기 스위치 및 키보드 단축키를 통해 PX4의 모든 핵심 모드 및 비상 안전 명령을 총괄 제어합니다.
     - 시동(Spacebar), 수동 스로틀 이륙(W/S), 자동착륙(L), 홈복귀(R), 긴급킬(E), 모드전환(1~4).
  2. 실시간 3차원 좌표계 변환 브릿지 (Coordinate Transformation Bridge):
     - 상위 플래너(SUPER, 텔레옵)의 ROS 2 표준 ENU/FLU 좌표계를 PX4 항공 표준 NED/FRD로 실시간 변환.
  3. 상위 자율비행 인터페이스 (Swappable Planner / OMMPC Interface):
     - 모드 3: 상위 플래너(/zav/cmd_vel) -> PX4 TrajectorySetpoint (Level 4/3 제어)
     - 모드 4: 고속 제어기(/zav/cmd_rate) -> PX4 VehicleRatesSetpoint (Level 1 모터 제어)

[수학적 원리 및 좌표 변환 공식 (Rule 2 상세 주석)]:
1. 기체 바디(Body FLU) -> 월드 관성계(Inertial NED) 속도 회전 변환:
   - 기체의 전방 방향 헤딩각 psi (Yaw, rad) 기준:
     v_North =  v_x * cos(psi) - v_y * sin(psi)
     v_East  =  v_x * sin(psi) + v_y * cos(psi)
     v_Down  = -v_z  (하강이 +Z인 NED 특성 반영, 상승 시 음수)

2. 대각선 속도 벡터 원형 정규화 (Circular Velocity Gate):
   - 전후(vx), 좌우(vy) 동시 입력 시 유클리드 노름 norm = sqrt(vx^2 + vy^2)이
     최대 수평 속도(MPC_XY_VEL_MAX = 12.0 m/s)를 초과하지 않도록 축소 스케일링.
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

# 비트마스크 상수 정의 (무지연 다중 키보드 동시 입력용)
KEY_UP    = 1 << 0  # 1   (전진 ↑)
KEY_DOWN  = 1 << 1  # 2   (후진 ↓)
KEY_LEFT  = 1 << 2  # 4   (좌측 ←)
KEY_RIGHT = 1 << 3  # 8   (우측 →)
KEY_W     = 1 << 4  # 16  (상승 W)
KEY_S     = 1 << 5  # 32  (하강 S)
KEY_A     = 1 << 6  # 64  (좌회전 A)
KEY_D     = 1 << 7  # 128 (우회전 D)
KEY_SPACE = 1 << 8  # 256 (시동 Arm/Disarm 토글 or 비상 브레이크)


def wrap_pi(angle: float) -> float:
    """각도를 [-pi, +pi] 범위로 정규화"""
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


def ramp_value(current: float, target: float, accel: float, decel: float, dt: float) -> float:
    """스틱 선형 가감속 (Ramp) 필터: 부드러운 가속 및 즉각적인 감속"""
    if abs(target) > abs(current):
        rate = accel
    else:
        rate = decel

    step = rate * dt
    if target > current:
        return min(target, current + step)
    elif target < current:
        return max(target, current - step)
    return current


class MultiKeyPoller:
    """
    Windows GetAsyncKeyState 기반 초고속 무지연 다중 키 감지 백엔드.
    버퍼 밀림(Buffer Bloat)을 원천 방지하여 키에서 손을 떼는 즉시 0.01초 내 감속 복귀.
    """
    def __init__(self):
        self.active = False
        self.key_mask = 0
        self.proc = None
        self._pipe_fd = None

        is_wsl = False
        try:
            with open("/proc/version", "r") as f:
                if "microsoft" in f.read().lower():
                    is_wsl = True
        except Exception:
            pass

        if not is_wsl:
            return

        powershell_bin = None
        for p in ["/mnt/c/Windows/System32/WindowsPowerShell/v1.0/powershell.exe",
                  "/mnt/c/Windows/System32/powershell.exe"]:
            if os.path.isfile(p):
                powershell_bin = p
                break

        if not powershell_bin:
            return

        ps_script = (
            "$code = @'\n"
            "using System;\n"
            "using System.Runtime.InteropServices;\n"
            "public class K {\n"
            "    [DllImport(\"user32.dll\")]\n"
            "    public static extern short GetAsyncKeyState(int vKey);\n"
            "}\n"
            "'@\n"
            "Add-Type -TypeDefinition $code\n"
            "$keys = @(0x26, 0x28, 0x25, 0x27, 0x57, 0x53, 0x41, 0x44, 0x20)\n"
            "while ($true) {\n"
            "    $m = 0\n"
            "    for ($i = 0; $i -lt $keys.Length; $i++) {\n"
            "        if ([K]::GetAsyncKeyState($keys[$i]) -band 0x8000) {\n"
            "            $m = $m -bor (1 -shl $i)\n"
            "        }\n"
            "    }\n"
            "    [Console]::WriteLine($m)\n"
            "    [System.Threading.Thread]::Sleep(12)\n"
            "}\n"
        )
        encoded_cmd = base64.b64encode(ps_script.encode('utf-16le')).decode('ascii')

        try:
            self.proc = subprocess.Popen(
                [powershell_bin, "-NoProfile", "-NonInteractive", "-EncodedCommand", encoded_cmd],
                stdout=subprocess.PIPE,
                stderr=subprocess.DEVNULL,
                bufsize=0
            )
            self._pipe_fd = self.proc.stdout.fileno()
            os.set_blocking(self._pipe_fd, False)
            self.active = True
        except Exception:
            self.active = False

    def poll_latest(self):
        """파이프 버퍼를 완전히 비우고 가장 최신의 키 비트마스크 갱신"""
        if not self.active or self._pipe_fd is None:
            return self.key_mask

        latest_data = b""
        while True:
            try:
                chunk = os.read(self._pipe_fd, 4096)
                if not chunk:
                    break
                latest_data += chunk
            except (BlockingIOError, InterruptedError):
                break

        if latest_data:
            lines = latest_data.strip().split(b"\n")
            if lines:
                last_line = lines[-1].strip()
                try:
                    self.key_mask = int(last_line)
                except ValueError:
                    pass
        return self.key_mask

    def terminate(self):
        if self.proc:
            try:
                self.proc.terminate()
                self.proc.wait(timeout=0.3)
            except Exception:
                pass


class ZAVFlightManager(Node):
    """
    ZAV 드론 중앙 비행 관리자 및 3차원 좌표계 변환 브릿지 노드
    """

    def __init__(self):
        super().__init__('zav_flight_manager')

        # QoS 프로파일 설정 (PX4 uORB 최적화: Best Effort, Transient Local)
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

        # -------------------------------------------------------------
        # 1. PX4 통신 퍼블리셔 (Micro-XRCE-DDS)
        # -------------------------------------------------------------
        self.cmd_pub = self.create_publisher(VehicleCommand, '/fmu/in/vehicle_command', qos_pub)
        self.offboard_mode_pub = self.create_publisher(OffboardControlMode, '/fmu/in/offboard_control_mode', qos_pub)
        self.traj_setpoint_pub = self.create_publisher(TrajectorySetpoint, '/fmu/in/trajectory_setpoint', qos_pub)
        self.rate_setpoint_pub = self.create_publisher(VehicleRatesSetpoint, '/fmu/in/vehicle_rates_setpoint', qos_pub)

        # -------------------------------------------------------------
        # 2. PX4 상태 구독자
        # -------------------------------------------------------------
        self.create_subscription(VehicleStatus, '/fmu/out/vehicle_status', self._on_vehicle_status, qos_sub)
        self.create_subscription(VehicleLocalPosition, '/fmu/out/vehicle_local_position', self._on_local_position, qos_sub)

        # -------------------------------------------------------------
        # 3. 상위 자율비행 제어기 입력 인터페이스 (포트 A & 포트 B)
        # -------------------------------------------------------------
        # 포트 A: 일반 플래너(SUPER/경로계획) 속도 명령 수신 (/zav/cmd_vel)
        self.create_subscription(Twist, '/zav/cmd_vel', self._on_external_cmd_vel, 10)
        # 포트 B: 고속 제어기(OMMPC) 각속도/추력 명령 수신 (/zav/cmd_rate)
        self.create_subscription(Twist, '/zav/cmd_rate', self._on_external_cmd_rate, 10)

        # -------------------------------------------------------------
        # 4. 비행 상태 관리 변수
        # -------------------------------------------------------------
        self.is_armed = False
        self.nav_state = 0
        self.flight_mode = "MANUAL_HOLD"  # POSCTL, ALTCTL, OFFBOARD_TRAJ, OFFBOARD_RATE

        # 기체 현재 상태 (PX4 EKF2 수신값)
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_z_ned = 0.0  # NED: 아래가 +, 상승 시 음수
        self.current_alt_enu = 0.0  # ENU: 위가 +, 직관적인 양수 고도(m)
        self.current_yaw = 0.0
        self.yaw_initialized = False

        # 목표 제어 변수
        self.target_yaw = 0.0
        self.hold_x = 0.0
        self.hold_y = 0.0
        self.hold_z_ned = 0.0

        # 속도 및 가감속 스펙 (ZAV 물리 제원 기반)
        self.max_speed_xy = 12.0     # 최대 수평 속도 12.0 m/s (43.2 km/h)
        self.max_speed_z_up = 3.0    # 최대 상승 속도 3.0 m/s
        self.max_speed_z_dn = 1.5    # 최대 하강 속도 1.5 m/s
        self.max_yaw_rate = 2.5      # 최대 각속도 2.5 rad/s (~143 deg/s)

        self.accel_xy = 14.0
        self.decel_xy = 28.0
        self.accel_z = 8.0
        self.decel_z = 16.0
        self.accel_yaw = 8.0
        self.decel_yaw = 16.0

        # 실시간 속도 명령 (Body FLU 기준)
        self.cmd_vx_body = 0.0
        self.cmd_vy_body = 0.0
        self.cmd_vz_body = 0.0       # Body FLU: +가 상승
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

        # 다중 키 감지기 초기화
        self.multi_key = MultiKeyPoller()

        # 터미널 설정 (싱글 키 감지용 non-blocking stdin)
        self.orig_term_settings = None
        try:
            self.orig_term_settings = termios.tcgetattr(sys.stdin)
            tty.setcbreak(sys.stdin.fileno())
            os.set_blocking(sys.stdin.fileno(), False)
        except Exception:
            pass

        # 주기적 제어 루프 타이머 (50Hz = 20ms 주기)
        self.last_loop_time = time.time()
        self.last_print_time = time.time()
        self.timer = self.create_timer(0.02, self._control_loop)

        # 안내 대시보드 출력
        self._print_banner()

    def _print_banner(self):
        print("""
==============================================================================
   🚁 [ZAV Quadrotor Central Flight Manager & Coordinate Bridge] 🚁
==============================================================================
  [비행 시동 및 비상 명령]
    Spacebar : 【시동(Arm) / 시동끄기(Disarm) 토글】 (비행 중: 비상 급정지 브레이크)
    W        : 【수동 스로틀 상승 / 이륙】 (시동 켠 뒤 W 누르면 부드럽게 지면 박차고 상승!)
    S        : 【수동 스로틀 하강】 (손 떼면 현재 고도에서 딱 고정 호버링)
    L        : 【자동 착륙 (Land)】 (서서히 수직 착륙 후 모터 자동 셧다운)
    R        : 【홈 복귀 후 착륙 (RTL)】 (이륙 지점으로 자동 복귀)
    E        : 🚨【EMERGENCY KILL】 (모터 즉시 강제 전원 차단 / 비상 추락)

  [비행 모드 원클릭 스위칭]
    [1] Position Mode  : GPS 기반 수동 제자리 호버링 모드
    [2] Altitude Mode  : 고도 유지 & 관성/바람 따라 흐르는 수동 모드
    [3] Offboard Traj  : 상위 플래너(/zav/cmd_vel) 자율비행 연동 (Level 4/3)
    [4] Offboard Rate  : 미래 OMMPC(/zav/cmd_rate) 고속 자율비행 연동 (Level 1)

  [방향키 수동 조종 (다중키 동시입력 완벽 지원)]
    ↑ / ↓    : 전진 / 후진 (0 -> 최대 12m/s [43km/h] 선형 가감속)
    ← / →    : 좌측 / 우측 이동
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
        self.current_alt_enu = -msg.z  # NED -> ENU 고도 변환 (지면 위 = 양수)

        if not self.yaw_initialized and not math.isnan(msg.heading):
            self.current_yaw = msg.heading
            self.target_yaw = msg.heading
            self.yaw_initialized = True
        elif not math.isnan(msg.heading):
            self.current_yaw = msg.heading

    def _on_external_cmd_vel(self, msg: Twist):
        """상위 플래너(SUPER 등)로부터의 속도 명령 수신 (/zav/cmd_vel)"""
        self.ext_cmd_vel = msg
        self.last_ext_vel_time = time.time()

    def _on_external_cmd_rate(self, msg: Twist):
        """고속 제어기(OMMPC 등)로부터의 각속도/추력 명령 수신 (/zav/cmd_rate)"""
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

    def set_arm(self, arm: bool):
        """시동 걸기(Arm) / 끄기(Disarm)"""
        self.send_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0 if arm else 0.0)

    def set_mode_posctl(self):
        """Position Mode (PX4 POSCTL) 전환"""
        self.send_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 3.0)  # Custom mode 3 = POSCTL
        self.flight_mode = "POSCTL"
        print("\r\n🎯 [모드 변경] Position Mode (GPS 제자리 호버링) 활성화!\r")

    def set_mode_altctl(self):
        """Altitude Mode (PX4 ALTCTL) 전환"""
        self.send_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 2.0)  # Custom mode 2 = ALTCTL
        self.flight_mode = "ALTCTL"
        print("\r\n🎈 [모드 변경] Altitude Mode (고도 유지 & 수평 수동 흐름) 활성화!\r")

    def set_mode_offboard_traj(self):
        """Offboard Trajectory Mode (위치/속도 자율비행) 전환"""
        self.send_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0)  # Custom mode 6 = OFFBOARD
        self.flight_mode = "OFFBOARD_TRAJ"
        print("\r\n🚀 [모드 변경] Offboard Trajectory Mode (상위 플래너 자율비행) 활성화!\r")

    def set_mode_offboard_rate(self):
        """Offboard Rate Mode (OMMPC 고속 각속도 자율비행) 전환"""
        self.send_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0)
        self.flight_mode = "OFFBOARD_RATE"
        print("\r\n⚡ [모드 변경] Offboard Rate Mode (OMMPC 고속 각속도 제어) 활성화!\r")

    def set_land(self):
        """자동 착륙 (Land)"""
        self.send_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
        self.flight_mode = "LAND"
        print("\r\n🛬 [명령] 자동 착륙(Land) 시작...\r")

    def set_rtl(self):
        """홈 복귀 후 착륙 (RTL)"""
        self.send_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_RETURN_TO_LAUNCH)
        self.flight_mode = "RTL"
        print("\r\n🏠 [명령] 홈 복귀(RTL) 시작...\r")

    def set_emergency_kill(self):
        """비상 강제 모터 차단 (Kill)"""
        self.send_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_FLIGHTTERMINATION, 1.0)
        print("\r\n🚨🚨 [EMERGENCY KILL] 모터 전원이 강제 차단되었습니다! 🚨🚨\r")

    def _poll_single_keys(self):
        """단축키 (Space, T, L, R, E, 1~4) 비차단 감지"""
        while True:
            try:
                ch = sys.stdin.read(1)
                if not ch:
                    break
            except Exception:
                break

            if ch == ' ':
                # 지면에 있으면 시동 토글, 공중에 있으면 급정지 브레이크
                if self.current_alt_enu < 0.25:
                    if not self.is_armed:
                        print("\r\n⚡ [시동] 시동(Arm) 인가 중... 모터 아이들링 시작!\r")
                        self.set_arm(True)
                    else:
                        print("\r\n🛑 [시동] 모터 시동 끄기(Disarm) 완료!\r")
                        self.set_arm(False)
                else:
                    print("\r\n🛑 [급제동] 제자리 긴급 브레이크 호버링!\r")
                    self.des_vx_body = self.cmd_vx_body = 0.0
                    self.des_vy_body = self.cmd_vy_body = 0.0
                    self.des_vz_body = self.cmd_vz_body = 0.0
                    self.des_yaw_rate = self.cmd_yaw_rate = 0.0
                    self.hold_x = self.current_x
                    self.hold_y = self.current_y
                    self.hold_z_ned = self.current_z_ned
                    self.target_yaw = self.current_yaw
            elif ch in ('t', 'T'):
                print("\r\n🚀 [자동이륙] 목표 고도 2.0m 자동 수직 상승 시작!\r")
                self.is_auto_takeoff = True
                self.auto_takeoff_start_time = time.time()
                self.hold_x = self.current_x
                self.hold_y = self.current_y
                self.target_yaw = self.current_yaw
                self.set_arm(True)
                self.set_mode_offboard_traj()
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
        """다중 키보드 입력으로부터 바디 속도 목표값(des_*) 산출"""
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

        # 수직 스로틀 축 (W / S) - W는 상승(+), S는 하강(-)
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
        """PX4 Offboard 모드 유지를 위한 하트비트 송신"""
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

        # 1. 단축키 및 방향키 입력 감지
        self._poll_single_keys()
        self._update_stick_inputs()

        # 2. 자동 2m 이륙(T) 모드 시퀀스 처리
        if self.is_auto_takeoff:
            if self.current_alt_enu >= 1.8 or (now - self.auto_takeoff_start_time > 8.0):
                print(f"\r\n✨ 자동 2.0m 이륙 완료! (현재 고도: {self.current_alt_enu:.2f}m) -> 수동/자율 조종 가능!\r")
                self.is_auto_takeoff = False
                self.hold_z_ned = -2.0
            else:
                self._publish_offboard_heartbeat(pos_vel=True, body_rate=False)
                sp = TrajectorySetpoint()
                sp.timestamp = 0
                sp.position = [float(self.hold_x), float(self.hold_y), -2.0]
                sp.velocity = [0.0, 0.0, -1.5]
                sp.yaw = float(self.target_yaw) if self.yaw_initialized else float('nan')
                self.traj_setpoint_pub.publish(sp)
                return

        # 3. 비행 모드에 따른 분기 제어
        if self.flight_mode == "OFFBOARD_RATE":
            # [모드 4: OMMPC 각속도/추력 직결 모드]
            self._publish_offboard_heartbeat(pos_vel=False, body_rate=True)
            rate_sp = VehicleRatesSetpoint()
            rate_sp.timestamp = 0
            if self.ext_cmd_rate and (now - self.last_ext_rate_time < 0.5):
                rate_sp.roll = float(self.ext_cmd_rate.angular.x)
                rate_sp.pitch = float(self.ext_cmd_rate.angular.y)
                rate_sp.yaw = float(self.ext_cmd_rate.angular.z)
                rate_sp.thrust_body[2] = float(-self.ext_cmd_rate.linear.z)  # NED Z축 추력
            else:
                rate_sp.roll = 0.0
                rate_sp.pitch = 0.0
                rate_sp.yaw = 0.0
                rate_sp.thrust_body[2] = -0.58  # 호버링 기본 추력 유지
            self.rate_setpoint_pub.publish(rate_sp)

        elif self.flight_mode in ("OFFBOARD_TRAJ", "MANUAL_HOLD"):
            # [모드 3 또는 기본 수동 텔레옵 모드: 위치/속도 제어]
            self._publish_offboard_heartbeat(pos_vel=True, body_rate=False)

            # 상위 플래너(/zav/cmd_vel) 명령이 최근에 들어왔는지 확인 (우선권 부여)
            use_external_vel = (self.ext_cmd_vel is not None) and (now - self.last_ext_vel_time < 0.3)

            # 키보드 입력이 있으면 조종사 우선 (Safety Interlock)
            key_active = (abs(self.des_vx_body) > 0.01 or abs(self.des_vy_body) > 0.01 or
                          abs(self.des_vz_body) > 0.01 or abs(self.des_yaw_rate) > 0.01)

            if key_active or not use_external_vel:
                # 조종사 수동 스틱 가감속 적용
                self.cmd_vx_body = ramp_value(self.cmd_vx_body, self.des_vx_body, self.accel_xy, self.decel_xy, dt)
                self.cmd_vy_body = ramp_value(self.cmd_vy_body, self.des_vy_body, self.accel_xy, self.decel_xy, dt)
                self.cmd_vz_body = ramp_value(self.cmd_vz_body, self.des_vz_body, self.accel_z, self.decel_z, dt)
                self.cmd_yaw_rate = ramp_value(self.cmd_yaw_rate, self.des_yaw_rate, self.accel_yaw, self.decel_yaw, dt)
            else:
                # 상위 플래너 명령 그대로 패스스루
                self.cmd_vx_body = self.ext_cmd_vel.linear.x
                self.cmd_vy_body = self.ext_cmd_vel.linear.y
                self.cmd_vz_body = self.ext_cmd_vel.linear.z
                self.cmd_yaw_rate = self.ext_cmd_vel.angular.z

            # 대각선 속도 원형 정규화
            vel_xy_norm = math.hypot(self.cmd_vx_body, self.cmd_vy_body)
            if vel_xy_norm > self.max_speed_xy:
                scale = self.max_speed_xy / vel_xy_norm
                vx_eff = self.cmd_vx_body * scale
                vy_eff = self.cmd_vy_body * scale
            else:
                vx_eff = self.cmd_vx_body
                vy_eff = self.cmd_vy_body

            # Yaw 헤딩 갱신
            if abs(self.cmd_yaw_rate) > 0.01:
                self.target_yaw = wrap_pi(self.current_yaw + self.cmd_yaw_rate * 0.15)

            # -------------------------------------------------------------
            # [실시간 3차원 좌표계 변환 (Body FLU -> World NED)]
            # -------------------------------------------------------------
            psi = self.target_yaw if self.yaw_initialized else self.current_yaw
            vn = vx_eff * math.cos(psi) - vy_eff * math.sin(psi)
            ve = vx_eff * math.sin(psi) + vy_eff * math.cos(psi)
            vd = -self.cmd_vz_body  # 상승(vz > 0) -> NED 하강 음수(vd < 0)

            sp = TrajectorySetpoint()
            sp.timestamp = 0

            # 조종 입력이 완전히 멈춘 경우: 위치 홀드(Position Hold) 고정
            is_moving = (abs(vx_eff) > 0.02 or abs(vy_eff) > 0.02 or
                         abs(self.cmd_vz_body) > 0.02 or abs(self.cmd_yaw_rate) > 0.02)

            if not is_moving and self.current_alt_enu > 0.2:
                # 제자리 위치 유지
                sp.position = [float('nan'), float('nan'), float('nan')]
                sp.velocity = [0.0, 0.0, 0.0]
            else:
                # 속도 추종
                sp.position = [float('nan'), float('nan'), float('nan')]
                sp.velocity = [float(vn), float(ve), float(vd)]

            sp.acceleration = [float('nan'), float('nan'), float('nan')]
            sp.yaw = float(self.target_yaw) if self.yaw_initialized else float('nan')
            sp.yawspeed = float(self.cmd_yaw_rate)
            self.traj_setpoint_pub.publish(sp)

        # 4. 실시간 대시보드 상태 출력 (0.08초 간격)
        if now - self.last_print_time > 0.08:
            self.last_print_time = now
            arm_str = "🟢 시동(Armed)" if self.is_armed else "🔴 정지(Disarmed)"
            mode_str = self.flight_mode
            v_xy = math.hypot(self.cmd_vx_body, self.cmd_vy_body)
            print(
                f"\r[{arm_str}] [{mode_str:13s}] "
                f"고도:{self.current_alt_enu:5.2f}m | "
                f"속도:XY {v_xy:4.1f}m/s, Z {self.cmd_vz_body:+4.1f}m/s | "
                f"헤딩:{math.degrees(self.current_yaw):+6.1f}°   ",
                end='', flush=True
            )

    def destroy_node(self):
        if self.orig_term_settings:
            try:
                termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.orig_term_settings)
            except Exception:
                pass
        self.multi_key.terminate()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ZAVFlightManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\r\n[종료] ZAV Flight Manager를 안전하게 종료합니다...\r")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
