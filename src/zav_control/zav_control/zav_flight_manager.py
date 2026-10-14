#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
==============================================================================
    ZAV Flight Manager (Bridge Node & Safety Gateway)
==============================================================================
- 기반 코드: 검증 완료된 teleop_keyboard.py 원본 로직 100% 보존
- 핵심 제어 원리 (Rule 2 상세 주석):
  1. 드론 로컬 프레임(Body FLU) 기준 전후좌우 비행:
     - 전진/후진/좌/우 비행 방향은 월드 고정 좌표가 아니라,
       항상 드론의 현재 실시간 헤딩(self.current_yaw)을 기준으로 계산합니다.
     - 수식:
       vn = vx_body * cos(current_yaw) + vy_body * sin(current_yaw)
       ve = vx_body * sin(current_yaw) - vy_body * cos(current_yaw)
       vd = cmd_vz_ned
     - 이를 통해 드론 기수가 어느 각도를 바라보고 있든 '전진'은 항상 드론의 앞,
       '좌'는 항상 드론의 왼쪽으로 직관적이고 정확하게 기동하며,
       전진하면서 Yaw를 회전할 때 기수가 돌아가는 방향으로 전진 궤적이 자연스럽게 선회합니다.

  2. 안전 게이트웨이 (Safety Switch & Mode Gateway):
     - 1번 (MANUAL)       : teleop_keyboard.py 원본 조종 (누르면 가속, 떼면 즉시 제동 정지)
     - 3번 (OFFBOARD_TRAJ): 상위 경로 플래너(/zav/cmd_vel) 전담, 키보드 이동키 완전 차단 ⛔
     - 4번 (OFFBOARD_RATE): 고속 기동 제어기(/zav/cmd_rate) 전담, 키보드 이동키 완전 차단 ⛔
     - Spacebar           : 비상 호버링 브레이크 (시동 토글 지원)
     - T                  : 자동 2.0m 이륙 (Arm + Offboard 모드 자동 전환)
     - L                  : 자동 착륙 (Land)
     - E                  : 비상 모터 강제 전원 차단 (Forced Disarm 21196.0)
     - Q / Ctrl+C         : 안전 종료
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
KEY_SPACE = 1 << 8  # 256 (긴급 브레이크 Space)


def wrap_pi(angle: float) -> float:
    """각도를 [-pi, +pi] 범위로 정규화"""
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


def ramp_value(current: float, target: float, accel_rate: float, decel_rate: float, dt: float) -> float:
    """
    [스틱 선형 가감속 함수 (teleop_keyboard.py 원본 100% 동일)]
    - 스틱을 밀 때(가속)는 accel_rate로 선형 증가
    - 스틱을 놓거나 반대 방향 꺾을 때(감속)는 decel_rate로 즉각 복귀
    """
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
    """
    WSL2 환경에서 Windows user32.dll GetAsyncKeyState를 호출하여 다중 키 동시 입력을
    버퍼 지연(Zero-Lag) 없이 최신 상태로 읽어오는 리더
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

        # ----------------------------------------------------------------------
        # 1. teleop_keyboard.py 검증 스펙 100% 동일 적용
        # ----------------------------------------------------------------------
        self.is_armed = False
        self.nav_state = 0
        self.is_flying = False
        self.is_taking_off = False
        self.takeoff_start_time = 0.0
        self.flight_mode = "MANUAL"

        # 수평 비행 스펙 (PX4 MPC_XY_VEL_MAX = 12.0 m/s)
        self.max_speed_xy = 12.0
        self.accel_xy = 5.0
        self.decel_xy = 14.0

        # 수직 비행 스펙
        self.max_speed_z_up = 3.0
        self.max_speed_z_dn = 1.5
        self.accel_z = 2.5
        self.decel_z = 6.0

        # Yaw 회전 스펙
        self.max_yaw_rate = 2.5
        self.accel_yaw = 2.5
        self.decel_yaw = 8.0

        # 센서 피드백
        self.current_alt = 0.0
        self.current_yaw = 0.0
        self.current_x = 0.0
        self.current_y = 0.0
        self.takeoff_x = 0.0
        self.takeoff_y = 0.0
        self.yaw_initialized = False

        # 목표 스틱값 및 명령값
        self.des_vx_body = 0.0
        self.des_vy_body = 0.0
        self.des_vz_ned = 0.0
        self.des_yaw_rate = 0.0

        self.cmd_vx_body = 0.0
        self.cmd_vy_body = 0.0
        self.cmd_vz_ned = 0.0
        self.cmd_yaw_rate = 0.0

        self.target_yaw = 0.0
        self.was_turning_yaw = False
        self.was_moving = False
        self.last_print_time = 0.0

        # 외부 상위 명령 인터페이스 버퍼 (SUPER / Fast-LIO / OMMPC)
        self.ext_cmd_vel = None
        self.last_ext_vel_time = 0.0
        self.ext_cmd_rate = None
        self.last_ext_rate_time = 0.0

        # 다중 키 리더 시작
        self.multi_key = Win32MultiKeyReader()

        # 폴백용 타이머
        self.vx_release_time = 0.0
        self.vy_release_time = 0.0
        self.vz_release_time = 0.0
        self.yaw_release_time = 0.0

        # ----------------------------------------------------------------------
        # 2. QoS 프로파일 및 통신 인터페이스
        # ----------------------------------------------------------------------
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

        self.cmd_pub = self.create_publisher(VehicleCommand, '/fmu/in/vehicle_command', 10)
        self.offboard_mode_pub = self.create_publisher(OffboardControlMode, '/fmu/in/offboard_control_mode', 10)
        self.setpoint_pub = self.create_publisher(TrajectorySetpoint, '/fmu/in/trajectory_setpoint', 10)
        self.rate_setpoint_pub = self.create_publisher(VehicleRatesSetpoint, '/fmu/in/vehicle_rates_setpoint', qos_pub)

        self.status_sub = self.create_subscription(VehicleStatus, '/fmu/out/vehicle_status', self._status_cb, qos_sub)
        self.local_pos_sub = self.create_subscription(VehicleLocalPosition, '/fmu/out/vehicle_local_position', self._pos_cb, qos_sub)

        # 상위 플래너 입력
        self.create_subscription(Twist, '/zav/cmd_vel', self._on_external_cmd_vel, 10)
        self.create_subscription(Twist, '/zav/cmd_rate', self._on_external_cmd_rate, 10)

        # 50Hz (20ms) 고정밀 제어 루프
        self.dt = 0.02
        self.last_loop_time = time.time()
        self.timer = self.create_timer(self.dt, self._control_loop)

        self._print_guide()

    def _status_cb(self, msg: VehicleStatus):
        self.is_armed = (msg.arming_state == VehicleStatus.ARMING_STATE_ARMED)
        self.nav_state = msg.nav_state

    def _pos_cb(self, msg: VehicleLocalPosition):
        self.current_alt = -msg.z
        self.current_yaw = msg.heading
        self.current_x = msg.x
        self.current_y = msg.y
        if not self.yaw_initialized:
            self.target_yaw = wrap_pi(msg.heading)
            self.yaw_initialized = True
        if self.current_alt > 0.5 and not self.is_taking_off and not self.is_flying:
            self.is_flying = True
            self.target_yaw = wrap_pi(msg.heading)

    def _on_external_cmd_vel(self, msg: Twist):
        self.ext_cmd_vel = msg
        self.last_ext_vel_time = time.time()

    def _on_external_cmd_rate(self, msg: Twist):
        self.ext_cmd_rate = msg
        self.last_ext_rate_time = time.time()

    def publish_offboard_mode(self, pos_vel=True, body_rate=False):
        mode = OffboardControlMode()
        mode.position = pos_vel
        mode.velocity = pos_vel
        mode.acceleration = False
        mode.attitude = False
        mode.body_rate = body_rate
        mode.timestamp = 0
        self.offboard_mode_pub.publish(mode)

    def send_vehicle_command(self, command, param1=0.0, param2=0.0, param7=0.0):
        cmd = VehicleCommand()
        cmd.command = command
        cmd.param1 = float(param1)
        cmd.param2 = float(param2)
        cmd.param7 = float(param7)
        cmd.target_system = 1
        cmd.target_component = 1
        cmd.source_system = 1
        cmd.source_component = 1
        cmd.from_external = True
        cmd.timestamp = 0
        self.cmd_pub.publish(cmd)

    def _update_multi_key_targets(self):
        mask = self.multi_key.key_mask

        # 자율비행 모드(3번, 4번)일 때는 안전을 위해 수동 조종키 완전 차단!
        if self.flight_mode in ("OFFBOARD_TRAJ", "OFFBOARD_RATE"):
            self.des_vx_body = 0.0
            self.des_vy_body = 0.0
            self.des_vz_ned = 0.0
            self.des_yaw_rate = 0.0
            return

        # Spacebar: 즉시 긴급 정지 브레이크 (모든 속도 0)
        if mask & KEY_SPACE:
            self.des_vx_body = self.cmd_vx_body = 0.0
            self.des_vy_body = self.cmd_vy_body = 0.0
            self.des_vz_ned = self.cmd_vz_ned = 0.0
            self.des_yaw_rate = self.cmd_yaw_rate = 0.0
            self.target_yaw = wrap_pi(self.current_yaw)
            return

        # 1) 전후 축 (Up / Down) -> Body X (전진: +vx, 후진: -vx)
        if (mask & KEY_UP) and not (mask & KEY_DOWN):
            self.des_vx_body = self.max_speed_xy
        elif (mask & KEY_DOWN) and not (mask & KEY_UP):
            self.des_vx_body = -self.max_speed_xy
        else:
            self.des_vx_body = 0.0

        # 2) 좌우 축 (Left / Right) -> Body Y
        if (mask & KEY_LEFT) and not (mask & KEY_RIGHT):
            self.des_vy_body = self.max_speed_xy
        elif (mask & KEY_RIGHT) and not (mask & KEY_LEFT):
            self.des_vy_body = -self.max_speed_xy
        else:
            self.des_vy_body = 0.0

        # 3) 수직 축 (W / S) -> 상승: -vz, 하강: +vz (NED 기준)
        if (mask & KEY_W) and not (mask & KEY_S):
            self.des_vz_ned = -self.max_speed_z_up
            # 지상에서 W를 누를 때 Offboard가 아니면 자동 활성화
            if self.is_armed and self.nav_state != 14:
                self.send_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0)
        elif (mask & KEY_S) and not (mask & KEY_W):
            self.des_vz_ned = self.max_speed_z_dn
        else:
            self.des_vz_ned = 0.0

        # 4) Yaw 회전 축 (A / D) -> 좌회전: -rate, 우회전: +rate
        if (mask & KEY_A) and not (mask & KEY_D):
            self.des_yaw_rate = -self.max_yaw_rate
        elif (mask & KEY_D) and not (mask & KEY_A):
            self.des_yaw_rate = self.max_yaw_rate
        else:
            self.des_yaw_rate = 0.0

    def _control_loop(self):
        now = time.time()
        dt = min(0.1, max(0.005, now - self.last_loop_time))
        self.last_loop_time = now

        # 최신 키 상태 업데이트
        self.multi_key.poll_latest()

        # [케이스 1] 4번 모드: OFFBOARD_RATE (OMMPC 고속 각속도 자율비행)
        if self.flight_mode == "OFFBOARD_RATE":
            self.publish_offboard_mode(pos_vel=False, body_rate=True)
            rate_sp = VehicleRatesSetpoint()
            rate_sp.timestamp = 0
            if self.ext_cmd_rate and (now - self.last_ext_rate_time < 0.3):
                rate_sp.roll = float(self.ext_cmd_rate.angular.x)
                rate_sp.pitch = float(self.ext_cmd_rate.angular.y)
                rate_sp.yaw = float(self.ext_cmd_rate.angular.z)
                thrust_val = max(0.0, min(0.8, self.ext_cmd_rate.linear.z))
                rate_sp.thrust_body[2] = float(-thrust_val)
            else:
                rate_sp.roll = rate_sp.pitch = rate_sp.yaw = 0.0
                rate_sp.thrust_body[2] = -0.24
            self.rate_setpoint_pub.publish(rate_sp)
            return

        # [케이스 2] 3번 모드: OFFBOARD_TRAJ (SUPER / Fast-Planner 자율 속도 비행)
        if self.flight_mode == "OFFBOARD_TRAJ":
            self.publish_offboard_mode(pos_vel=True, body_rate=False)
            sp = TrajectorySetpoint()
            sp.timestamp = 0
            if self.ext_cmd_vel and (now - self.last_ext_vel_time < 0.3):
                vx = self.ext_cmd_vel.linear.x
                vy = self.ext_cmd_vel.linear.y
                vz = self.ext_cmd_vel.linear.z
                yaw_rate = self.ext_cmd_vel.angular.z

                # 드론 로컬 헤딩(current_yaw) 기준 변환
                psi = self.current_yaw
                vn = vx * math.cos(psi) + vy * math.sin(psi)
                ve = vx * math.sin(psi) - vy * math.cos(psi)
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
            self.setpoint_pub.publish(sp)
            return

        # [케이스 3] 수동 비행 및 이륙 (1번 MANUAL / T 키 이륙)
        self.publish_offboard_mode(pos_vel=True, body_rate=False)
        sp = TrajectorySetpoint()
        sp.timestamp = 0

        # A. 자동 이륙 시퀀스 (T 키)
        if self.is_taking_off:
            if (self.current_alt >= 1.8) or (now - self.takeoff_start_time > 8.0):
                self.target_yaw = wrap_pi(self.current_yaw)
                print(f"\r\n✨ 2.0m 이륙 완료! (고도: {self.current_alt:.2f}m, 헤딩: {math.degrees(self.target_yaw):.1f}°) -> 수동 조종 가능!\r\n")
                self.is_taking_off = False
                self.is_flying = True
                self.des_vx_body = self.cmd_vx_body = 0.0
                self.des_vy_body = self.cmd_vy_body = 0.0
                self.des_vz_ned = self.cmd_vz_ned = 0.0
                self.des_yaw_rate = self.cmd_yaw_rate = 0.0
            else:
                sp.position = [float(self.takeoff_x), float(self.takeoff_y), -2.0]
                sp.velocity = [0.0, 0.0, -1.5]
                sp.yaw = float(self.target_yaw) if self.yaw_initialized else float('nan')
                sp.yawspeed = 0.0
                self.setpoint_pub.publish(sp)
                return

        # B. 다중 키 입력 처리
        if self.multi_key.active:
            self._update_multi_key_targets()
        else:
            if now > self.vx_release_time: self.des_vx_body = 0.0
            if now > self.vy_release_time: self.des_vy_body = 0.0
            if now > self.vz_release_time: self.des_vz_ned = 0.0
            if now > self.yaw_release_time: self.des_yaw_rate = 0.0

        # C. 스틱 선형 가감속(Linear Ramp)
        self.cmd_vx_body = ramp_value(self.cmd_vx_body, self.des_vx_body, self.accel_xy, self.decel_xy, dt)
        self.cmd_vy_body = ramp_value(self.cmd_vy_body, self.des_vy_body, self.accel_xy, self.decel_xy, dt)
        self.cmd_vz_ned = ramp_value(self.cmd_vz_ned, self.des_vz_ned, self.accel_z, self.decel_z, dt)
        self.cmd_yaw_rate = ramp_value(self.cmd_yaw_rate, self.des_yaw_rate, self.accel_yaw, self.decel_yaw, dt)

        # D. 대각선 속도 원형 정규화
        vel_xy_norm = math.hypot(self.cmd_vx_body, self.cmd_vy_body)
        if vel_xy_norm > self.max_speed_xy:
            scale = self.max_speed_xy / vel_xy_norm
            vx_eff = self.cmd_vx_body * scale
            vy_eff = self.cmd_vy_body * scale
        else:
            vx_eff = self.cmd_vx_body
            vy_eff = self.cmd_vy_body

        # 실시간 속도 상태 출력 (조작 중에만 출력!)
        is_moving_now = (vel_xy_norm > 0.02) or (abs(self.cmd_vz_ned) > 0.02) or (abs(self.cmd_yaw_rate) > 0.02)
        if (is_moving_now or self.was_moving) and (now - self.last_print_time > 0.08):
            self.last_print_time = now
            self.was_moving = is_moving_now
            print(
                f"\r🎮 [스틱] 수평:{vel_xy_norm:4.1f}m/s (전후:{vx_eff:+5.1f}, 좌우:{vy_eff:+5.1f}) | "
                f"수직:{-self.cmd_vz_ned:+4.1f}m/s | "
                f"Yaw:{math.degrees(self.cmd_yaw_rate):+5.1f}°/s (헤딩:{math.degrees(self.current_yaw):+6.1f}°)   ",
                end='', flush=True
            )

        # E. 드론 로컬 헤딩 기준 3차원 속도 변환 (핵심!)
        # 전진(↑)은 항상 드론 기수 실시간 방향(self.current_yaw)의 앞쪽으로 이동!
        # 전진하면서 Yaw 회전 시 기수가 도는 방향으로 전진 궤적이 자연스럽게 선회함!
        psi = self.current_yaw
        vn = vx_eff * math.cos(psi) + vy_eff * math.sin(psi)
        ve = vx_eff * math.sin(psi) - vy_eff * math.cos(psi)
        vd = self.cmd_vz_ned

        # F. TrajectorySetpoint 퍼블리시
        sp.position = [float('nan'), float('nan'), float('nan')]
        sp.velocity = [float(vn), float(ve), float(vd)]
        sp.acceleration = [float('nan'), float('nan'), float('nan')]

        # 회전 중일 때는 yawspeed 지정, 뗐을 때는 헤딩 잠금
        if abs(self.cmd_yaw_rate) > 0.01:
            self.was_turning_yaw = True
            self.target_yaw = wrap_pi(self.current_yaw + self.cmd_yaw_rate * 0.15)
            sp.yaw = float('nan')
            sp.yawspeed = float(self.cmd_yaw_rate)
        else:
            if self.was_turning_yaw:
                self.target_yaw = wrap_pi(self.current_yaw)
                self.was_turning_yaw = False
            sp.yaw = float(self.target_yaw) if self.yaw_initialized else float('nan')
            sp.yawspeed = 0.0

        self.setpoint_pub.publish(sp)

    def _print_guide(self):
        print("""
======================================================================
   🚁 [ZAV Flight Manager - Clean Architecture] 🚁
======================================================================
  [비행 명령 (터미널 창에서 입력)]
    Spacebar : 시동 토글 (Arm / Disarm) 및 긴급 제자리 브레이크
    T        : 자동 수직 이륙 (목표 고도 2.0m 도달 후 호버링)
    1        : 수동 조종 모드 (MANUAL)
    3        : 외부 자율 속도 모드 (OFFBOARD_TRAJ - 키보드 차단 ⛔)
    4        : 외부 자율 각속도 모드 (OFFBOARD_RATE - OMMPC 연동 ⛔)
    L        : 자동 착륙 (Land)
    E        : 비상 모터 강제 전원 차단 (Emergency Kill)
    Q        : 안전 종료

  [수동 조종키 (드론 로컬 기수 방향 기준)]
    W / S    : 상승(최대 3.0 m/s) / 하강(최대 1.5 m/s)
    A / D    : 좌회전 / 우회전 (선형 가속)
    ↑ / ↓    : 드론 기수 앞(전진) / 뒤(후진)
    ← / →    : 드론 기수 왼쪽 / 오른쪽
    ★ 대각선 : [↑ + ←], [↑ + →] 다중 키 동시 입력 지원
======================================================================
입력 대기 중...
""")

    def _extend_hold_time(self, current_release_time: float, now: float) -> float:
        return (now + 0.58) if now > current_release_time else (now + 0.18)

    def update_teleop(self, key: str) -> bool:
        now = time.time()

        if key in ('t', 'T'):
            print("\r\n🚀 [이륙] 시동(Arm) & 2.0m 자동 수직 이륙 시작...\r\n")
            self.takeoff_x = self.current_x
            self.takeoff_y = self.current_y
            self.target_yaw = wrap_pi(self.current_yaw)
            self.yaw_initialized = True
            self.des_vx_body = self.cmd_vx_body = 0.0
            self.des_vy_body = self.cmd_vy_body = 0.0
            self.des_vz_ned = self.cmd_vz_ned = 0.0
            self.des_yaw_rate = self.cmd_yaw_rate = 0.0
            self.send_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)
            time.sleep(0.05)
            self.send_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0)
            self.is_taking_off = True
            self.is_flying = True
            self.takeoff_start_time = now

        elif key in ('l', 'L'):
            print("\r\n🛬 [착륙] 자동 착륙(NAV_LAND) 명령 송신...\r\n")
            self.send_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
            self.des_vx_body = self.cmd_vx_body = 0.0
            self.des_vy_body = self.cmd_vy_body = 0.0
            self.des_vz_ned = self.cmd_vz_ned = 0.0
            self.des_yaw_rate = self.cmd_yaw_rate = 0.0
            self.is_flying = False
            self.is_taking_off = False

        elif key == ' ':
            new_arm = not self.is_armed
            self.send_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0 if new_arm else 0.0)
            if new_arm:
                time.sleep(0.05)
                self.send_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0)
            self.des_vx_body = self.cmd_vx_body = 0.0
            self.des_vy_body = self.cmd_vy_body = 0.0
            self.des_vz_ned = self.cmd_vz_ned = 0.0
            self.des_yaw_rate = self.cmd_yaw_rate = 0.0
            self.target_yaw = wrap_pi(self.current_yaw)
            action = "시동 켜기(ARM) & 브레이크" if new_arm else "시동 끄기(DISARM)"
            print(f"\r\n🛑 [{action}] (고도: {self.current_alt:.2f}m, 헤딩: {math.degrees(self.target_yaw):.1f}°)\r\n")

        elif key in ('e', 'E'):
            self.send_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0, 21196.0)
            self.is_armed = False
            print("\r\n🚨🚨 [EMERGENCY STOP] 모터 전원이 강제 차단되었습니다! 🚨🚨\r\n")

        elif key == '1':
            self.flight_mode = "MANUAL"
            print("\r\n🎯 [모드 변경] 수동 조종 모드 (MANUAL) 활성화!\r\n")

        elif key == '3':
            self.flight_mode = "OFFBOARD_TRAJ"
            print("\r\n🚀 [모드 변경] Offboard Trajectory Mode (상위 플래너 자율비행, 키보드 이동 차단 ⛔)!\r\n")

        elif key == '4':
            self.flight_mode = "OFFBOARD_RATE"
            print("\r\n⚡ [모드 변경] Offboard Rate Mode (OMMPC 고속 자율비행, 키보드 이동 차단 ⛔)!\r\n")

        elif key in ('q', 'Q', '\x03'):
            print("\r\n🛑 [종료] 노드를 안전하게 종료합니다...\r\n")
            if self.is_flying:
                self.send_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
            time.sleep(0.3)
            return False

        # 리눅스 터미널 직접 입력 폴백
        if not self.multi_key.active and self.flight_mode == "MANUAL":
            if key in ('w', 'W'):
                self.vz_release_time = self._extend_hold_time(self.vz_release_time, now)
                self.des_vz_ned = -self.max_speed_z_up
            elif key in ('s', 'S'):
                self.vz_release_time = self._extend_hold_time(self.vz_release_time, now)
                self.des_vz_ned = self.max_speed_z_dn
            elif key in ('a', 'A'):
                self.yaw_release_time = self._extend_hold_time(self.yaw_release_time, now)
                self.des_yaw_rate = -self.max_yaw_rate
            elif key in ('d', 'D'):
                self.yaw_release_time = self._extend_hold_time(self.yaw_release_time, now)
                self.des_yaw_rate = self.max_yaw_rate
            elif key in ('\x1b[A', '\x1bOA'):
                self.vx_release_time = self._extend_hold_time(self.vx_release_time, now)
                self.des_vx_body = self.max_speed_xy
            elif key in ('\x1b[B', '\x1bOB'):
                self.vx_release_time = self._extend_hold_time(self.vx_release_time, now)
                self.des_vx_body = -self.max_speed_xy
            elif key in ('\x1b[D', '\x1bOD'):
                self.vy_release_time = self._extend_hold_time(self.vy_release_time, now)
                self.des_vy_body = self.max_speed_xy
            elif key in ('\x1b[C', '\x1bOC'):
                self.vy_release_time = self._extend_hold_time(self.vy_release_time, now)
                self.des_vy_body = -self.max_speed_xy

        return True


def main(args=None):
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    rclpy.init(args=args)
    node = ZAVFlightManager()

    try:
        tty.setraw(fd, termios.TCSANOW)
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.005)
            key = read_key_nonblocking(fd)
            if key != '':
                if not node.update_teleop(key):
                    break
    except KeyboardInterrupt:
        pass
    finally:
        node.multi_key.stop()
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        node.destroy_node()
        rclpy.shutdown()
        print("\n\n✅ ZAV Flight Manager가 안전하게 종료되었습니다.")


if __name__ == '__main__':
    main()
