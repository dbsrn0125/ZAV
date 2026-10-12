#!/usr/bin/env python3
"""
==============================================================================
ZAV Quadrotor Standard Teleoperation Node (PX4 Offboard Control)
==============================================================================
- 기체: 26 SfUAV ZAV 커스텀 쿼드콥터 (1.99 kg)
- 조종 방식: PX4 공식 표준 Offboard 속도 및 헤딩 제어
- 주요 특징:
  1. 기체 기준 헤딩 고정(Heading Lock):
     전후좌우 이동 시 sp.yaw에 목표 헤딩 각도를 지속적으로 인가하여
     피칭/롤링 중 발생하는 미세 반작용 토크에도 불구하고 기체 헤딩(Yaw)이 절대 틀어지지 않습니다.
  2. 스틱 중앙 복귀(Spring Return):
     키에서 손을 떼면 0.2초 후 수평/수직 목표 속도가 즉시 0.0으로 리셋되어
     제자리에서 안정적으로 호버링합니다.
  3. 안정적인 수직 이륙:
     이륙 시 수평 위치(0, 0)를 사수하며 수직 상승하여 대각선 밀림을 방지합니다.
  4. 직관적인 조종 방향 100% 일치:
     ↑: 전진, ↓: 후진, ←: 좌측 이동, →: 우측 이동
==============================================================================
"""

import sys
import termios
import tty
import select
import time
import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

# PX4 uORB 메시지 인터페이스
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand, VehicleStatus, VehicleLocalPosition


def wrap_pi(angle: float) -> float:
    """
    각도를 [-pi, +pi] 범위로 정규화 (PX4 uORB TrajectorySetpoint 규격)
    원리:
      Yaw 각도가 +pi를 초과하거나 -pi 미만으로 떨어질 때,
      2*pi 주기로 래핑하여 불연속성(급회전) 없이 부드러운 헤딩을 유지합니다.
    """
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


class ZAVTeleopNode(Node):
    def __init__(self):
        super().__init__('zav_teleop_keyboard')

        # ----------------------------------------------------------------------
        # 1. 상태 변수 및 비행 파라미터
        # ----------------------------------------------------------------------
        self.is_armed = False
        self.nav_state = 0
        self.is_flying = False
        self.is_taking_off = False
        self.takeoff_start_time = 0.0

        # 속도 파라미터
        self.move_speed_xy = 1.0     # 전후/좌우 비행 속도 (m/s)
        self.climb_speed_z = 0.6     # 상승/하강 속도 (m/s)
        self.yaw_speed = 0.5         # Yaw 회전 각속도 (rad/s, 초당 약 28.6°)

        # 센서 피드백
        self.current_alt = 0.0       # 현재 고도 (m)
        self.current_yaw = 0.0       # 현재 헤딩 (rad)
        self.current_x = 0.0         # 현재 X (m)
        self.current_y = 0.0         # 현재 Y (m)
        self.takeoff_x = 0.0         # 이륙 시점의 고정 X 좌표
        self.takeoff_y = 0.0         # 이륙 시점의 고정 Y 좌표

        # 제어 목표치 (Body 속도 및 목표 헤딩)
        self.target_vx_body = 0.0    # 전진(+)/후진(-)
        self.target_vy_body = 0.0    # 좌측(+)/우측(-)
        self.target_vz_ned = 0.0     # 상승(음수)/하강(양수)
        self.target_yaw = 0.0        # 목표 헤딩 각도 (rad, [-pi, +pi])
        self.target_yaw_rate = 0.0   # 목표 Yaw 각속도 (rad/s)
        self.yaw_initialized = False # 첫 EKF2 수신 시 현재 헤딩으로 초기화 플래그

        # 중앙 복귀 타이머
        self.last_key_time = 0.0
        self.last_yaw_key_time = 0.0
        self.key_timeout = 0.20      # 0.2초 동안 키 입력 없으면 속도 0 리셋

        # ----------------------------------------------------------------------
        # 2. QoS 프로파일 (PX4 uORB 규격)
        # ----------------------------------------------------------------------
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # ----------------------------------------------------------------------
        # 3. 퍼블리셔 및 서브스크라이버
        # ----------------------------------------------------------------------
        self.cmd_pub = self.create_publisher(VehicleCommand, '/fmu/in/vehicle_command', 10)
        self.offboard_mode_pub = self.create_publisher(OffboardControlMode, '/fmu/in/offboard_control_mode', 10)
        self.setpoint_pub = self.create_publisher(TrajectorySetpoint, '/fmu/in/trajectory_setpoint', 10)

        self.status_sub = self.create_subscription(
            VehicleStatus, '/fmu/out/vehicle_status', self._status_cb, qos_profile
        )
        self.status_v4_sub = self.create_subscription(
            VehicleStatus, '/fmu/out/vehicle_status_v4', self._status_cb, qos_profile
        )
        self.local_pos_sub = self.create_subscription(
            VehicleLocalPosition, '/fmu/out/vehicle_local_position', self._pos_cb, qos_profile
        )
        self.local_pos_v1_sub = self.create_subscription(
            VehicleLocalPosition, '/fmu/out/vehicle_local_position_v1', self._pos_cb, qos_profile
        )

        # 20Hz (50ms) 주기 제어 루프
        self.timer = self.create_timer(0.05, self._control_loop)

        self._print_guide()

    def _status_cb(self, msg: VehicleStatus):
        self.is_armed = (msg.arming_state == VehicleStatus.ARMING_STATE_ARMED)
        self.nav_state = msg.nav_state

    def _pos_cb(self, msg: VehicleLocalPosition):
        self.current_alt = -msg.z
        self.current_yaw = msg.heading
        self.current_x = msg.x
        self.current_y = msg.y
        # EKF2로부터 첫 헤딩을 수신하면 목표 헤딩을 현재 헤딩으로 초기 동기화
        if not self.yaw_initialized:
            self.target_yaw = msg.heading
            self.yaw_initialized = True

    def publish_offboard_mode(self):
        """PX4에게 속도 및 위치 제어 모드임을 지속 송신"""
        mode = OffboardControlMode()
        mode.position = True
        mode.velocity = True
        mode.acceleration = False
        mode.attitude = False
        mode.body_rate = False
        mode.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.offboard_mode_pub.publish(mode)

    def send_vehicle_command(self, command, param1=0.0, param2=0.0, param7=0.0):
        """PX4 시스템 명령 전송"""
        cmd = VehicleCommand()
        cmd.command = command
        cmd.param1 = param1
        cmd.param2 = param2
        cmd.param7 = param7
        cmd.target_system = 1
        cmd.target_component = 1
        cmd.source_system = 1
        cmd.source_component = 1
        cmd.from_external = True
        cmd.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.cmd_pub.publish(cmd)

    def _control_loop(self):
        now = time.time()
        self.publish_offboard_mode()

        sp = TrajectorySetpoint()
        sp.timestamp = int(self.get_clock().now().nanoseconds / 1000)

        # 1. 이륙 시퀀스: 수평 위치 (0, 0)을 사수하며 수직 상승 (대각선 밀림 방지)
        if self.is_taking_off:
            # 3초 조급한 타임아웃을 15초로 연장하고, 실제 센서 고도가 1.8m 이상 도달했을 때만 이륙 완료 판정
            if (self.current_alt >= 1.8) or (now - self.takeoff_start_time > 15.0 and self.current_alt >= 1.5):
                print(f"✨ 2.0m 도달 완료! (현재 고도: {self.current_alt:.2f}m) -> 제자리 호버링 고정!")
                self.is_taking_off = False
                self.is_flying = True
                self.target_vz_ned = 0.0
                self.target_vx_body = 0.0
                self.target_vy_body = 0.0
            else:
                # 시원한 박력 상승: 목표 고도 -2.0m, 수직 상승 속도 -1.5 m/s 인가하여 지면효과 신속 탈출
                sp.position = [float(self.takeoff_x), float(self.takeoff_y), -2.0]
                sp.velocity = [0.0, 0.0, -1.5]
                sp.yaw = float(wrap_pi(self.target_yaw)) if self.yaw_initialized else float('nan')
                sp.yawspeed = float('nan')
                self.setpoint_pub.publish(sp)
                return

        # 2. 비행 중 중앙 복귀: 키를 떼면 속도 0으로 리셋 (호버링 브레이크)
        if self.is_flying and (now - self.last_key_time > self.key_timeout):
            self.target_vx_body = 0.0
            self.target_vy_body = 0.0
            self.target_vz_ned = 0.0

        # -------------------------------------------------------------------------
        # 3. 항공우주 및 로봇공학 표준 Body(FLU) -> Inertial(NED) 2D 수평 회전 변환
        # -------------------------------------------------------------------------
        # 원리 설명 (Rule 2 상세 주석):
        # - 기체 바디 프레임(Body Frame, FLU):
        #     target_vx_body: 기체 정면(+X) 속도 성분 [전진: +, 후진: -]
        #     target_vy_body: 기체 좌측(+Y) 속도 성분 [좌측: +, 우측: -]
        # - PX4 비행제어 프레임(Inertial Frame, NED):
        #     vn: 북쪽(North, +X) 속도 성분
        #     ve: 동쪽(East, +Y) 속도 성분
        #     vd: 하향(Down, +Z) 속도 성분 (상승 시 음수, 하강 시 양수)
        #     psi (current_yaw): 기체의 현재 방위각 (북쪽 기준 시계방향 rad)
        #
        # FLU Body 속도를 FRD Body 속도로 변환:
        #     v_forward = target_vx_body
        #     v_right   = -target_vy_body
        #
        # FRD -> NED 2D 수평 회전 변환 행렬:
        #     [ vn ] = [ cos(psi)  -sin(psi) ] [ v_forward ]
        #     [ ve ]   [ sin(psi)   cos(psi) ] [ v_right   ]
        #
        # 대입하여 전개:
        #     vn = target_vx_body * cos(psi) + target_vy_body * sin(psi)
        #     ve = target_vx_body * sin(psi) - target_vy_body * cos(psi)
        # -------------------------------------------------------------------------
        # 조종 기준 헤딩(psi):
        # 키보드 A/D 키를 누르면 self.target_yaw가 갱신되며 드론이 그 방향으로 머리를 돌립니다.
        # 조종자가 조작하는 방향을 "머리가 향한 전진 방향"과 100% 일치시키기 위해,
        # 목표 헤딩(self.target_yaw)을 회전 변환 기준으로 직접 사용합니다.
        # -------------------------------------------------------------------------
        psi = self.target_yaw if self.yaw_initialized else self.current_yaw
        vn = self.target_vx_body * math.cos(psi) + self.target_vy_body * math.sin(psi)
        ve = self.target_vx_body * math.sin(psi) - self.target_vy_body * math.cos(psi)
        vd = self.target_vz_ned

        # 4. TrajectorySetpoint 송신:
        #    - PX4 공식 표준: 목표 각도(sp.yaw)를 직접 인가하고 sp.yawspeed는 NaN으로 설정하면
        #      PX4 자체의 EKF2/자세 제어기(mc_att_control)가 가장 안정적이고 부드러운 토크로 목표 헤딩을 사수합니다.
        sp.position = [float('nan'), float('nan'), float('nan')]
        sp.velocity = [float(vn), float(ve), float(vd)]
        sp.acceleration = [float('nan'), float('nan'), float('nan')]
        sp.yaw = float(wrap_pi(self.target_yaw)) if self.yaw_initialized else float('nan')
        sp.yawspeed = float('nan')
        self.setpoint_pub.publish(sp)

    def _print_guide(self):
        print("""
==================================================================
        🚁 [ZAV 드론 RC Mode 2 표준 조종 콘솔] 🚁
==================================================================
  [비행 명령]
    T        : 자동 수직 이륙 (2.0m 도달 후 호버링 고정)
    L        : 자동 착륙 (Land)
    Spacebar : 긴급 정지 호버링
    Q        : 착륙 후 종료

  [왼쪽 스틱 - 고도 및 Yaw 회전]
    W / S    : 상승 / 하강 (손 떼면 고도 유지)
    A / D    : 좌회전 / 우회전 (손 떼면 현재 각도 유지)

  [오른쪽 스틱 - 수평 비행 (중앙 복귀형)]
      ↑      : 전진 (Forward)  - [누를 때만 전진, 손 떼면 호버링]
    ←   →    : 좌측 / 우측 이동 - [누를 때만 이동, 손 떼면 호버링]
      ↓      : 후진 (Backward) - [누를 때만 후진, 손 떼면 호버링]
==================================================================
입력 대기 중... (T 키를 눌러 이륙하세요)
""")

    def update_teleop(self, key):
        now = time.time()

        if key == 't' or key == 'T':
            print("🚀 [이륙] 시동(Arm) & 수직 이륙 시작 (목표 고도 2.0m)...")
            self.takeoff_x = self.current_x
            self.takeoff_y = self.current_y
            self.target_yaw = wrap_pi(self.current_yaw) # 이륙 전 헤딩 각도 캡처
            self.target_yaw_rate = 0.0
            self.yaw_initialized = True
            self.send_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)
            time.sleep(0.05)
            self.send_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0)
            self.is_taking_off = True
            self.is_flying = True
            self.takeoff_start_time = now
            self.target_vz_ned = -1.5
            self.target_vx_body = 0.0
            self.target_vy_body = 0.0

        elif key == 'l' or key == 'L':
            print("🛬 [착륙] 자동 착륙(NAV_LAND) 명령 송신...")
            self.send_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
            self.target_vx_body = 0.0
            self.target_vy_body = 0.0
            self.target_vz_ned = 0.0
            self.target_yaw_rate = 0.0
            self.is_flying = False
            self.is_taking_off = False

        elif key == ' ':
            self.target_vx_body = 0.0
            self.target_vy_body = 0.0
            self.target_vz_ned = 0.0
            self.target_yaw = wrap_pi(self.current_yaw)
            self.target_yaw_rate = 0.0
            self.last_key_time = 0.0
            self.last_yaw_key_time = 0.0
            print(f"🛑 [브레이크] 호버링 고정! (고도: {self.current_alt:.2f}m, 헤딩: {math.degrees(self.target_yaw):.1f}°)")

        # 고도 조종 (W: 상승 / S: 하강)
        elif key == 'w' or key == 'W':
            self.target_vz_ned = -self.climb_speed_z
            self.last_key_time = now
            print(f"⬆ [상승] {self.climb_speed_z:.1f} m/s (고도: {self.current_alt:.2f}m)")

        elif key == 's' or key == 'S':
            self.target_vz_ned = self.climb_speed_z
            self.last_key_time = now
            print(f"⬇ [하강] {self.climb_speed_z:.1f} m/s (고도: {self.current_alt:.2f}m)")

        # Yaw 회전 조종 (A: 좌회전 / D: 우회전) - 누를 때마다 0.15 rad (약 8.6°) 회전 및 wrap_pi 정규화
        elif key == 'a' or key == 'A':
            self.target_yaw = wrap_pi(self.target_yaw - 0.15)
            self.last_key_time = now
            print(f"↺ [Yaw 좌회전] 목표각: {math.degrees(self.target_yaw):.1f}° (현재 기체 헤딩: {math.degrees(self.current_yaw):.1f}°)")

        elif key == 'd' or key == 'D':
            self.target_yaw = wrap_pi(self.target_yaw + 0.15)
            self.last_key_time = now
            print(f"↻ [Yaw 우회전] 목표각: {math.degrees(self.target_yaw):.1f}° (현재 기체 헤딩: {math.degrees(self.current_yaw):.1f}°)")

        # 수평 비행 (화살표 키) - 전진/후진, 좌/우
        elif key == '\x1b[A': # Up (전진)
            self.target_vx_body = self.move_speed_xy
            self.last_key_time = now
            print(f"▲ [전진] {self.move_speed_xy:.1f} m/s")

        elif key == '\x1b[B': # Down (후진)
            self.target_vx_body = -self.move_speed_xy
            self.last_key_time = now
            print(f"▼ [후진] {self.move_speed_xy:.1f} m/s")

        elif key == '\x1b[D': # Left (좌측 이동)
            self.target_vy_body = self.move_speed_xy
            self.last_key_time = now
            print(f"◀ [좌측 이동] {self.move_speed_xy:.1f} m/s")

        elif key == '\x1b[C': # Right (우측 이동)
            self.target_vy_body = -self.move_speed_xy
            self.last_key_time = now
            print(f"▶ [우측 이동] {self.move_speed_xy:.1f} m/s")

        # 종료 키 (Q)
        elif key == 'q' or key == 'Q':
            print("\n🛑 [종료] 자동 착륙 명령 후 노드를 안전하게 종료합니다...")
            if self.is_flying:
                self.send_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
            time.sleep(0.5)
            return False

        return True


def get_key(settings):
    """Linux 콘솔에서 논블로킹 방식으로 단일 키 및 화살표 특수키 안전 캡처"""
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.05)
    key = ''
    if rlist:
        key = sys.stdin.read(1)
        if key == '\x1b':
            rlist_arrow, _, _ = select.select([sys.stdin], [], [], 0.05)
            if rlist_arrow:
                key += sys.stdin.read(2)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


def main(args=None):
    old_settings = termios.tcgetattr(sys.stdin)
    rclpy.init(args=args)
    node = ZAVTeleopNode()

    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.01)
            key = get_key(old_settings)
            if key != '':
                if not node.update_teleop(key):
                    break

    except KeyboardInterrupt:
        pass
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)
        node.destroy_node()
        rclpy.shutdown()
        print("✅ 조종 노드가 정상적으로 종료되었습니다.")


if __name__ == '__main__':
    main()
