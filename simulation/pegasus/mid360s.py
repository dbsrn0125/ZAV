"""
| File: mid360s.py
| Description: Livox Mid-360s 3D LiDAR graphical sensor simulation for Pegasus Simulator (Isaac Sim 5.1)
| Specs & Extrinsics (extracted from ZAV_v1 Fusion 360 CAD & FAST-LIVO2 config):
|   - Channels: 40 channels, Vertical FOV: -7.0 deg ~ +52.0 deg, Horizontal FOV: 360 deg
|   - Scan Rate: 10 Hz (reportRateBaseHz = 5000 -> 200,000 pts/s)
|   - Range: 0.10 m ~ 100.0 m (minReflectionRange = 40.0 m @ 10% reflectivity)
|   - Optical Origin in ZAV_v1 Body Frame (FLU): [0.01205, 0.0, 0.10857] m
|   - Mounting Orientation in Body Frame (FLU): Roll = 0.0 deg, Pitch = +22.0 deg (tilted forward-down), Yaw = 0.0 deg
|   - Default ROS 2 Topic: /livox/lidar (frame_id: livox_frame)
"""
__all__ = ["Mid360s"]

import os
import sys
import carb
import numpy as np
from scipy.spatial.transform import Rotation

import omni.kit.app
import omni.kit.commands
import omni.replicator.core as rep
from pxr import Gf, Sdf, Usd, Vt
from omni.usd import get_stage_next_free_path
from isaacsim.core.utils.extensions import enable_extension

from pegasus.simulator.logic.state import State
from pegasus.simulator.logic.graphical_sensors import GraphicalSensor
from pegasus.simulator.logic.interface.pegasus_interface import PegasusInterface


class Mid360s(GraphicalSensor):
    """
    Livox Mid-360s RTX LiDAR 센서 클래스.

    [물리 및 좌표계 변환 원리 (Rule 2 상세 주석)]:
    1. 기체 바디 좌표계 (Body Frame, FLU):
       - 원점: ZAV_v1 기체 기준 원점 (/World/quadrotor/body)
       - 축 정의: +X(전방, Forward), +Y(좌측, Left), +Z(상방, Up)
    2. 센서 장착 위치 (Optical Origin Offset):
       - ZAV_v1 CAD 메쉬(MeshInstance_151~163)에서 추출한 라이다 밑면 중심은 [-0.00143, 0.0, 0.07519] m.
       - Mid-360s 매뉴얼 규격상 레이저 발사 광학 원점(Optical Center)은 밑면으로부터 센서 +Z 축으로 +36.0 mm 위에 위치함.
       - 센서가 전방으로 theta = +22.0 deg 기울어져 있으므로, 광학 원점의 기체 좌표계 위치는:
         O_L = P_base + 0.036 * [sin(22 deg), 0.0, cos(22 deg)]^T
             = [0.01205, 0.0, 0.10857]^T (m)
    3. 센서 장착 자세 (Mounting Orientation):
       - 실기체와 동일하게 기체 정면(+X) 아래 방향으로 22도 숙여진(Forward-downward tilt) 장착 상태.
       - FLU 좌표계에서 +Y(좌측) 축 기준 오른손 법칙 회전(Pitch = +22.0 deg)은 센서 +X 축을 전방 아래로,
         센서 +Z(상단) 축을 전방 기울어진 방향으로 회전시킴.
       - 쿼터니언 변환: q = [qx, qy, qz, qw] = [0.0, sin(11 deg), 0.0, cos(11 deg)]
         (USD Gf.Quatd 입력 시에는 실수부 qw가 첫 번째 인자로 들어감: Gf.Quatd(qw, qx, qy, qz))
    4. 자가 충돌(Self-occlusion) 방지 원리:
       - ZAV_v1.usd의 외형 메쉬 내부에 광학 원점이 위치하지만,
         Mid-360s의 최소 측정 거리(nearRangeM = 0.10 m)가 라이다 외형 반경(~0.032 m)보다 크므로
         라이다 자체 메쉬에 레이저가 가려지지 않고 투과하여 주변 지형/장애물만 정확히 스캔함.
    """

    # Mid-360s 40채널 수직 발사각 (Elevation Angles: -7.0 deg ~ +52.0 deg 균등 분포)
    DEFAULT_ELEVATION_DEG = [
        -7.0, -5.4871793, -3.974359, -2.4615386, -0.94871795,
        0.5641026, 2.0769231, 3.5897436, 5.1025643, 6.6153846,
        8.128205, 9.641026, 11.153846, 12.666667, 14.179487,
        15.692307, 17.205128, 18.717949, 20.23077, 21.74359,
        23.25641, 24.76923, 26.282051, 27.794872, 29.307692,
        30.820513, 32.333332, 33.846153, 35.358974, 36.871796,
        38.384617, 39.897434, 41.410255, 42.923077, 44.435898,
        45.94872, 47.46154, 48.974358, 50.48718, 52.0
    ]

    def __init__(self, lidar_name: str = "mid360s", config: dict = {}):
        """
        Mid360s 라이다 센서를 초기화합니다.

        Args:
            lidar_name (str): 드론의 /body 하위에 생성될 센서 Prim 이름 (기본값: "mid360s")
            config (dict): 센서 장착 위치, 자세, 주파수, ROS 2 토픽 설정 딕셔너리
                - position (list/np.ndarray): 기체 /body 대비 장착 위치 [x, y, z] (m). 기본값: [0.01205, 0.0, 0.10857]
                - orientation (list/np.ndarray): 기체 /body 대비 오일러 회전각 [Roll, Pitch, Yaw] (deg). 기본값: [0.0, 22.0, 0.0]
                - frequency (float): 스캔 주파수 (Hz). 기본값: 10.0
                - ros2_publish (bool): 자체 ROS 2 PointCloud2 퍼블리셔 활성화 여부. 기본값: True
                - full_scan (bool): 360도 전체 스캔 누적 버퍼(RtxLidarROS2PublishPointCloudBuffer) 사용 여부. 기본값: True
                - topic_name (str): 발행할 ROS 2 PointCloud2 토픽명. 기본값: "/livox/lidar"
                - frame_id (str): PointCloud2 메시지의 헤더 frame_id. 기본값: "livox_frame"
                - show_render (bool): Isaac Sim 뷰포트 내 3D 포인트클라우드 디버그 시각화(DebugDraw) 여부. 기본값: False
        """
        frequency = float(config.get("frequency", 10.0))
        super().__init__(sensor_type="Mid360s", update_rate=frequency)

        self._lidar_name = lidar_name
        self._stage_prim_path = ""

        # ZAV_v1 CAD에서 추출한 정밀 장착 좌표 및 전방 +22도 틸트 각도 기본 적용
        self._position = np.array(config.get("position", [0.01205, 0.0, 0.10857]), dtype=float)
        euler_rpy = np.array(config.get("orientation", [0.0, 22.0, 0.0]), dtype=float)
        # Intrinsic XYZ (Roll -> Pitch -> Yaw) 오일러 각을 쿼터니언 [qx, qy, qz, qw]로 변환
        self._orientation = Rotation.from_euler("XYZ", euler_rpy, degrees=True).as_quat()

        self._frequency = int(round(frequency))
        self._ros2_publish = config.get("ros2_publish", True)
        self._full_scan = config.get("full_scan", True)
        self._topic_name = config.get("topic_name", "/livox/lidar")
        self._frame_id = config.get("frame_id", "livox_frame")
        self._show_render = config.get("show_render", False)

        self._sensor = None
        self._render_product = None
        self._writers = []
        self._initialized_publishers = False
        self._state = {}

    @staticmethod
    def _ensure_ros2_bridge_env():
        """
        Windows 및 Linux 환경에서 isaacsim.ros2.bridge 확장이 내장 ROS 2(humble) 라이브러리를
        에러 없이 로드할 수 있도록 환경변수(ROS_DISTRO, RMW_IMPLEMENTATION, PATH/LD_LIBRARY_PATH)를 자동 설정합니다.
        """
        ros_distro = os.environ.get("ROS_DISTRO") or "humble"
        os.environ["ROS_DISTRO"] = ros_distro
        os.environ.setdefault("RMW_IMPLEMENTATION", "rmw_fastrtps_cpp")

        ext_path = None
        try:
            ext_mgr = omni.kit.app.get_app().get_extension_manager()
            ext_id = ext_mgr.get_extension_id_by_module("isaacsim.ros2.bridge")
            if ext_id:
                ext_path = ext_mgr.get_extension_path(ext_id)
        except Exception:
            pass

        if not ext_path:
            isaac_path = os.environ.get("ISAAC_PATH", "")
            if isaac_path:
                cand = os.path.join(isaac_path, "exts", "isaacsim.ros2.bridge")
                if os.path.isdir(cand):
                    ext_path = cand

        if ext_path:
            ros_lib_dir = os.path.normpath(os.path.join(ext_path, ros_distro, "lib"))
            bin_dir = os.path.normpath(os.path.join(ext_path, "bin"))
            if sys.platform == "win32":
                cur_path = os.environ.get("PATH", "")
                for d in (ros_lib_dir, bin_dir):
                    if os.path.isdir(d) and d not in cur_path:
                        cur_path = cur_path + ";" + d if cur_path else d
                os.environ["PATH"] = cur_path
            elif sys.platform.startswith("linux"):
                cur_ld = os.environ.get("LD_LIBRARY_PATH", "")
                if os.path.isdir(ros_lib_dir) and ros_lib_dir not in cur_ld:
                    os.environ["LD_LIBRARY_PATH"] = cur_ld + ":" + ros_lib_dir if cur_ld else ros_lib_dir

    def _apply_mid360s_usd_attributes(self, prim: Usd.Prim):
        """
        Mid-360s.usd에 정의된 Livox Mid-360s 전용 OmniSensorGenericLidarCoreAPI 파라미터를
        생성된 OmniLidar Prim에 그대로 주입합니다.
        """
        if not prim or not prim.IsValid():
            carb.log_error(f"[Mid360s] Invalid Lidar Prim at {self._stage_prim_path}")
            return

        num_channels = 40
        elev_array = Vt.FloatArray(self.DEFAULT_ELEVATION_DEG)
        zeros_f = Vt.FloatArray([0.0] * num_channels)
        ones_f = Vt.FloatArray([1.0] * num_channels)
        zeros_i = Vt.IntArray([0] * num_channels)
        ones_u = Vt.UIntArray([1] * num_channels)
        chan_ids = Vt.UIntArray(list(range(1, num_channels + 1)))

        attrs = {
            # 스캔 주기 및 리포트 레이트 (10 Hz 회전, 초당 5000회 발사 * 40채널 = 200,000 pts/s)
            "omni:sensor:Core:scanRateBaseHz": (Sdf.ValueTypeNames.Int, self._frequency),
            "omni:sensor:Core:reportRateBaseHz": (Sdf.ValueTypeNames.Int, 5000),
            "omni:sensor:Core:numberOfChannels": (Sdf.ValueTypeNames.Int, num_channels),
            "omni:sensor:Core:numberOfEmitters": (Sdf.ValueTypeNames.Int, num_channels),
            # 측정 거리 및 반사율 특성 (최소 0.1m ~ 최대 100.0m, 10% 반사율 기준 40.0m)
            "omni:sensor:Core:nearRangeM": (Sdf.ValueTypeNames.Float, 0.1),
            "omni:sensor:Core:farRangeM": (Sdf.ValueTypeNames.Float, 100.0),
            "omni:sensor:Core:minReflectionRangeM": (Sdf.ValueTypeNames.Float, 40.0),
            "omni:sensor:Core:minReflectance": (Sdf.ValueTypeNames.Float, 0.1),
            "omni:sensor:Core:maxReturns": (Sdf.ValueTypeNames.Int, 1),
            "omni:sensor:Core:waveLengthNm": (Sdf.ValueTypeNames.Float, 905.0),
            "omni:sensor:Core:pulseTimeNs": (Sdf.ValueTypeNames.Int, 6),
            # 측정 노이즈 및 정확도 (거리 오차 2cm, 방위각/고도각 편차 0.15 deg)
            "omni:sensor:Core:rangeAccuracyM": (Sdf.ValueTypeNames.Float, 0.02),
            "omni:sensor:Core:azimuthErrorStd": (Sdf.ValueTypeNames.Float, 0.15),
            "omni:sensor:Core:elevationErrorStd": (Sdf.ValueTypeNames.Float, 0.15),
            "omni:sensor:Core:skipDroppingInvalidPoints": (Sdf.ValueTypeNames.Bool, True),
            # 40채널 에미터 배열 설정 (-7.0 deg ~ +52.0 deg)
            "omni:sensor:Core:emitterState:s001:elevationDeg": (Sdf.ValueTypeNames.FloatArray, elev_array),
            "omni:sensor:Core:emitterState:s001:azimuthDeg": (Sdf.ValueTypeNames.FloatArray, zeros_f),
            "omni:sensor:Core:emitterState:s001:fireTimeNs": (Sdf.ValueTypeNames.IntArray, zeros_i),
            "omni:sensor:Core:emitterState:s001:channelId": (Sdf.ValueTypeNames.UIntArray, chan_ids),
            "omni:sensor:Core:emitterState:s001:bank": (Sdf.ValueTypeNames.UIntArray, ones_u),
            "omni:sensor:Core:emitterState:s001:horOffsetM": (Sdf.ValueTypeNames.FloatArray, zeros_f),
            "omni:sensor:Core:emitterState:s001:vertOffsetM": (Sdf.ValueTypeNames.FloatArray, zeros_f),
            "omni:sensor:Core:emitterState:s001:reportRateDiv": (Sdf.ValueTypeNames.FloatArray, ones_f),
        }

        for attr_name, (sdf_type, val) in attrs.items():
            attr = prim.GetAttribute(attr_name)
            if not attr.IsValid():
                attr = prim.CreateAttribute(attr_name, sdf_type)
            attr.Set(val)

    def initialize(self, vehicle):
        """
        드론(Vehicle)의 /body 하위에 Mid-360s RTX Lidar Prim을 생성하고 파라미터를 바인딩합니다.
        """
        super().initialize(vehicle)

        stage = PegasusInterface().world.stage
        parent_path = self._vehicle.prim_path + "/body"
        self._stage_prim_path = get_stage_next_free_path(stage, parent_path + "/" + self._lidar_name, False)
        self._lidar_name = self._stage_prim_path.rpartition("/")[-1]

        # Isaac Sim 5.1 RTX Lidar 생성 (Example_Rotary 템플릿 기반 OmniLidar 생성)
        _, self._sensor = omni.kit.commands.execute(
            "IsaacSensorCreateRtxLidar",
            path=self._lidar_name,
            parent=parent_path,
            config="Example_Rotary",
            translation=Gf.Vec3d(float(self._position[0]), float(self._position[1]), float(self._position[2])),
            orientation=Gf.Quatd(
                float(self._orientation[3]),
                float(self._orientation[0]),
                float(self._orientation[1]),
                float(self._orientation[2]),
            ),
        )

        # 생성된 OmniLidar Prim에 Livox Mid-360s 전용 파라미터 덮어쓰기
        sensor_prim = self._sensor if isinstance(self._sensor, Usd.Prim) else stage.GetPrimAtPath(self._stage_prim_path)
        self._apply_mid360s_usd_attributes(sensor_prim)

        carb.log_info(
            f"[Mid360s] Initialized at {self._stage_prim_path} | "
            f"pos={self._position.tolist()}, topic={self._topic_name}, frame_id={self._frame_id}"
        )

    def start(self):
        """
        시뮬레이션 시작 시 RTX Lidar용 Render Product 및 ROS 2 PointCloud2 퍼블리셔를 연결합니다.
        """
        if self._initialized_publishers:
            return

        if not self._ros2_publish and not self._show_render:
            return

        # RTX Lidar는 GPU 레이트레이싱을 수행하므로 전용 Render Product가 필수임
        self._render_product = rep.create.render_product(self._stage_prim_path, [1, 1], name=f"rp_{self._lidar_name}")

        # 1. ROS 2 PointCloud2 토픽 발행 파이프라인 연결 (/livox/lidar)
        if self._ros2_publish:
            try:
                self._ensure_ros2_bridge_env()
                enable_extension("isaacsim.ros2.bridge")
                writer_type = "RtxLidarROS2PublishPointCloudBuffer" if self._full_scan else "RtxLidarROS2PublishPointCloud"
                writer_pc = rep.writers.get(writer_type)
                writer_pc.initialize(
                    topicName=self._topic_name,
                    frameId=self._frame_id,
                )
                writer_pc.attach([self._render_product])
                self._writers.append(writer_pc)
                carb.log_info(
                    f"[Mid360s] ROS 2 PointCloud2 publisher ({writer_type}) attached -> "
                    f"topic: '{self._topic_name}' (frame_id: '{self._frame_id}')"
                )
            except Exception as e:
                carb.log_error(f"[Mid360s] ROS 2 PointCloud2 퍼블리셔 연결 실패: {e}")

        # 2. 뷰포트 3D 포인트 시각화 (옵션)
        if self._show_render:
            try:
                draw_type = "RtxLidarDebugDrawPointCloudBuffer" if self._full_scan else "RtxLidarDebugDrawPointCloud"
                writer_draw = rep.writers.get(draw_type)
                writer_draw.attach([self._render_product])
                self._writers.append(writer_draw)
            except Exception as e:
                carb.log_warn(f"[Mid360s] DebugDraw 연결 실패: {e}")

        self._initialized_publishers = True

    def stop(self):
        """시뮬레이션 정지 시 호출"""
        pass

    @property
    def state(self):
        """현재 센서의 상태 딕셔너리 반환"""
        return self._state

    @GraphicalSensor.update_at_rate
    def update(self, state: State, dt: float):
        """
        매 렌더링 주기에 맞춰 센서 메타데이터를 업데이트합니다.
        (실제 PointCloud2 데이터는 Replicator C++/GPU 백그라운드 스레드에서 직접 ROS 2로 발행됨)
        """
        self._state = {
            "lidar_name": self._lidar_name,
            "stage_prim_path": self._stage_prim_path,
            "topic_name": self._topic_name,
            "frame_id": self._frame_id,
        }
        return self._state
