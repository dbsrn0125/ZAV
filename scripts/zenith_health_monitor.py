#!/usr/bin/env python3
import sys
for p in ['/opt/ros/humble/local/lib/python3.10/dist-packages', '/opt/ros/humble/lib/python3.10/site-packages', '/root/zenith_ws/install/livox_ros_driver2/local/lib/python3.10/dist-packages']:
    if p not in sys.path:
        sys.path.insert(0, p)

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, Imu
from livox_ros_driver2.msg import CustomMsg
import time
import os
import sys
import glob
import json
import shutil
import signal

class ZenithHealthMonitor(Node):
    def __init__(self):
        super().__init__('zenith_health_monitor')

        self.cam_count = 0
        self.lidar_count = 0
        self.imu_count = 0

        self.start_time = time.time()
        self.last_calc_time = time.time()
        
        self.last_cam_time = time.time()
        self.last_lidar_time = time.time()
        self.last_imu_time = time.time()

        self.cam_fps = 0.0
        import collections
        self.cam_history = collections.deque()
        self.lidar_history = collections.deque()
        self.imu_history = collections.deque()

        self.last_cam_time = 0.0
        self.last_lidar_time = 0.0
        self.last_imu_time = 0.0

        self.cam_fps = 0.0
        self.lidar_fps = 0.0
        self.imu_fps = 0.0

        from rclpy.callback_groups import ReentrantCallbackGroup
        self.cb_group = ReentrantCallbackGroup()

        from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
        
        imu_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1000,
            reliability=ReliabilityPolicy.BEST_EFFORT
        )
        sensor_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=50,
            reliability=ReliabilityPolicy.BEST_EFFORT
        )

        self.sub_cam = self.create_subscription(
            Image, '/camera/image_raw', self.cam_cb, sensor_qos, callback_group=self.cb_group
        )
        self.sub_lidar = self.create_subscription(
            CustomMsg, '/livox/lidar', self.lidar_cb, sensor_qos, callback_group=self.cb_group
        )
        self.sub_imu = self.create_subscription(
            Imu, '/livox/imu', self.imu_cb, imu_qos, callback_group=self.cb_group
        )

        self.status_file = "/root/zenith_ws/.sensor_status.json"
        self.temp_file = "/root/zenith_ws/.sensor_status.tmp"
        self.bags_dir = "/root/zenith_ws/bags"

        # Check and write status every 0.5s
        self.timer = self.create_timer(0.5, self.update_health, callback_group=self.cb_group)
        self.get_logger().info("Zenith Health Monitor initialized. Tracking Camera, LiDAR, IMU.")

    def cam_cb(self, msg):
        now = time.time()
        self.cam_history.append(now)
        self.last_cam_time = now

    def lidar_cb(self, msg):
        now = time.time()
        self.lidar_history.append(now)
        self.last_lidar_time = now

    def imu_cb(self, msg):
        now = time.time()
        self.imu_history.append(now)
        self.last_imu_time = now

    def get_latest_bag_size_mb(self):
        try:
            if not os.path.exists(self.bags_dir):
                return 0.0
            dirs = [os.path.join(self.bags_dir, d) for d in os.listdir(self.bags_dir)
                    if os.path.isdir(os.path.join(self.bags_dir, d))]
            if not dirs:
                return 0.0
            latest_dir = max(dirs, key=os.path.getmtime)
            total_size = 0
            for dirpath, dirnames, filenames in os.walk(latest_dir):
                for f in filenames:
                    fp = os.path.join(dirpath, f)
                    if os.path.exists(fp):
                        total_size += os.path.getsize(fp)
            return round(total_size / (1024 * 1024), 1)
        except Exception:
            return 0.0

    def update_health(self):
        now = time.time()
        window = 2.0  # 2.0 second moving average window

        # Prune old timestamps
        cutoff = now - window
        while self.cam_history and self.cam_history[0] < cutoff:
            self.cam_history.popleft()
        while self.lidar_history and self.lidar_history[0] < cutoff:
            self.lidar_history.popleft()
        while self.imu_history and self.imu_history[0] < cutoff:
            self.imu_history.popleft()

        # Calculate smooth FPS
        self.cam_fps = round(len(self.cam_history) / window, 1)
        self.lidar_fps = round(len(self.lidar_history) / window, 1)
        self.imu_fps = round(len(self.imu_history) / window, 1)

        uptime = now - self.start_time
        is_init = (uptime < 7.0)  # Grace period (7s) for Livox/Camera link establishment

        cam_ok = True
        lidar_ok = True
        imu_ok = True
        alert_level = "ok"
        alerts = []

        if not is_init:
            # Camera health check (timeout 3.0s or rate < 3 Hz)
            if (self.last_cam_time == 0.0) or (now - self.last_cam_time > 3.0) or (self.cam_fps < 3.0):
                cam_ok = False
                alert_level = "error"
                alerts.append(f"📷 카메라 영상 중단 ({self.cam_fps} Hz)")

            # LiDAR health check (timeout 3.0s or rate < 3 Hz)
            if (self.last_lidar_time == 0.0) or (now - self.last_lidar_time > 3.0) or (self.lidar_fps < 3.0):
                lidar_ok = False
                alert_level = "error"
                alerts.append(f"📡 라이다 포인트 중단 ({self.lidar_fps} Hz)")

            # IMU health check (timeout 2.0s or rate < 30 Hz)
            if (self.last_imu_time == 0.0) or (now - self.last_imu_time > 2.0) or (self.imu_fps < 30.0):
                imu_ok = False
                alert_level = "error"
                alerts.append(f"🧭 IMU 신호 중단 ({self.imu_fps} Hz)")
        else:
            # During startup grace period
            cam_ok = (self.last_cam_time > 0.0)
            lidar_ok = (self.last_lidar_time > 0.0)
            imu_ok = (self.last_imu_time > 0.0)

        # Check disk space
        disk_free_gb = 0.0
        try:
            disk_free_gb = round(shutil.disk_usage("/root/zenith_ws").free / (1024**3), 1)
            if disk_free_gb < 1.0:
                alert_level = "error"
                alerts.append(f"💾 용량 부족 ({disk_free_gb} GB 남음)")
            elif disk_free_gb < 2.0 and alert_level != "error":
                alert_level = "warning"
                alerts.append(f"💾 용량 주의 ({disk_free_gb} GB 남음)")
        except Exception:
            pass

        # Check RAM and CPU stats
        mem_total_gb = 11.3
        mem_avail_gb = 11.3
        mem_used_gb = 0.0
        mem_used_pct = 0.0
        try:
            with open("/proc/meminfo") as mf:
                mem_kbs = {}
                for line in mf:
                    parts = line.split()
                    if len(parts) >= 2:
                        mem_kbs[parts[0]] = int(parts[1])
                t_kb = mem_kbs.get("MemTotal:", 0)
                a_kb = mem_kbs.get("MemAvailable:", 0)
                if t_kb > 0:
                    mem_total_gb = round(t_kb / (1024 * 1024), 1)
                    mem_avail_gb = round(a_kb / (1024 * 1024), 1)
                    mem_used_gb = round(mem_total_gb - mem_avail_gb, 1)
                    mem_used_pct = round(((t_kb - a_kb) / t_kb) * 100.0, 1)
        except Exception:
            pass

        cpu_temp_c = 0.0
        try:
            with open("/sys/class/thermal/thermal_zone0/temp") as tf:
                cpu_temp_c = round(int(tf.read().strip()) / 1000.0, 1)
        except Exception:
            pass

        cpu_usage_pct = 0.0
        try:
            import psutil
            cpu_usage_pct = round(psutil.cpu_percent(interval=None), 1)
        except Exception:
            pass

        if not is_init:
            if mem_avail_gb < 0.8:
                alert_level = "error"
                alerts.append(f"🧠 RAM 위험 ({int(mem_avail_gb * 1024)}MB 남음)")
            elif mem_avail_gb < 1.8 and alert_level != "error":
                alert_level = "warning"
                alerts.append(f"🧠 RAM 주의 ({mem_avail_gb}GB 남음)")

            if mem_avail_gb < 2.2:
                try:
                    os.system("sync; echo 1 > /proc/sys/vm/drop_caches 2>/dev/null &")
                except Exception:
                    pass

            if cpu_temp_c >= 85.0:
                alert_level = "error"
                alerts.append(f"🔥 CPU 과열 ({int(cpu_temp_c)}°C)")
            elif cpu_temp_c >= 78.0 and alert_level != "error":
                alert_level = "warning"
                alerts.append(f"🔥 CPU 고온 ({int(cpu_temp_c)}°C)")

        mem_ok = (mem_avail_gb >= 1.0)
        cpu_ok = (cpu_temp_c < 85.0)
        mem_status = "위험" if mem_avail_gb < 0.8 else ("주의" if mem_avail_gb < 1.8 else "정상")
        cpu_status = "과열" if cpu_temp_c >= 85.0 else ("주의" if cpu_temp_c >= 78.0 else "정상")

        bag_mb = self.get_latest_bag_size_mb()

        # Build payload
        alert_msg = " / ".join(alerts) if alerts else None
        overall_ok = (cam_ok and lidar_ok and imu_ok and mem_ok and (disk_free_gb >= 1.0))

        status_data = {
            "updated_at": now,
            "uptime": round(uptime, 1),
            "state": "INITIALIZING" if is_init else "ACTIVE",
            "overall_ok": overall_ok,
            "alert_level": alert_level,
            "alert_msg": alert_msg,
            "camera": {
                "ok": cam_ok,
                "fps": self.cam_fps,
                "status": "정상" if cam_ok else ("연결 중" if is_init else "중단")
            },
            "lidar": {
                "ok": lidar_ok,
                "fps": self.lidar_fps,
                "status": "정상" if lidar_ok else ("연결 중" if is_init else "중단")
            },
            "imu": {
                "ok": imu_ok,
                "fps": self.imu_fps,
                "status": "정상" if imu_ok else ("연결 중" if is_init else "중단")
            },
            "memory": {
                "ok": mem_ok,
                "total_gb": mem_total_gb,
                "used_gb": mem_used_gb,
                "avail_gb": mem_avail_gb,
                "percent": mem_used_pct,
                "status": mem_status
            },
            "cpu": {
                "ok": cpu_ok,
                "percent": cpu_usage_pct,
                "temp_c": cpu_temp_c,
                "cores": 8,
                "status": cpu_status
            },
            "bag": {
                "ok": True,
                "size_mb": bag_mb,
                "status": "기록 중"
            },
            "disk_free_gb": disk_free_gb
        }

        # Atomic write to JSON
        try:
            with open(self.temp_file, "w") as f:
                json.dump(status_data, f, indent=2)
            os.replace(self.temp_file, self.status_file)
            try:
                os.chmod(self.status_file, 0o666)
            except Exception:
                pass
        except Exception as e:
            self.get_logger().warn(f"Failed to write sensor status: {e}")

    def clean_exit(self):
        try:
            status_data = {
                "updated_at": time.time(),
                "state": "IDLE",
                "overall_ok": True,
                "alert_level": "ok",
                "alert_msg": None
            }
            with open(self.temp_file, "w") as f:
                json.dump(status_data, f, indent=2)
            os.replace(self.temp_file, self.status_file)
            try:
                os.chmod(self.status_file, 0o666)
            except Exception:
                pass
        except Exception:
            pass

def main(args=None):
    rclpy.init(args=args)
    monitor = ZenithHealthMonitor()

    def sig_handler(sig, frame):
        monitor.clean_exit()
        monitor.destroy_node()
        rclpy.shutdown()
        sys.exit(0)

    signal.signal(signal.SIGINT, sig_handler)
    signal.signal(signal.SIGTERM, sig_handler)

    from rclpy.executors import MultiThreadedExecutor
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(monitor)

    try:
        executor.spin()
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        monitor.clean_exit()
        if rclpy.ok():
            monitor.destroy_node()
            rclpy.shutdown()

if __name__ == '__main__':
    main()
