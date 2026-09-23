#!/usr/bin/env python3
"""
비행 전 센서 자동 점검 스크립트
카메라 / IMU / LiDAR 의 Hz를 30초간 측정하여 기준치 충족 여부를 출력합니다.
사용법: docker exec zenith_dev bash -c "source /opt/ros/humble/setup.bash && source /root/zenith_ws/install/setup.bash && python3 /root/zenith_ws/scripts/preflight_check.py"
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image, Imu
import sys
import time

MEASURE_DURATION = 30.0
WARMUP_DURATION  = 3.0

THRESHOLDS = {
    "/camera/image_raw": 9.0,
    "/livox/imu":       180.0,
    "/livox/lidar":       9.0,
}

class SensorChecker(Node):
    def __init__(self):
        super().__init__("preflight_sensor_checker")
        self.counts = {t: 0 for t in THRESHOLDS}
        self.start_time = None
        self.warmup_done = False
        self.measure_start = None

        rel_qos = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                             history=HistoryPolicy.KEEP_LAST, depth=10)
        be_qos  = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                             history=HistoryPolicy.KEEP_LAST, depth=10)

        self.create_subscription(Image, "/camera/image_raw", self._cam_cb,   rel_qos)
        self.create_subscription(Imu,   "/livox/imu",        self._imu_cb,   be_qos)

        # Livox CustomMsg - import lazily to handle missing package
        try:
            from livox_ros_driver2.msg import CustomMsg
            self.create_subscription(CustomMsg, "/livox/lidar", self._lidar_cb, be_qos)
        except ImportError:
            self.get_logger().warn("livox_ros_driver2 not found, skipping /livox/lidar check")

        self.get_logger().info(f"워밍업 {WARMUP_DURATION:.0f}초 후 {MEASURE_DURATION:.0f}초 측정 시작...")

    def _now(self):
        return self.get_clock().now().nanoseconds / 1e9

    def _tick(self, topic):
        t = self._now()
        if not self.warmup_done:
            if self.start_time is None:
                self.start_time = t
            if t - self.start_time >= WARMUP_DURATION:
                self.warmup_done = True
                self.measure_start = t
                self.get_logger().info("측정 시작!")
            return
        self.counts[topic] += 1

    def _cam_cb(self, _):   self._tick("/camera/image_raw")
    def _imu_cb(self, _):   self._tick("/livox/imu")
    def _lidar_cb(self, _): self._tick("/livox/lidar")

    def is_done(self):
        if not self.warmup_done or self.measure_start is None:
            return False
        return (self._now() - self.measure_start) >= MEASURE_DURATION

    def report(self):
        elapsed = self._now() - self.measure_start
        print("\n" + "="*55)
        print("    PRE-FLIGHT SENSOR CHECK RESULTS")
        print("="*55)
        all_ok = True
        for topic, threshold in THRESHOLDS.items():
            hz = self.counts[topic] / elapsed
            ok = hz >= threshold
            status = "OK  " if ok else "FAIL"
            if not ok:
                all_ok = False
            print(f"  [{status}]  {topic:<22s}  {hz:6.1f} Hz  (min {threshold:.0f})")
        print("-"*55)
        if all_ok:
            print("  >> ALL SENSORS HEALTHY - 비행 가능 <<")
        else:
            print("  >> SENSOR FAILURE - 비행 불가 <<")
        print("="*55 + "\n")
        return all_ok

def main():
    rclpy.init()
    node = SensorChecker()
    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(node)
    try:
        while rclpy.ok() and not node.is_done():
            executor.spin_once(timeout_sec=0.1)
    except KeyboardInterrupt:
        print("\n[중단됨]")
    ok = node.report()
    node.destroy_node()
    rclpy.shutdown()
    sys.exit(0 if ok else 1)

if __name__ == "__main__":
    main()
