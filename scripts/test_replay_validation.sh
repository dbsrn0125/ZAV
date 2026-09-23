#!/bin/bash
set -e
source /opt/ros/humble/setup.bash
source /root/zenith_ws/install/setup.bash

LOG_FILE="/root/zenith_ws/validation_test.log"
rm -f "${LOG_FILE}"

echo "[TEST] Cleaning up previous nodes..."
pkill -9 -f "[f]astlivo_mapping" 2>/dev/null || true
pkill -9 -f "[r]os2 bag" 2>/dev/null || true
sleep 2

echo "[TEST] Starting FAST-LIVO2 mapping node..."
ros2 launch fast_livo zenith_corridor_scan.launch.py rviz:=False > "${LOG_FILE}" 2>&1 &
SLAM_PID=$!
echo "[TEST] SLAM PID: ${SLAM_PID}"

sleep 10

echo "[TEST] Starting bag playback (1.5x speed)..."
ros2 bag play /root/zenith_ws/bags/scan_sensors_20260914_135945 --rate 1.5 --topics /livox/lidar /livox/imu >> "${LOG_FILE}" 2>&1 || true

echo "[TEST] Bag playback finished. Checking status..."
sleep 5
kill ${SLAM_PID} 2>/dev/null || true
echo "[TEST] Complete!"
