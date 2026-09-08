#!/bin/bash
set -e

WORKSPACE_DIR="/home/radxa/zenith_ws"

# Ensure docker container is running
if ! docker ps --format '{{.Names}}' | grep -q "^zenith_dev$"; then
    echo "[INFO] Starting zenith_dev docker container..."
    if docker ps -aq -f name="^zenith_dev$" | grep -q .; then
        docker start zenith_dev >/dev/null
    else
        echo "[ERROR] zenith_dev container does not exist. Please run ${WORKSPACE_DIR}/docker/run_container.sh first."
        exit 1
    fi
    sleep 2
fi

# Allow local X11 access for Docker GUI windows
xhost +local:root >/dev/null 2>&1 || true

echo "=========================================================="
echo "    Zenith Drone: LiDAR-Camera Extrinsic Calibration     "
echo "=========================================================="
echo "This tool aligns Livox Mid-360 LiDAR and Hikrobot Camera."
echo ""
echo "1. Place the drone stationary on a table or tripod."
echo "2. Point both sensors at an object with clear corners"
echo "   (e.g., Computer monitor, doorway, desk corner, or box 1~2m away)."
echo "3. Press [Enter] to capture a synchronized snapshot (image + 3D cloud)."
echo "4. Click corresponding corners in Camera window, then in 3D PointCloud."
echo "=========================================================="
echo ""
read -p "Press [Enter] when the drone is positioned and ready: "

docker exec -it zenith_dev /bin/bash -c "
source /opt/ros/humble/setup.bash
source /root/zenith_ws/install/setup.bash

echo '[1/4] Starting Camera and LiDAR drivers...'
ros2 launch mvs_ros2_driver zenith_camera.launch.py >/dev/null 2>&1 &
CAM_PID=\$!

ros2 launch livox_ros_driver2 msg_MID360s_launch.py >/dev/null 2>&1 &
LIDAR_PID=\$!

echo '[2/4] Waiting 3s for sensor streams to stabilize...'
sleep 3

echo '[3/4] Launching Extrinsic Calibrator...'
python3 /root/zenith_ws/scripts/calibrate_lidar_camera.py --live || true

echo '[4/4] Shutting down sensor drivers...'
kill \$CAM_PID \$LIDAR_PID 2>/dev/null || true
"

echo ""
echo "=========================================================="
echo " Calibration session ended. Check zenith_mid360s.yaml"
echo "=========================================================="
