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

echo "=========================================================="
echo "    Zenith Drone: Camera Intrinsic Auto-Calibration"
echo "=========================================================="
echo "1. Display the 9x6 Chessboard on your monitor full-screen."
echo "2. Hold the drone/camera ~0.5m to 1.5m away from the monitor."
echo "3. Slowly move and tilt the camera around:"
echo "   - Center view"
echo "   - Tilt left / right (roll & yaw)"
echo "   - Tilt up / down (pitch)"
echo "   - Move to corners of the image (essential for distortion!)"
echo "4. The script will automatically capture 20 valid frames (~15 sec) and"
echo "   calculate the exact distortion coefficients & focal lengths."
echo "=========================================================="
echo ""
read -p "Press [Enter] when the chessboard is displayed on your monitor to start: "

docker exec -i zenith_dev /bin/bash -c "
source /opt/ros/humble/setup.bash
source /root/zenith_ws/install/setup.bash

# Clean previous calibration images
rm -rf /root/zenith_ws/calib_images /root/zenith_ws/calib_annotated
mkdir -p /root/zenith_ws/calib_images /root/zenith_ws/calib_annotated

# 1. Start Camera Node in background (silence raw driver logs)
ros2 launch mvs_ros2_driver zenith_camera.launch.py >/dev/null 2>&1 &
CAM_PID=\$!
trap 'pkill -9 -f "mvs_camera|mvs_ros2_driver|auto_camera_calib" 2>/dev/null || true' INT TERM EXIT
sleep 3

# 2. Run auto-calibrator
python3 /root/zenith_ws/scripts/auto_camera_calib.py || true

# Cleanup
pkill -9 -f "mvs_camera|mvs_ros2_driver" 2>/dev/null || true
"

echo ""
echo "=========================================================="
echo "[INFO] Camera calibration process completed."
echo "=========================================================="
