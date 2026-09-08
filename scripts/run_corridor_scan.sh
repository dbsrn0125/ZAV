#!/bin/bash
set -e

WORKSPACE_DIR="/home/radxa/zenith_ws"
SCANS_DIR="${WORKSPACE_DIR}/scans"
BAGS_DIR="${WORKSPACE_DIR}/bags"
mkdir -p "${SCANS_DIR}" "${BAGS_DIR}"

# Parse command line argument or prompt
MODE="$1"
if [ -z "${MODE}" ]; then
    echo "=========================================================="
    echo "    Zenith Drone: Mobile Corridor 3D Scan Pipeline"
    echo "=========================================================="
    echo "Select ROS 2 Bag Recording Option:"
    echo "  1) [Recommended] Record Raw Sensors (/livox/lidar, /livox/imu, /camera/image_raw)"
    echo "  2) Record Full SLAM (Sensors + Points + Trajectory + Odometry)"
    echo "  3) No Bag (Only save PCD 3D map)"
    echo "----------------------------------------------------------"
    read -t 5 -p "Select option [1-3] (Default: 1 in 5s): " USER_INPUT || USER_INPUT="1"
    echo ""
    MODE="${USER_INPUT:-1}"
fi

case "${MODE}" in
    1|--sensors|-s)
        BAG_MODE="sensors"
        RECORD_TOPICS="/livox/lidar /livox/imu /camera/image_raw"
        ;;
    2|--all|-a)
        BAG_MODE="all"
        RECORD_TOPICS="/livox/lidar /livox/imu /camera/image_raw /cloud_registered /aft_mapped_to_init /path /Laser_map"
        ;;
    3|--no-bag|-n)
        BAG_MODE="none"
        ;;
    *)
        BAG_MODE="sensors"
        RECORD_TOPICS="/livox/lidar /livox/imu /camera/image_raw"
        ;;
esac

TIMESTAMP=$(date +%Y%m%d_%H%M%S)
BAG_NAME="scan_${BAG_MODE}_${TIMESTAMP}"
DOCKER_BAG_PATH="/root/zenith_ws/bags/${BAG_NAME}"

echo "=========================================================="
echo "    Zenith Drone: Mobile Corridor 3D Scan Pipeline"
echo "=========================================================="
echo "[1/3] Camera:   Hikrobot MV-CU013 (10Hz Internal Clock)"
echo "[2/3] LiDAR:    Livox Mid-360S (10Hz PTP/ROS Clock)"
echo "[3/3] SLAM:     FAST-LIVO2 Headless Mode (PCD Auto-save ON)"
if [ "${BAG_MODE}" != "none" ]; then
    echo "[BAG] Recording: ${BAG_NAME} (${BAG_MODE})"
else
    echo "[BAG] Recording: Disabled (PCD only)"
fi
echo "----------------------------------------------------------"
echo "Press Ctrl+C when you return from your corridor walk."
echo "The 3D map will be automatically saved to: ${SCANS_DIR}/"
echo "=========================================================="
echo ""

# Trap Ctrl+C (SIGINT) to ensure post-scan archiving routine runs
trap 'echo ""; echo "[INFO] Scan terminated by user. Finalizing Bag and 3D map..."' INT

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

# Start bag recording in background if enabled
if [ "${BAG_MODE}" != "none" ]; then
    echo "[INFO] Starting background ROS 2 bag recorder..."
    docker exec -d zenith_dev /bin/bash -c "source /opt/ros/humble/setup.bash && source /root/zenith_ws/install/setup.bash && exec ros2 bag record -o ${DOCKER_BAG_PATH} ${RECORD_TOPICS}"
    sleep 1
fi

# Run unified corridor scan launch file inside container
docker exec -it zenith_dev /bin/bash -c "source /opt/ros/humble/setup.bash && source /root/zenith_ws/install/setup.bash && ros2 launch fast_livo zenith_corridor_scan.launch.py rviz:=False" || true

echo ""
echo "[POST-SCAN] Scan stopped. Processing and archiving data..."

# Stop bag recorder cleanly with SIGINT (signal 2)
if [ "${BAG_MODE}" != "none" ]; then
    docker exec zenith_dev pkill -2 -f 'ros2 bag record' 2>/dev/null || true
    sleep 2
fi

docker exec zenith_dev /bin/bash -c "chmod -R 777 /root/zenith_ws/src/FAST-LIVO2/Log/PCD /root/zenith_ws/scans /root/zenith_ws/bags 2>/dev/null || true"

PCD_SRC="${WORKSPACE_DIR}/src/FAST-LIVO2/Log/PCD/all_downsampled_points.pcd"
RAW_PCD_SRC="${WORKSPACE_DIR}/src/FAST-LIVO2/Log/PCD/all_raw_points.pcd"
DEST_PCD="${SCANS_DIR}/corridor_${TIMESTAMP}.pcd"

echo ""
echo "=========================================================="
echo "               >>> SCAN ARCHIVE SUMMARY <<<"
echo "=========================================================="

if [ -f "${PCD_SRC}" ] && [ -s "${PCD_SRC}" ]; then
    cp "${PCD_SRC}" "${DEST_PCD}"
    echo "[1] 3D PCD Map Saved:"
    echo "    File: ${DEST_PCD}"
    echo "    Size: $(du -h "${DEST_PCD}" | cut -f1)"
elif [ -f "${RAW_PCD_SRC}" ] && [ -s "${RAW_PCD_SRC}" ]; then
    cp "${RAW_PCD_SRC}" "${DEST_PCD}"
    echo "[1] Raw 3D Map Saved:"
    echo "    File: ${DEST_PCD}"
    echo "    Size: $(du -h "${DEST_PCD}" | cut -f1)"
else
    echo "[WARNING] No PCD file found in ${PCD_SRC}."
fi

if [ "${BAG_MODE}" != "none" ]; then
    HOST_BAG_PATH="${BAGS_DIR}/${BAG_NAME}"
    if [ -d "${HOST_BAG_PATH}" ]; then
        echo ""
        echo "[2] ROS 2 Bag Saved:"
        echo "    Folder: ${HOST_BAG_PATH}"
        echo "    Size:   $(du -sh "${HOST_BAG_PATH}" | cut -f1)"
        echo "    Replay: ${WORKSPACE_DIR}/scripts/play_bag.sh ${HOST_BAG_PATH}"
    fi
fi
echo "=========================================================="
echo ""
