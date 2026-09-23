#!/bin/bash
set -e

WORKSPACE_DIR="/home/radxa/zenith_ws"
BAGS_DIR="${WORKSPACE_DIR}/bags"

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

# 1. Authorize Docker container to connect to host X11 display
if command -v xhost >/dev/null 2>&1; then
    xhost +local:root >/dev/null 2>&1 || xhost + >/dev/null 2>&1 || true
fi

# 2. Share Xauthority token with Docker container
XAUTH_FILE="${WORKSPACE_DIR}/.docker_xauth"
if [ -n "${XAUTHORITY}" ] && [ -f "${XAUTHORITY}" ]; then
    cp "${XAUTHORITY}" "${XAUTH_FILE}" 2>/dev/null || true
    chmod 666 "${XAUTH_FILE}" 2>/dev/null || true
elif [ -f "${HOME}/.Xauthority" ]; then
    cp "${HOME}/.Xauthority" "${XAUTH_FILE}" 2>/dev/null || true
    chmod 666 "${XAUTH_FILE}" 2>/dev/null || true
else
    MUTTER_AUTH=$(find /run/user/$(id -u) -name ".*auth*" 2>/dev/null | head -n 1 || true)
    if [ -n "${MUTTER_AUTH}" ] && [ -f "${MUTTER_AUTH}" ]; then
        cp "${MUTTER_AUTH}" "${XAUTH_FILE}" 2>/dev/null || true
        chmod 666 "${XAUTH_FILE}" 2>/dev/null || true
    fi
fi

ARG_BAG="$1"
ARG_MODE="$2"
ARG_RATE="${3:-1.0}"

echo "=========================================================="
echo "    Zenith Drone: ROS 2 Bag Replay & FAST-LIVO2 RViz2"
echo "=========================================================="

# Resolve Bag Directory
TARGET_BAG=""
if [ -n "${ARG_BAG}" ]; then
    if [ -d "${ARG_BAG}" ]; then
        TARGET_BAG="${ARG_BAG}"
    elif [ -d "${BAGS_DIR}/${ARG_BAG}" ]; then
        TARGET_BAG="${BAGS_DIR}/${ARG_BAG}"
    else
        # Match partial keyword (e.g. timestamp "101118")
        MATCH=$(ls -td ${BAGS_DIR}/*${ARG_BAG}* 2>/dev/null | head -n 1 || true)
        if [ -n "${MATCH}" ] && [ -d "${MATCH}" ]; then
            TARGET_BAG="${MATCH}"
        fi
    fi
fi

# If no target found or provided, show list of available bags
if [ -z "${TARGET_BAG}" ]; then
    AVAILABLE_BAGS=($(ls -td ${BAGS_DIR}/*/ 2>/dev/null || true))
    if [ ${#AVAILABLE_BAGS[@]} -eq 0 ]; then
        echo "[ERROR] No ROS 2 bags found in: ${BAGS_DIR}"
        echo "Please record a scan first using: ${WORKSPACE_DIR}/scripts/run_corridor_scan.sh"
        exit 1
    fi

    echo "Available ROS 2 Bags in ${BAGS_DIR}:"
    idx=1
    for b in "${AVAILABLE_BAGS[@]}"; do
        bname=$(basename "${b}")
        bsize=$(du -sh "${b}" 2>/dev/null | cut -f1)
        if [ ${idx} -eq 1 ]; then
            echo "  [${idx}] ${bname} (${bsize}) <- [Latest]"
        else
            echo "  [${idx}] ${bname} (${bsize})"
        fi
        idx=$((idx + 1))
    done
    echo "----------------------------------------------------------"
    read -t 10 -p "Select bag [1-${#AVAILABLE_BAGS[@]}] (Default: 1 in 10s): " BAG_SELECTION || BAG_SELECTION="1"
    echo ""
    BAG_SELECTION="${BAG_SELECTION:-1}"

    # Check if number was selected
    if [[ "${BAG_SELECTION}" =~ ^[0-9]+$ ]] && [ "${BAG_SELECTION}" -le "${#AVAILABLE_BAGS[@]}" ] && [ "${BAG_SELECTION}" -ge 1 ]; then
        TARGET_BAG="${AVAILABLE_BAGS[$((BAG_SELECTION - 1))]}"
    else
        TARGET_BAG="${AVAILABLE_BAGS[0]}"
    fi
fi

TARGET_BAG="${TARGET_BAG%/}" # Remove trailing slash
BAG_NAME=$(basename "${TARGET_BAG}")
DOCKER_BAG="/root/zenith_ws/bags/${BAG_NAME}"

echo "[SELECTED BAG] ${BAG_NAME}"
echo "[FOLDER PATH]  ${TARGET_BAG}"
echo "[SIZE]         $(du -sh "${TARGET_BAG}" | cut -f1)"
echo "----------------------------------------------------------"

# Resolve Execution Mode
MODE_CHOICE="${ARG_MODE}"
if [ -z "${MODE_CHOICE}" ]; then
    echo "Select Playback Mode:"
    echo "  1) Pure Replay:      Play Bag directly and open RViz2"
    echo "  2) LIVO SLAM (RGB):  FAST-LIVO2 (LiDAR + RGB Camera, Default)"
    echo "  3) Pure LIO (LiDAR): FAST-LIVO2 (LiDAR + IMU only, Full 10min scan)"
    echo "----------------------------------------------------------"
    read -t 10 -p "Select mode [1, 2, or 3] (Default: 2 in 10s): " USER_MODE || USER_MODE="2"
    echo ""
    MODE_CHOICE="${USER_MODE:-2}"
fi

case "${MODE_CHOICE}" in
    1|--replay|-r)
        RUN_MODE="1"
        ;;
    2|--slam|-s|--livo)
        RUN_MODE="2"
        ;;
    3|--lio|-l|--lidar)
        RUN_MODE="3"
        ;;
    *)
        RUN_MODE="2"
        ;;
esac

# Determine interactive flag
DOCKER_FLAGS="-i"
if [ -t 0 ] && [ -t 1 ]; then
    DOCKER_FLAGS="-it"
fi

# Clean up any lingering live sensor nodes from host before replay
docker exec zenith_dev pkill -9 -f "[l]ivox_ros_driver2" 2>/dev/null || true
docker exec zenith_dev pkill -9 -f "[m]vs_camera_node" 2>/dev/null || true
docker exec zenith_dev pkill -9 -f "[f]astlivo_mapping" 2>/dev/null || true
docker exec zenith_dev pkill -9 -f "[r]viz2" 2>/dev/null || true
docker exec zenith_dev pkill -9 -f "[r]osbag2" 2>/dev/null || true
docker exec zenith_dev pkill -9 -f "[r]os2 bag play" 2>/dev/null || true
sleep 1

if [ "${RUN_MODE}" = "2" ]; then
    echo "[MODE 2] Offline FAST-LIVO2 LIVO SLAM (LiDAR + Camera RGB) with RViz2..."
    echo "[INFO] Running FAST-LIVO2 mapping node and RViz2..."
    echo "[INFO] Replaying raw sensor topics: /livox/lidar, /livox/imu, /camera/image_raw"
    echo "----------------------------------------------------------"
elif [ "${RUN_MODE}" = "3" ]; then
    echo "[MODE 3] Offline FAST-LIVO2 Pure LIO SLAM (LiDAR + IMU only) with RViz2..."
    echo "[INFO] Running FAST-LIVO2 mapping node (Camera Disabled) and RViz2..."
    echo "[INFO] Replaying raw sensor topics: /livox/lidar, /livox/imu"
    echo "----------------------------------------------------------"
else
    echo "[MODE 1] Pure Bag Replay & RViz2..."
    echo "----------------------------------------------------------"
fi

docker exec ${DOCKER_FLAGS} \
    -e DOCKER_BAG="${DOCKER_BAG}" \
    -e ARG_RATE="${ARG_RATE}" \
    -e RUN_MODE="${RUN_MODE}" \
    -e DISPLAY="${DISPLAY:-:0}" \
    zenith_dev /root/zenith_ws/scripts/docker_play.sh

PCD_SRC="${WORKSPACE_DIR}/src/FAST-LIVO2/Log/PCD/all_downsampled_points.pcd"
if [ -f "${PCD_SRC}" ] && [ -s "${PCD_SRC}" ]; then
    DEST_PCD="${WORKSPACE_DIR}/scans/${BAG_NAME}_color.pcd"
    cp "${PCD_SRC}" "${DEST_PCD}"
    echo ""
    echo "[SUCCESS] 3D RGB Color PCD Map saved to:"
    echo "          ${DEST_PCD}"
    echo "          Size: $(du -h "${DEST_PCD}" | cut -f1)"
fi

echo "=========================================================="
echo "[INFO] Playback finished."
echo "=========================================================="
