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
    echo "  1) Pure Replay:  Play Bag directly and open RViz2"
    echo "  2) Offline SLAM: Re-run FAST-LIVO2 on Raw Sensors + RViz2 (Recommended)"
    echo "----------------------------------------------------------"
    read -t 10 -p "Select mode [1 or 2] (Default: 2 in 10s): " USER_MODE || USER_MODE="2"
    echo ""
    MODE_CHOICE="${USER_MODE:-2}"
fi

case "${MODE_CHOICE}" in
    1|--replay|-r)
        RUN_MODE="1"
        ;;
    2|--slam|-s)
        RUN_MODE="2"
        ;;
    *)
        RUN_MODE="2"
        ;;
esac

if [ "${RUN_MODE}" = "2" ]; then
    echo "[MODE 2] Offline FAST-LIVO2 SLAM with RViz2..."
    echo "[INFO] Running FAST-LIVO2 mapping node and RViz2..."
    echo "[INFO] Replaying raw sensor topics: /livox/lidar, /livox/imu, /camera/image_raw"
    echo "----------------------------------------------------------"
    
    docker exec -it zenith_dev /bin/bash -c "
        source /opt/ros/humble/setup.bash
        source /root/zenith_ws/install/setup.bash
        export DISPLAY=${DISPLAY:-:0}
        [ -f /root/zenith_ws/.docker_xauth ] && export XAUTHORITY=/root/zenith_ws/.docker_xauth

        # 1. Start FAST-LIVO2 mapping node with RViz2 in background
        ros2 launch fast_livo zenith_mapping.launch.py rviz:=True &
        SLAM_PID=\$!
        sleep 4

        # 2. Play bag with 1.0x real-time speed
        echo ''
        echo '=========================================================='
        echo '>>> [PLAYING BAG] Streaming sensor data into FAST-LIVO2... <<<'
        echo 'Rate: 1.0x (Real-time speed)'
        echo 'Press Ctrl+C to stop early.'
        echo '=========================================================='
        ros2 bag play ${DOCKER_BAG} --rate 1.0 || true

        echo ''
        echo '[INFO] Bag playback completed.'
        echo '[INFO] Waiting for user to close RViz2 window (or press Ctrl+C)...'
        wait \$SLAM_PID 2>/dev/null || true
    "
else
    echo "[MODE 1] Pure Bag Replay & RViz2..."
    echo "----------------------------------------------------------"

    docker exec -it zenith_dev /bin/bash -c "
        source /opt/ros/humble/setup.bash
        source /root/zenith_ws/install/setup.bash
        export DISPLAY=${DISPLAY:-:0}
        [ -f /root/zenith_ws/.docker_xauth ] && export XAUTHORITY=/root/zenith_ws/.docker_xauth

        # 1. Start RViz2 in background
        rviz2 -d /root/zenith_ws/src/FAST-LIVO2/rviz_cfg/fast_livo2.rviz &
        RVIZ_PID=\$!
        sleep 3

        # 2. Play bag in loop
        echo ''
        echo '=========================================================='
        echo '>>> [PLAYING BAG] Loop playback started... <<<'
        echo 'Press Ctrl+C to stop.'
        echo '=========================================================='
        ros2 bag play ${DOCKER_BAG} --loop || true

        kill \$RVIZ_PID 2>/dev/null || true
    "
fi

echo "=========================================================="
echo "[INFO] Playback finished."
echo "=========================================================="
