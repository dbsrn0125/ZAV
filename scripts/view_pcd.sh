#!/bin/bash
set -e

WORKSPACE_DIR="/home/radxa/zenith_ws"
SCANS_DIR="${WORKSPACE_DIR}/scans"

TARGET_PCD="$1"

# If no file specified, find the most recent PCD file in scans/ or Log/PCD/
if [ -z "${TARGET_PCD}" ]; then
    TARGET_PCD=$(ls -t ${SCANS_DIR}/*.pcd 2>/dev/null | head -n 1 || true)
    if [ -z "${TARGET_PCD}" ]; then
        TARGET_PCD="${WORKSPACE_DIR}/src/FAST-LIVO2/Log/PCD/all_downsampled_points.pcd"
    fi
fi

if [ ! -f "${TARGET_PCD}" ]; then
    echo "[ERROR] No PCD file found at: ${TARGET_PCD}"
    echo "Usage: $0 [path_to_pcd_file]"
    exit 1
fi

echo "=========================================================="
echo "    Zenith Drone: 3D Point Cloud Map Viewer (RViz2)"
echo "=========================================================="
echo "Opening 3D Map: ${TARGET_PCD}"
echo "Size: $(du -h "${TARGET_PCD}" | cut -f1)"
echo "----------------------------------------------------------"
echo "Tips in RViz2:"
echo "- Left Click + Drag: Rotate 3D view"
echo "- Right Click + Drag / Scroll: Zoom In / Out"
echo "- Middle Click + Drag: Pan"
echo "=========================================================="

# Convert host path to docker container path
DOCKER_PCD="/root/zenith_ws/${TARGET_PCD#${WORKSPACE_DIR}/}"

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

# Publish PCD and view in RViz
docker exec -it zenith_dev /bin/bash -c "
source /opt/ros/humble/setup.bash
source /root/zenith_ws/install/setup.bash
export DISPLAY=${DISPLAY:-:0}
[ -f /root/zenith_ws/.docker_xauth ] && export XAUTHORITY=/root/zenith_ws/.docker_xauth

ros2 run pcl_ros pcd_to_pointcloud --ros-args -p file_name:=${DOCKER_PCD} -p tf_frame:=camera_init -r cloud_pcd:=/cloud_registered &
PCD_PUB_PID=\$!

rviz2 -d /root/zenith_ws/src/FAST-LIVO2/rviz_cfg/fast_livo2.rviz

kill \$PCD_PUB_PID 2>/dev/null || true
"
