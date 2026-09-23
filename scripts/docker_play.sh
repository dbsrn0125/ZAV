#!/bin/bash
set -e

source /opt/ros/humble/setup.bash
source /root/zenith_ws/install/setup.bash

export DISPLAY="${DISPLAY:-:0}"
export XDG_RUNTIME_DIR=/tmp/runtime-root
mkdir -p /tmp/runtime-root && chmod 700 /tmp/runtime-root
[ -f /root/zenith_ws/.docker_xauth ] && export XAUTHORITY=/root/zenith_ws/.docker_xauth

# Clean up any lingering processes inside container
pkill -9 -f "[f]astlivo_mapping" 2>/dev/null || true
pkill -9 -f "[r]viz2" 2>/dev/null || true
pkill -9 -f "[r]osbag2" 2>/dev/null || true
pkill -9 -f "[r]os2 bag play" 2>/dev/null || true
pkill -9 -f "[l]ivox_ros_driver2" 2>/dev/null || true
pkill -9 -f "[m]vs_camera_node" 2>/dev/null || true
sleep 1

# Check mode
if [ "${RUN_MODE}" = "2" ] || [ "${RUN_MODE}" = "3" ] || [ "${RUN_MODE}" = "4" ] || [ "${RUN_MODE}" = "5" ]; then
    ENABLE_CAM="True"
    ENABLE_RVIZ="True"

    if [ "${RUN_MODE}" = "3" ] || [ "${RUN_MODE}" = "5" ]; then
        ENABLE_CAM="False"
    fi

    if [ "${RUN_MODE}" = "4" ] || [ "${RUN_MODE}" = "5" ]; then
        ENABLE_RVIZ="False"
    fi

    echo "[DOCKER] Launching FAST-LIVO2 (camera:=${ENABLE_CAM}, rviz:=${ENABLE_RVIZ})..."
    ros2 launch fast_livo zenith_mapping.launch.py camera:=${ENABLE_CAM} rviz:=${ENABLE_RVIZ} </dev/null &
    SLAM_PID=$!

    cleanup() {
        echo ""
        echo "[DOCKER] Shutting down FAST-LIVO2 and saving PCD map..."
        pkill -2 -f "[f]astlivo_mapping" 2>/dev/null || true
        sleep 3
        kill $SLAM_PID 2>/dev/null || true
        pkill -9 -f "[f]astlivo_mapping" 2>/dev/null || true
        pkill -9 -f "[r]viz2" 2>/dev/null || true
        pkill -9 -f "[r]osbag2" 2>/dev/null || true
        pkill -9 -f "[r]os2 bag play" 2>/dev/null || true
        exit 0
    }
    trap cleanup INT TERM EXIT

    # Wait for laserMapping (and RViz2 if enabled) to be initialized
    sleep 4

    echo ""
    echo "=========================================================="
    echo ">>> [PLAYING BAG] Streaming sensor data into FAST-LIVO2... <<<"
    echo "Rate:     ${ARG_RATE}x speed"
    echo "Mode:     ${RUN_MODE} (camera=${ENABLE_CAM}, rviz=${ENABLE_RVIZ})"
    echo "Bag:      ${DOCKER_BAG}"
    echo "Press Ctrl+C to stop early."
    echo "=========================================================="

    ros2 bag play "${DOCKER_BAG}" --rate "${ARG_RATE}" --disable-keyboard-controls || true

    echo ""
    echo "=========================================================="
    echo "[SUCCESS] Bag playback completed!"
    if [ "${ENABLE_RVIZ}" = "True" ]; then
        echo "[INFO] RViz2 is still open showing the complete 3D map."
        echo "[INFO] Close the RViz2 window or press Ctrl+C to exit."
        echo "=========================================================="
        wait $SLAM_PID 2>/dev/null || true
    else
        echo "[INFO] Headless mode: Finalizing FAST-LIVO2 and saving PCD map..."
        echo "=========================================================="
        sleep 3
    fi
    trap - INT TERM EXIT
    cleanup

else
    echo "[DOCKER] Launching RViz2 for pure playback..."
    rviz2 -d /root/zenith_ws/src/FAST-LIVO2/rviz_cfg/fast_livo2.rviz </dev/null &
    RVIZ_PID=$!

    cleanup() {
        echo ""
        echo "[DOCKER] Closing RViz2..."
        kill $RVIZ_PID 2>/dev/null || true
        pkill -9 -f "[r]viz2" 2>/dev/null || true
        pkill -9 -f "[r]osbag2" 2>/dev/null || true
        pkill -9 -f "[r]os2 bag play" 2>/dev/null || true
        exit 0
    }
    trap cleanup INT TERM EXIT

    sleep 3

    echo ""
    echo "=========================================================="
    echo ">>> [PLAYING BAG] Loop playback started... <<<"
    echo "Bag:  ${DOCKER_BAG}"
    echo "Press Ctrl+C to stop."
    echo "=========================================================="
    ros2 bag play "${DOCKER_BAG}" --loop --disable-keyboard-controls || true

    trap - INT TERM EXIT
    cleanup
fi
