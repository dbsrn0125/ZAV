#!/bin/bash
set -e

xhost +local:root >/dev/null 2>&1 || true

echo "=========================================================="
echo "    Zenith Drone: Real-Time Visual Extrinsic Tuner        "
echo "=========================================================="
echo "Opening tuner window on your screen..."
echo ""

docker exec -it zenith_dev /root/zenith_ws/scripts/tune_lidar_camera_extrinsic.py "$@"
