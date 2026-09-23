#!/bin/bash
# Zenith WS: Micro-XRCE-DDS Agent Runner
DEV="${1:-/dev/ttyMSM0}"
BAUD="${2:-921600}"

# Ensure docker container is running
if ! docker ps --format '{{.Names}}' | grep -q "^zenith_dev$"; then
    echo "[INFO] Starting zenith_dev docker container..."
    docker start zenith_dev >/dev/null 2>&1 || true
    sleep 1
fi

echo "=========================================================="
echo "    Starting Micro-XRCE-DDS Agent for PX4 / ROS 2"
echo "    Device:   ${DEV}"
echo "    Baudrate: ${BAUD}"
echo "=========================================================="
echo "Waiting for FC connection (Session created)..."
echo "Press Ctrl+C to stop."
echo "----------------------------------------------------------"

DOCKER_FLAGS="-i"
if [ -t 0 ] && [ -t 1 ]; then
    DOCKER_FLAGS="-it"
fi

docker exec ${DOCKER_FLAGS} zenith_dev MicroXRCEAgent serial -D "${DEV}" -b "${BAUD}" -v 4
