#!/bin/bash
set -e

WORKSPACE_DIR="/home/radxa/zenith_ws"
CONTROLLER="${WORKSPACE_DIR}/scripts/remote_scan_controller.py"

echo "=========================================================="
echo "    Zenith Drone: Wireless Remote Scan Launcher"
echo "=========================================================="

python3 "${CONTROLLER}" "$@"
