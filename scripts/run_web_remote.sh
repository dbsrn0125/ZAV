#!/bin/bash
set -e

WORKSPACE_DIR="/home/radxa/zenith_ws"
CONTROLLER="${WORKSPACE_DIR}/scripts/web_scan_controller.py"

python3 "${CONTROLLER}" "$@"
