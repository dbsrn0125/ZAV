#!/bin/bash
set -e

DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )"
IMAGE_NAME="zenith_humble:base"

echo "[INFO] Building Docker image '${IMAGE_NAME}' from ${DIR}/Dockerfile..."
docker build -t ${IMAGE_NAME} -f ${DIR}/Dockerfile ${DIR}

echo "[SUCCESS] Docker image '${IMAGE_NAME}' built successfully!"
