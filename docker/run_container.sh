#!/bin/bash
# Zenith Drone Development Container Runner

CONTAINER_NAME="zenith_dev"
IMAGE_NAME="zenith_humble:base"

# Check if container is already running
if [ "$(docker ps -q -f name=^/${CONTAINER_NAME}$)" ]; then
    echo "[INFO] Container '${CONTAINER_NAME}' is already running. Entering container..."
    docker exec -it ${CONTAINER_NAME} /bin/bash
    exit 0
fi

# Check if container exists but is stopped
if [ "$(docker ps -aq -f status=exited -f name=^/${CONTAINER_NAME}$)" ]; then
    echo "[INFO] Starting existing stopped container '${CONTAINER_NAME}'..."
    docker start -ai ${CONTAINER_NAME}
    exit 0
fi

echo "[INFO] Creating and running a new container '${CONTAINER_NAME}'..."

docker run -it \
    --name ${CONTAINER_NAME} \
    --privileged \
    --net=host \
    --ipc=host \
    -e DISPLAY=${DISPLAY:-:0} \
    -e LD_LIBRARY_PATH=/opt/MVS/lib/aarch64 \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    -v /dev:/dev \
    -v /opt/MVS:/opt/MVS \
    -v /home/radxa/zenith_ws:/root/zenith_ws \
    ${IMAGE_NAME} \
    /bin/bash
