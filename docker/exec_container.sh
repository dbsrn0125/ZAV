#!/bin/bash
# Open an additional bash terminal inside the running container

CONTAINER_NAME="zenith_dev"

if [ ! "$(docker ps -q -f name=^/${CONTAINER_NAME}$)" ]; then
    echo "[ERROR] Container '${CONTAINER_NAME}' is not currently running."
    echo "[INFO] Please run './run_container.sh' first."
    exit 1
fi

docker exec -it ${CONTAINER_NAME} /bin/bash
