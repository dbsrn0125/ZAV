#!/bin/bash
set -e

# Source ROS 2 Humble setup
source /opt/ros/humble/setup.bash

# Source workspace install if available
if [ -f "/root/zenith_ws/install/setup.bash" ]; then
    source "/root/zenith_ws/install/setup.bash"
fi

exec "$@"
