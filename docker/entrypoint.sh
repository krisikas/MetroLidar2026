#!/bin/bash
set -e

# Source ROS 2 Humble base system environment
if [ -f "/opt/ros/humble/setup.bash" ]; then
    source "/opt/ros/humble/setup.bash"
fi

# Source MetroLidar2026 workspace overlay environment
if [ -f "/ros2_ws/install/setup.bash" ]; then
    source "/ros2_ws/install/setup.bash"
fi

# Export CycloneDDS / FastDDS config if available
if [ -f "/ros2_ws/cyclonedds.xml" ]; then
    export CYCLONEDDS_URI="file:///ros2_ws/cyclonedds.xml"
fi

exec "$@"
