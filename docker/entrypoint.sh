#!/bin/bash
# FIX: export TURTLEBOT3_MODEL so all child processes (ros2 launch, etc.) see it.
# FIX: set RMW_IMPLEMENTATION explicitly — ensures CycloneDDS is always used.
# FIX: set GZ_SIM_RESOURCE_PATH if not already injected via -e.
set -e

source /opt/ros/${ROS_DISTRO}/setup.bash
source /opt/ros2_ws/install/setup.bash

export TURTLEBOT3_MODEL=${TURTLEBOT3_MODEL:-burger}
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=${CYCLONEDDS_URI:-file:///etc/cyclonedds.xml}
export GZ_SIM_RESOURCE_PATH=${GZ_SIM_RESOURCE_PATH:-/opt/ros2_ws/install/tb3_multi_robot/share/tb3_multi_robot/models}

exec "$@"
