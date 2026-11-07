#!/usr/bin/env bash
set -e

source /opt/ros/jazzy/setup.bash
source "$HOME/IsaacSim-ros_workspaces/jazzy_ws/install/local_setup.bash"

export ROS_DOMAIN_ID=0
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp

exec ros2 launch "$HOME/echo/launch/echo_carter_navigation.launch.xml"
