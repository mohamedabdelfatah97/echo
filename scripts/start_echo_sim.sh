#!/usr/bin/env bash
set -e

# Leave ECHO's Python virtual environment if active
if [ -n "${VIRTUAL_ENV:-}" ]; then
    deactivate 2>/dev/null || true
fi

# ROS 2 Jazzy environment
source /opt/ros/jazzy/setup.bash
source "$HOME/IsaacSim-ros_workspaces/jazzy_ws/install/local_setup.bash"

export ROS_DOMAIN_ID=0
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp

SCENE="$HOME/echo/scenes/echo_warehouse_nav_meters.usda"

if [ ! -f "$SCENE" ]; then
    echo "ERROR: ECHO warehouse scene not found: $SCENE"
    exit 1
fi

echo "======================================"
echo " ECHO AUTONOMOUS ROBOT SIMULATION"
echo "======================================"
echo "Isaac Sim: 6.2"
echo "Robot: Nova Carter"
echo "ROS: Jazzy"
echo "Scene: $SCENE"
echo "Auto-play: ENABLED"
echo "======================================"

exec ros2 launch isaacsim_bringup run_isaacsim.launch.xml \
    install_path:="$HOME/isaacsim" \
    ros_distro:=jazzy \
    dds_type:=fastdds \
    gui:="$SCENE" \
    play_sim_on_start:=true
