#!/usr/bin/env bash
set -eo pipefail
# Separate experimental scene. Never launches or overwrites ECHO navigation baseline.
if [ -n "${VIRTUAL_ENV:-}" ]; then
    deactivate 2>/dev/null || true
fi
source /opt/ros/jazzy/setup.bash
source "$HOME/IsaacSim-ros_workspaces/jazzy_ws/install/local_setup.bash"
export ROS_DOMAIN_ID=0
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
SCENE="$HOME/echo/scenes/echo_warehouse_dynamic_red_body_v2.usda"
if [ ! -f "$SCENE" ]; then
    echo "Missing scene: $SCENE" >&2
    echo "Run: python3 ~/echo/tools/build_echo_red_body_v2.py" >&2
    exit 1
fi
echo "ECHO — red Carter body-only V2: $SCENE"
exec ros2 launch isaacsim_bringup run_isaacsim.launch.xml \
    install_path:="$HOME/isaacsim" \
    ros_distro:=jazzy \
    dds_type:=fastdds \
    gui:="$SCENE" \
    play_sim_on_start:=true
