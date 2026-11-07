#!/usr/bin/env bash
set -eo pipefail
# Experimental stage only; intentionally never edits the baseline Nav2 setup.
if [ -n "${VIRTUAL_ENV:-}" ]; then
  deactivate 2>/dev/null || true
fi
source /opt/ros/jazzy/setup.bash
source "$HOME/IsaacSim-ros_workspaces/jazzy_ws/install/local_setup.bash"
export ROS_DOMAIN_ID=0
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
SCENE="$HOME/echo/scenes/echo_warehouse_dynamic_red_patrol_v1.usda"
[ -f "$SCENE" ] || { echo "Missing $SCENE" >&2; exit 1; }
echo "Launching ECHO red patrol pilot $SCENE"
exec ros2 launch isaacsim_bringup run_isaacsim.launch.xml \
  install_path:="$HOME/isaacsim" \
  ros_distro:=jazzy \
  dds_type:=fastdds \
  gui:="$SCENE" \
  play_sim_on_start:=true
