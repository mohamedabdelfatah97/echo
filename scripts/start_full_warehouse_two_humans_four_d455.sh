#!/usr/bin/env bash
set -e

ISAAC="$HOME/isaacsim"
SCRIPT="$HOME/echo/scripts/start_full_warehouse_two_humans_four_d455.py"

if [ -n "${VIRTUAL_ENV:-}" ]; then
    deactivate 2>/dev/null || true
fi

source /opt/ros/jazzy/setup.bash
source "$HOME/IsaacSim-ros_workspaces/jazzy_ws/install/local_setup.bash"

export ROS_DOMAIN_ID=0
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp

test -f "$ISAAC/python.sh"
test -f "$SCRIPT"

echo "=========================================="
echo " ECHO — FOUR D455 DYNAMIC WAREHOUSE"
echo "=========================================="
echo " Main robot: White ROS Nova Carter"
echo " Agent mode: ${ECHO_S04_AGENTS:-dynamic}"
if [ "${ECHO_S04_AGENTS:-dynamic}" = none ]; then
    echo " IRA actors: not spawned; NavMesh: skipped"
else
    echo " NPC robots: 2 Red IRA Nova Carters"
    echo " Warehouse workers: 2"
    echo " NavMesh: Automatic bake"
fi
echo " ROS 2: Jazzy / Fast DDS"
echo "=========================================="

cd "$ISAAC"
exec ./python.sh "$SCRIPT"
