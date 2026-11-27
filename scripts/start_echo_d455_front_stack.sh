#!/usr/bin/env bash
source /opt/ros/jazzy/setup.bash
set -euo pipefail

ROOT="$HOME/echo"
ISAAC="$HOME/isaacsim/isaac-sim.sh"

SCENE="$ROOT/scenes/s01_d455/s01_d455_front_ros_depth_raw_verified.usda"
RVIZ="$ROOT/config/rviz/echo_d455_front.rviz"
HELPERS="$ROOT/scripts/start_echo_d455_front_helpers.sh"

for file in "$ISAAC" "$SCENE" "$RVIZ" "$HELPERS"; do
    if [ ! -f "$file" ]; then
        echo "[ERROR] Missing: $file"
        exit 1
    fi
done

command -v setsid >/dev/null
ros2 pkg prefix depth_image_proc >/dev/null

LOG_DIR="$(mktemp -d /tmp/echo_d455_stack.XXXXXX)"
PIDS=()
MONITORED_PIDS=()

start_process() {
    local name="$1"
    shift

    setsid "$@" >"$LOG_DIR/$name.log" 2>&1 &
    local pid=$!

    PIDS+=("$pid")
    if [[ "$name" != "isaac_sim" ]]; then
        MONITORED_PIDS+=("$pid")
    fi
    echo "[STARTED] $name (PID $pid)"
}

cleanup() {
    trap - EXIT INT TERM
    echo
    echo "[ECHO] Stopping managed processes..."

    for pid in "${PIDS[@]}"; do
        # Signal only process groups started by this launcher.
        kill -TERM -- "-$pid" 2>/dev/null || true
    done

    for pid in "${PIDS[@]}"; do
        wait "$pid" 2>/dev/null || true
    done

    echo "[ECHO] Shutdown complete"
    echo "[ECHO] Logs: $LOG_DIR"
}

trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM

echo "========== ECHO FRONT D455 STACK =========="
echo "Logs: $LOG_DIR"
echo

# Reuse the existing Isaac Sim 6.2 GUI if running.
if pgrep -f "^${HOME}/isaacsim/kit/kit[[:space:]]+${HOME}/isaacsim/apps/isaacsim[.]exp[.]full[.]kit([[:space:]]|$)" >/dev/null; then
    echo "[REUSING] Isaac Sim already running (not managed)"
else
    start_process isaac_sim "$ISAAC"
fi

start_process d455_helpers "$HELPERS"

start_process depth_to_cloud \
    ros2 run depth_image_proc point_cloud_xyz_node \
    --ros-args \
    -r __node:=echo_d455_front_depth_to_cloud \
    -r image_rect:=/echo/cameras/front/depth/image_raw \
    -r camera_info:=/echo/cameras/front/depth/camera_info \
    -r points:=/echo/cameras/front/depth/points

start_process rviz \
    rviz2 -d "$RVIZ"

echo
echo "[ECHO] All processes launched"
echo "[ECHO] Open the D455 scene in Isaac Sim:"
echo "$SCENE"
echo "[ECHO] Press Play after the scene loads"
echo "[ECHO] Ctrl+C shuts down the managed stack"
echo

set +e
wait -n "${MONITORED_PIDS[@]}"
RESULT=$?
set -e

echo "[ECHO] A managed process exited (status $RESULT)"
echo "[ECHO] Check logs if it exited unexpectedly"
exit "$RESULT"
