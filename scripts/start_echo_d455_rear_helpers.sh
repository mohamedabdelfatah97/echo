#!/usr/bin/env bash
source /opt/ros/jazzy/setup.bash
set -euo pipefail

ROOT="$HOME/echo"
PIDS=()

cleanup() {
    trap - EXIT INT TERM
    echo "[ECHO] Stopping rear camera helpers..."

    for pid in "${PIDS[@]}"; do
        kill "$pid" 2>/dev/null || true
    done

    for pid in "${PIDS[@]}"; do
        wait "$pid" 2>/dev/null || true
    done
}

trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM

echo "=== ECHO REAR D455 HELPERS ==="

python3 "$ROOT/scripts/echo_d455_rear_static_tf.py" &
PIDS+=("$!")

python3 "$ROOT/scripts/echo_d455_rear_depth_camera_info.py" &
PIDS+=("$!")

ros2 run depth_image_proc point_cloud_xyz_node \
    --ros-args \
    -r __node:=echo_d455_rear_depth_to_cloud \
    -r image_rect:=/echo/cameras/rear/depth/image_raw \
    -r camera_info:=/echo/cameras/rear/depth/camera_info \
    -r points:=/echo/cameras/rear/depth/points &
PIDS+=("$!")

echo "[ECHO] All three rear helpers started."
echo "[ECHO] Ctrl+C stops all three."
echo

set +e
wait -n "${PIDS[@]}"
RESULT=$?
set -e

echo "[ECHO] A rear helper exited (status $RESULT)"
exit "$RESULT"
