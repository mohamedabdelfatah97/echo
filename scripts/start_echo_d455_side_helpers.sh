#!/usr/bin/env bash
source /opt/ros/jazzy/setup.bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
PIDS=()

cleanup() {
    trap - EXIT INT TERM
    echo "[ECHO] Stopping side camera helpers"
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

for side in left right; do
    echo "[ECHO] Starting $side D455 helpers"

    python3 "$ROOT/scripts/echo_d455_${side}_static_tf.py" &
    PIDS+=("$!")

    python3 "$ROOT/scripts/echo_d455_${side}_depth_camera_info.py" &
    PIDS+=("$!")

    ros2 run depth_image_proc point_cloud_xyz_node \
        --ros-args \
        -r __node:=echo_d455_${side}_depth_to_cloud \
        -r image_rect:=/echo/cameras/${side}/depth/image_raw \
        -r camera_info:=/echo/cameras/${side}/depth/camera_info \
        -r points:=/echo/cameras/${side}/depth/points &
    PIDS+=("$!")
done

echo "[ECHO] Six side helpers started. Ctrl+C stops them."

wait -n "${PIDS[@]}"
