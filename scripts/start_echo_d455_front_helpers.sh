#!/usr/bin/env bash

source /opt/ros/jazzy/setup.bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

TF="$SCRIPT_DIR/echo_d455_front_static_tf.py"
INFO="$SCRIPT_DIR/echo_d455_front_depth_camera_info.py"

for script in "$TF" "$INFO"; do
    if [ ! -f "$script" ]; then
        echo "[ERROR] Missing: $script"
        exit 1
    fi
done

PIDS=()

cleanup() {
    trap - EXIT INT TERM
    echo "[ECHO] Stopping D455 helpers"
    if [ "${#PIDS[@]}" -gt 0 ]; then
        kill "${PIDS[@]}" 2>/dev/null || true
        wait "${PIDS[@]}" 2>/dev/null || true
    fi
}

trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM

echo "[ECHO] Starting front D455 ROS helpers"

python3 "$TF" &
PIDS+=("$!")

python3 "$INFO" &
PIDS+=("$!")

echo "[ECHO] Both helper processes launched"
echo "[ECHO] Ctrl+C stops both"

wait -n "${PIDS[@]}"
