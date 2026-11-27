#!/usr/bin/env bash
source /opt/ros/jazzy/setup.bash

set -euo pipefail
cd "$HOME/echo"

TRIAL="$HOME/echo/tools/echo_s02_multiview_trial_heading_v1.py"

if [ ! -f "$TRIAL" ]; then
    echo "[ERROR] Heading-controlled trial script missing"
    exit 1
fi

if ! ros2 bag record --help >/dev/null 2>&1; then
    echo "[ERROR] ROS 2 bag recorder unavailable"
    exit 1
fi

OUT=$(mktemp -d "$HOME/echo/diagnostics/s02/dual_XXXXXXXX")
BAG="$OUT/lidar_bag"

echo "=== ECHO S02 DUAL-SENSOR EXPERIMENT ==="
echo "Output: $OUT"
echo "D455: RTAB-Map RGB-D database"
echo "LiDAR: ROS 2 bag with synchronized odometry and TF"
echo "WARNING: Route has no collision avoidance"
echo

BAG_PID=""

cleanup() {
    trap - EXIT INT TERM
    if [ -n "$BAG_PID" ] && kill -0 "$BAG_PID" 2>/dev/null; then
        echo "[ECHO] Stopping LiDAR recorder..."
        kill -TERM "$BAG_PID" 2>/dev/null || true
        wait "$BAG_PID" 2>/dev/null || true
    fi
}

trap cleanup EXIT

ros2 bag record \
    --use-sim-time \
    --storage sqlite3 \
    --output "$BAG" \
    --topics \
    /clock \
    /tf \
    /tf_static \
    /chassis/odom \
    /cmd_vel \
    /front_3d_lidar/lidar_points \
    /echo/cameras/front/color/camera_info \
    /echo/cameras/front/depth/camera_info \
    > "$OUT/rosbag.log" 2>&1 &

BAG_PID=$!

sleep 4

if ! kill -0 "$BAG_PID" 2>/dev/null; then
    echo "[ERROR] LiDAR recorder exited during startup"
    tail -n 30 "$OUT/rosbag.log"
    exit 1
fi

echo "[ECHO] LiDAR recorder started: PID $BAG_PID"
echo "[ECHO] Starting heading-controlled D455 mapping trial..."
echo

python3 "$TRIAL" --execute 2>&1 | tee "$OUT/trial_stdout.log"

echo
echo "=== DUAL-SENSOR RECORDING COMPLETE ==="
echo "Output directory: $OUT"

cleanup

if [ -f "$BAG/metadata.yaml" ]; then
    ros2 bag info "$BAG"
else
    echo "[WARNING] Bag metadata not available"
    tail -n 30 "$OUT/rosbag.log"
fi
