#!/usr/bin/env bash
# ECHO S01-C: four D455-style cameras; interactive Isaac console.
source /opt/ros/jazzy/setup.bash
set -euo pipefail

ROOT="$HOME/echo"
ISAAC="$HOME/isaacsim/isaac-sim.sh"
SCENE="$ROOT/scenes/s01_d455/s01_d455_four_rgbd_raw_trial.usda"
RVIZ_CONF="$ROOT/config/rviz/echo_d455_four_light.rviz"

required=(
  "$ISAAC" "$SCENE"
  "$ROOT/scripts/start_echo_d455_front_helpers.sh"
  "$ROOT/scripts/start_echo_d455_rear_helpers.sh"
  "$ROOT/scripts/start_echo_d455_side_helpers.sh"
)
for f in "${required[@]}"; do
  [[ -f "$f" ]] || { echo "[ERROR] Missing $f"; exit 1; }
done
command -v setsid >/dev/null
ros2 pkg prefix depth_image_proc >/dev/null

if pgrep -u "$(id -u)" -f 'isaacsim\.exp\.full\.kit' >/dev/null || \
   pgrep -u "$(id -u)" -x rviz2 >/dev/null; then
  echo '[ERROR] Isaac Sim or RViz is still running. Close the old session first.'
  exit 1
fi

if [[ ! -e "$RVIZ_CONF" ]]; then
  mkdir -p "$(dirname "$RVIZ_CONF")"
  cat > "$RVIZ_CONF" <<'RVIZ'
Panels:
  - Class: rviz_common/Displays
    Name: Displays
Visualization Manager:
  Class: ""
  Displays:
    - Class: rviz_default_plugins/Grid
      Enabled: true
      Name: Grid
      Plane: XY
  Global Options:
    Background Color: 48; 48; 48
    Fixed Frame: base_link
    Frame Rate: 15
  Name: root
  Tools:
    - Class: rviz_default_plugins/Interact
  Views:
    Current:
      Class: rviz_default_plugins/Orbit
      Distance: 5
RVIZ
  echo "[ECHO] Created lightweight RViz config: $RVIZ_CONF"
fi

LOG_DIR="$(mktemp -d /tmp/echo_four_d455.XXXXXX)"
PIDS=()
start_bg() {
  local name="$1"
  shift
  setsid "$@" >"$LOG_DIR/$name.log" 2>&1 &
  local pid=$!
  PIDS+=("$pid")
  echo "[ECHO] Started $name (PID $pid), log: $LOG_DIR/$name.log"
}
cleanup() {
  trap - EXIT INT TERM
  echo '[ECHO] Stopping this launcher’s helper processes and RViz...'
  for pid in "${PIDS[@]}"; do
    kill -TERM -- "-$pid" 2>/dev/null || true
  done
  for pid in "${PIDS[@]}"; do
    wait "$pid" 2>/dev/null || true
  done
  echo '[ECHO] All managed helpers stopped.'
}
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM

start_bg front_helpers bash "$ROOT/scripts/start_echo_d455_front_helpers.sh"
start_bg rear_helpers bash "$ROOT/scripts/start_echo_d455_rear_helpers.sh"
start_bg side_helpers bash "$ROOT/scripts/start_echo_d455_side_helpers.sh"
start_bg front_cloud ros2 run depth_image_proc point_cloud_xyz_node \
  --ros-args -r __node:=echo_d455_front_depth_to_cloud \
  -r image_rect:=/echo/cameras/front/depth/image_raw \
  -r camera_info:=/echo/cameras/front/depth/camera_info \
  -r points:=/echo/cameras/front/depth/points
start_bg rviz rviz2 -d "$RVIZ_CONF" --ros-args -p use_sim_time:=true

echo
printf '[ECHO] After Isaac Sim opens, use File > Open to load:\n  %s\n' "$SCENE"
echo '[ECHO] Press Play only after the warehouse loads.'
echo '[ECHO] Isaac Sim console output appears BELOW in this terminal.'
echo '[ECHO] Ctrl+C or closing Isaac Sim ends this session and stops helpers.'
echo

# Foreground; deliberately NO stdout/stderr redirection.
"$ISAAC" --info
