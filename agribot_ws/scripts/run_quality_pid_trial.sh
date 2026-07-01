#!/usr/bin/env bash
set -euo pipefail

WORKSPACE="${WORKSPACE:-/home/xkai/agribot/agribot_ws}"
WORLD="${WORLD:-zhenshi4hang16m.world}"
OUT_ROOT="${OUT_ROOT:-$HOME/agribot_test_logs/zhenshi4hang16m_quality_pid}"
TRIAL_NAME="${1:-trial_$(date +%Y%m%d_%H%M%S)}"
TRIAL_DIR="$OUT_ROOT/$TRIAL_NAME"
BAG_DIR="$TRIAL_DIR/bag"

GAZEBO_WAIT_SEC="${GAZEBO_WAIT_SEC:-12}"
DETECTOR_WAIT_SEC="${DETECTOR_WAIT_SEC:-4}"
BAG_WAIT_SEC="${BAG_WAIT_SEC:-2}"
RUN_DURATION_SEC="${RUN_DURATION_SEC:-200}"
FORCE_CLEANUP_BEFORE_START="${FORCE_CLEANUP_BEFORE_START:-true}"
FORCE_CLEANUP_AFTER_EXIT="${FORCE_CLEANUP_AFTER_EXIT:-true}"
PID_EXTRA_ARGS="${PID_EXTRA_ARGS:-}"
OVERWRITE_TRIAL="${OVERWRITE_TRIAL:-false}"

TOPICS=(
  /clock
  /tf
  /tf_static
  /odom
  /cmd_vel
  /target_point
  /mid360_PointCloud2
  /corn_row_center_line
  /corn_row_center_line_viz
  /under_canopy_left_boundary
  /under_canopy_right_boundary
  /corridor_width
  /corridor_safety_margin
  /corridor_confidence
  /centerline_detection_diagnostics
  /point_cloud_projected
  /left_row_points
  /right_row_points
)

if [[ -e "$BAG_DIR" && "$OVERWRITE_TRIAL" != "true" ]]; then
  echo "[ERROR] Bag directory already exists: $BAG_DIR"
  echo "[ERROR] Use a new trial name/OUT_ROOT, or set OVERWRITE_TRIAL=true to remove the old trial directory."
  exit 2
fi

if [[ -e "$TRIAL_DIR" && "$OVERWRITE_TRIAL" == "true" ]]; then
  echo "[WARN] OVERWRITE_TRIAL=true, removing existing trial directory: $TRIAL_DIR"
  rm -rf "$TRIAL_DIR"
fi

mkdir -p "$TRIAL_DIR"

LOG_GAZEBO="$TRIAL_DIR/gazebo.log"
LOG_DETECTOR="$TRIAL_DIR/centerline_detector.log"
LOG_NAV="$TRIAL_DIR/navigation_pid.log"
LOG_BAG="$TRIAL_DIR/rosbag.log"
SUMMARY="$TRIAL_DIR/trial_summary.md"

PIDS=()
CLEANED_UP=0

force_cleanup_stale_processes() {
  echo "[INFO] Cleaning stale simulation processes..."

  local patterns=(
    "ros2 launch diff_drive_robot robot.launch.py"
    "gzserver .*agribot_simulation/world"
    "gzclient"
    "gazebo"
    "spawn_entity.py.*diff_drive_robot"
    "robot_state_publisher.*robot_state_publisher"
    "corn_row_detector_projection"
    "cornfield_navigation_node"
    "ros2 bag record -o .*agribot_test_logs"
  )

  for pattern in "${patterns[@]}"; do
    pkill -INT -f "$pattern" 2>/dev/null || true
  done
  sleep 2

  for pattern in "${patterns[@]}"; do
    pkill -TERM -f "$pattern" 2>/dev/null || true
  done
  sleep 1

  for pattern in "${patterns[@]}"; do
    pkill -KILL -f "$pattern" 2>/dev/null || true
  done
}

start_managed_process() {
  local log_file="$1"
  shift

  setsid stdbuf -oL -eL "$@" > "$log_file" 2>&1 &
  PIDS+=("$!")
}

cleanup() {
  set +e
  if [[ "$CLEANED_UP" -eq 1 ]]; then
    return
  fi
  CLEANED_UP=1

  echo
  echo "[INFO] Stopping trial processes..."
  for pid in "${PIDS[@]:-}"; do
    if kill -0 "$pid" 2>/dev/null; then
      kill -INT -- "-$pid" 2>/dev/null || kill -INT "$pid" 2>/dev/null
    fi
  done
  sleep 3
  for pid in "${PIDS[@]:-}"; do
    if kill -0 "$pid" 2>/dev/null; then
      kill -TERM -- "-$pid" 2>/dev/null || kill -TERM "$pid" 2>/dev/null
    fi
  done
  sleep 2
  for pid in "${PIDS[@]:-}"; do
    if kill -0 "$pid" 2>/dev/null; then
      kill -KILL -- "-$pid" 2>/dev/null || kill -KILL "$pid" 2>/dev/null
    fi
  done
  wait "${PIDS[@]:-}" 2>/dev/null

  if [[ "$FORCE_CLEANUP_AFTER_EXIT" == "true" ]]; then
    force_cleanup_stale_processes
  fi
}
trap cleanup EXIT
trap 'cleanup; exit 130' INT
trap 'cleanup; exit 143' TERM

echo "[INFO] Workspace: $WORKSPACE"
echo "[INFO] World: $WORLD"
echo "[INFO] Trial: $TRIAL_NAME"
echo "[INFO] Output: $TRIAL_DIR"

if [[ "$FORCE_CLEANUP_BEFORE_START" == "true" ]]; then
  force_cleanup_stale_processes
fi

cd "$WORKSPACE"
set +u
source install/setup.bash
set -u

cat > "$SUMMARY" <<EOF
# Trial Summary

- Trial name: $TRIAL_NAME
- Date: $(date --iso-8601=seconds)
- Workspace: $WORKSPACE
- World: $WORLD
- Run duration: ${RUN_DURATION_SEC}s
- Gazebo wait: ${GAZEBO_WAIT_SEC}s
- Detector wait: ${DETECTOR_WAIT_SEC}s
- Bag wait: ${BAG_WAIT_SEC}s
- Overwrite trial: ${OVERWRITE_TRIAL}

## Commands

\`\`\`bash
ros2 launch diff_drive_robot robot.launch.py world:=$WORLD
ros2 run centerline_extraction corn_row_detector_projection --ros-args -p use_sim_time:=true
ros2 run centerline_extraction cornfield_navigation_node --ros-args -p use_sim_time:=true $PID_EXTRA_ARGS
ros2 bag record -o "$BAG_DIR" ${TOPICS[*]}
\`\`\`

## Manual Notes

- Success:
- Stop happened:
- Obvious deviation:
- Collision:
- Center line disappeared:
- Confidence response observed:
- Safety margin response observed:
- Notes:

EOF

echo "[INFO] Saving current git diff..."
git diff -- src/centerline_extraction/src/pid_controller.cpp \
  src/centerline_extraction/include/centerline_extraction/pid_controller.hpp \
  src/centerline_extraction/src/corn_row_detector_projection.cpp \
  src/centerline_extraction/include/centerline_extraction/corn_row_detector_projection.hpp \
  > "$TRIAL_DIR/git_diff_relevant.patch" || true

echo "[INFO] Starting Gazebo..."
start_managed_process "$LOG_GAZEBO" \
  ros2 launch diff_drive_robot robot.launch.py \
  world:="$WORLD"
sleep "$GAZEBO_WAIT_SEC"

echo "[INFO] Starting centerline detector..."
start_managed_process "$LOG_DETECTOR" \
  ros2 run centerline_extraction corn_row_detector_projection \
  --ros-args -p use_sim_time:=true
sleep "$DETECTOR_WAIT_SEC"

echo "[INFO] Capturing topic and parameter snapshots..."
ros2 topic list > "$TRIAL_DIR/topic_list.txt" || true
ros2 node list > "$TRIAL_DIR/node_list.txt" || true
ros2 param list > "$TRIAL_DIR/param_list.txt" || true
ros2 param dump /pid_controller > "$TRIAL_DIR/pid_controller_params.yaml" 2>/dev/null || true
ros2 param dump /corn_row_detector_projection > "$TRIAL_DIR/corn_row_detector_projection_params.yaml" 2>/dev/null || true

echo "[INFO] Starting rosbag record..."
start_managed_process "$LOG_BAG" \
  ros2 bag record -o "$BAG_DIR" "${TOPICS[@]}"
sleep "$BAG_WAIT_SEC"

echo "[INFO] Starting navigation PID..."
start_managed_process "$LOG_NAV" \
  ros2 run centerline_extraction cornfield_navigation_node \
  --ros-args -p use_sim_time:=true $PID_EXTRA_ARGS

echo "[INFO] Running trial for ${RUN_DURATION_SEC}s..."
sleep "$RUN_DURATION_SEC"

echo "[INFO] Trial duration reached."

echo "[INFO] Saving final snapshots..."
ros2 topic echo /corridor_confidence --once > "$TRIAL_DIR/final_corridor_confidence.txt" 2>/dev/null || true
ros2 topic echo /corridor_safety_margin --once > "$TRIAL_DIR/final_corridor_safety_margin.txt" 2>/dev/null || true
ros2 topic echo /centerline_detection_diagnostics --once > "$TRIAL_DIR/final_centerline_detection_diagnostics.txt" 2>/dev/null || true
ros2 topic echo /cmd_vel --once > "$TRIAL_DIR/final_cmd_vel.txt" 2>/dev/null || true
ros2 topic echo /odom --once > "$TRIAL_DIR/final_odom.txt" 2>/dev/null || true

cat >> "$SUMMARY" <<EOF
## Finished

- Finished at: $(date --iso-8601=seconds)
- Bag directory: $BAG_DIR
- Logs:
  - $LOG_GAZEBO
  - $LOG_DETECTOR
  - $LOG_NAV
  - $LOG_BAG

## Bag Info Command

\`\`\`bash
ros2 bag info "$BAG_DIR"
\`\`\`

EOF

echo "[INFO] Done. Trial data saved to: $TRIAL_DIR"
