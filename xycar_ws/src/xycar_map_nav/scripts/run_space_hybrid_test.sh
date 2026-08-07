#!/usr/bin/env bash

set -eo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "$(readlink -f "${BASH_SOURCE[0]}")")" && pwd)"
SOURCE_WORKSPACE="$(cd -- "$SCRIPT_DIR/../../.." && pwd)"
WORKSPACE="${XYCAR_WS:-$SOURCE_WORKSPACE}"
if [[ ! -f "$WORKSPACE/install/setup.bash" ]] || \
  [[ ! -f "$WORKSPACE/src/xycar_map_nav/launch/real_sequential_hybrid_drive.launch.py" ]]; then
  if [[ -n "${XYCAR_WS:-}" ]]; then
    echo "[경고] XYCAR_WS=$XYCAR_WS 는 현재 통합 주행 workspace가 아닙니다."
    echo "[자동 복구] $SOURCE_WORKSPACE 를 사용합니다."
  fi
  WORKSPACE="$SOURCE_WORKSPACE"
fi
export XYCAR_WS="$WORKSPACE"
SPEED_COMMAND="${1:-}"
START_WAYPOINT="${2:-}"
RUN_MODE="${3:-rule}"
MODEL_PROFILE="${4:-speed100}"
LOOKAHEAD_DISTANCE="${5:-}"
STANLEY_PERCENT="${6:-}"
LEFT_OFFSET_CM="${7:-}"
PURE_PURSUIT_CONTROL_X_M="${PURE_PURSUIT_CONTROL_X_M:-}"
STANLEY_CONTROL_X_M="${STANLEY_CONTROL_X_M:-}"
STANLEY_GAIN="${STANLEY_GAIN:-}"
STANLEY_SOFTENING_MPS="${STANLEY_SOFTENING_MPS:-}"
STRAIGHT_STANLEY_PERCENT="${STRAIGHT_STANLEY_PERCENT:-}"
STRAIGHT_STANLEY_GAIN="${STRAIGHT_STANLEY_GAIN:-}"
STRAIGHT_STANLEY_SOFTENING_MPS="${STRAIGHT_STANLEY_SOFTENING_MPS:-}"
OPPOSED_STANLEY_PERCENT="${OPPOSED_STANLEY_PERCENT:-}"
CONTROL_LATENCY_PREVIEW_SEC="${CONTROL_LATENCY_PREVIEW_SEC:-}"
ADAPTIVE_STEERING_SPEED_ENABLED="${ADAPTIVE_STEERING_SPEED_ENABLED:-}"
STEERING_TURN_SPEED_COMMAND="${STEERING_TURN_SPEED_COMMAND:-}"
STEERING_SLOWDOWN_START_ANGLE="${STEERING_SLOWDOWN_START_ANGLE:-}"
STEERING_FULL_SLOWDOWN_ANGLE="${STEERING_FULL_SLOWDOWN_ANGLE:-}"
CONTROL_LOG="/tmp/xycar_hybrid_control_$(date +%Y%m%d_%H%M%S).log"
RUN_CONFIG_FILE="${XYCAR_HYBRID_RUN_CONFIG_FILE:-/tmp/xycar_hybrid_run_config.yaml}"
CONE_SPEED_COMMAND="6.0"
TEST_PROFILE="${XYCAR_TEST_PROFILE:-integrated}"
ENABLE_RVIZ="${XYCAR_ENABLE_RVIZ:-false}"
START_CONE="${XYCAR_START_CONE:-true}"
STEERING_ONLY="${XYCAR_STEERING_ONLY:-false}"
VEHICLE_YOLO_MIN_CONFIDENCE="${VEHICLE_YOLO_MIN_CONFIDENCE:-0.45}"
CONE_AS_VEHICLE_OBSTACLE="${CONE_AS_VEHICLE_OBSTACLE:-false}"
CONE_AS_VEHICLE_MIN_CONFIDENCE="${CONE_AS_VEHICLE_MIN_CONFIDENCE:-0.50}"
VEHICLE_YOLO_REQUIRED_FRAMES="${VEHICLE_YOLO_REQUIRED_FRAMES:-2}"
VEHICLE_YOLO_TIMEOUT_SEC="${VEHICLE_YOLO_TIMEOUT_SEC:-2.50}"
VEHICLE_CAMERA_LIDAR_HFOV_DEG="${VEHICLE_CAMERA_LIDAR_HFOV_DEG:-60.0}"
VEHICLE_CAMERA_LIDAR_PADDING_DEG="${VEHICLE_CAMERA_LIDAR_PADDING_DEG:-3.0}"
VEHICLE_LIDAR_MIN_POINTS="${VEHICLE_LIDAR_MIN_POINTS:-2}"
VEHICLE_LIDAR_SECTOR_MEMORY_SEC="${VEHICLE_LIDAR_SECTOR_MEMORY_SEC:-0.50}"
VEHICLE_LIDAR_ASSOCIATION_ANGLE_MARGIN_DEG="${VEHICLE_LIDAR_ASSOCIATION_ANGLE_MARGIN_DEG:-2.0}"
VEHICLE_LIDAR_ASSOCIATION_DISTANCE_TOLERANCE_M="${VEHICLE_LIDAR_ASSOCIATION_DISTANCE_TOLERANCE_M:-0.35}"
VEHICLE_AVOIDANCE_IMMEDIATE_ON_YOLO="${VEHICLE_AVOIDANCE_IMMEDIATE_ON_YOLO:-true}"
VEHICLE_AVOIDANCE_ENTRY_DISTANCE_M="${VEHICLE_AVOIDANCE_ENTRY_DISTANCE_M:-1.20}"
VEHICLE_MINIMUM_SIDE_CLEARANCE_M="${VEHICLE_MINIMUM_SIDE_CLEARANCE_M:-0.70}"
VEHICLE_LEFT_OFFSET_M="${VEHICLE_LEFT_OFFSET_M:-0.33}"
VEHICLE_RIGHT_OFFSET_M="${VEHICLE_RIGHT_OFFSET_M:-0.33}"
VEHICLE_OFFSET_RATE_MPS="${VEHICLE_OFFSET_RATE_MPS:-0.35}"
VEHICLE_AVOIDANCE_SPEED_LIMIT_COMMAND="${VEHICLE_AVOIDANCE_SPEED_LIMIT_COMMAND:-8.0}"
VEHICLE_MINIMUM_AVOID_SEC="${VEHICLE_MINIMUM_AVOID_SEC:-0.80}"
VEHICLE_CLEAR_HOLD_SEC="${VEHICLE_CLEAR_HOLD_SEC:-1.0}"
VEHICLE_RETURN_HOLD_SEC="${VEHICLE_RETURN_HOLD_SEC:-0.30}"
VEHICLE_RETURN_DEADBAND_M="${VEHICLE_RETURN_DEADBAND_M:-0.02}"
VEHICLE_BODY_LENGTH_M="${VEHICLE_BODY_LENGTH_M:-0.55}"
VEHICLE_BODY_WIDTH_M="${VEHICLE_BODY_WIDTH_M:-0.28}"
LIDAR_OBSTACLE_DETECT_DISTANCE_M="${LIDAR_OBSTACLE_DETECT_DISTANCE_M:-1.50}"
LIDAR_OBSTACLE_MINIMUM_DISTANCE_M="${LIDAR_OBSTACLE_MINIMUM_DISTANCE_M:-0.18}"
LIDAR_OBSTACLE_PATH_HALF_WIDTH_M="${LIDAR_OBSTACLE_PATH_HALF_WIDTH_M:-0.18}"
LIDAR_OBSTACLE_MINIMUM_CLUSTER_POINTS="${LIDAR_OBSTACLE_MINIMUM_CLUSTER_POINTS:-3}"
LIDAR_OBSTACLE_MAXIMUM_SCAN_INDEX_GAP="${LIDAR_OBSTACLE_MAXIMUM_SCAN_INDEX_GAP:-2}"
LIDAR_OBSTACLE_MAXIMUM_CLUSTER_GAP_M="${LIDAR_OBSTACLE_MAXIMUM_CLUSTER_GAP_M:-0.16}"
LIDAR_OBSTACLE_MINIMUM_CLUSTER_WIDTH_M="${LIDAR_OBSTACLE_MINIMUM_CLUSTER_WIDTH_M:-0.09}"
LIDAR_OBSTACLE_MAXIMUM_CLUSTER_WIDTH_M="${LIDAR_OBSTACLE_MAXIMUM_CLUSTER_WIDTH_M:-0.70}"
LIDAR_OBSTACLE_SIDE_PROBE_INNER_M="${LIDAR_OBSTACLE_SIDE_PROBE_INNER_M:-0.18}"
LIDAR_OBSTACLE_SIDE_PROBE_OUTER_M="${LIDAR_OBSTACLE_SIDE_PROBE_OUTER_M:-0.55}"
LIDAR_OBSTACLE_LIDAR_X_M="${LIDAR_OBSTACLE_LIDAR_X_M:-0.065}"
LIDAR_OBSTACLE_LIDAR_Y_M="${LIDAR_OBSTACLE_LIDAR_Y_M:-0.0}"
LIDAR_OBSTACLE_LIDAR_YAW_DEG="${LIDAR_OBSTACLE_LIDAR_YAW_DEG:-0.0}"

prompt_float() {
  local variable_name="$1"
  local prompt="$2"
  local default_value="$3"
  local minimum="$4"
  local maximum="$5"
  local value="${!variable_name:-}"
  if [[ -z "$value" ]]; then
    if [[ -t 0 ]]; then
      read -r -p "$prompt [default $default_value]: " value
    fi
    value="${value:-$default_value}"
  fi
  value="${value/,/.}"
  if [[ ! "$value" =~ ^-?[0-9]+([.][0-9]+)?$ ]] || \
    ! awk -v value="$value" -v minimum="$minimum" -v maximum="$maximum" \
      'BEGIN { exit !(value >= minimum && value <= maximum) }'; then
    echo "ERROR: $prompt must be from $minimum to $maximum." >&2
    exit 2
  fi
  printf -v "$variable_name" '%.3f' "$value"
}

prompt_bool() {
  local variable_name="$1"
  local prompt="$2"
  local default_value="$3"
  local value="${!variable_name:-}"
  if [[ -z "$value" ]]; then
    if [[ -t 0 ]]; then
      read -r -p "$prompt [Y/n]: " value
    fi
    value="${value:-$default_value}"
  fi
  case "${value,,}" in
    y|yes|true|1) printf -v "$variable_name" '%s' true ;;
    n|no|false|0) printf -v "$variable_name" '%s' false ;;
    *)
      echo "ERROR: $prompt must be y or n." >&2
      exit 2
      ;;
  esac
}

if [[ "$CONE_AS_VEHICLE_OBSTACLE" == "true" ]]; then
  AVOIDANCE_TARGET_LABEL=cone
  AVOIDANCE_DISPLAY_CONFIDENCE="$CONE_AS_VEHICLE_MIN_CONFIDENCE"
else
  AVOIDANCE_TARGET_LABEL=vehicle
  AVOIDANCE_DISPLAY_CONFIDENCE="$VEHICLE_YOLO_MIN_CONFIDENCE"
fi

if [[ -z "$SPEED_COMMAND" ]]; then
  if [[ -t 0 ]]; then
    read -r -p "Driving speed command [3.0-30.0, default 3.0]: " SPEED_COMMAND
  fi
  SPEED_COMMAND="${SPEED_COMMAND:-3.0}"
fi
if [[ "$STEERING_ONLY" == "true" ]]; then
  SPEED_COMMAND=0.0
elif [[ ! "$SPEED_COMMAND" =~ ^[0-9]+([.][0-9]+)?$ ]] || \
  ! awk -v speed="$SPEED_COMMAND" 'BEGIN { exit !(speed >= 3.0 && speed <= 30.0) }'; then
    echo "ERROR: speed command must be a number from 3.0 to 30.0." >&2
    exit 2
fi
SPEED_COMMAND="$(awk -v speed="$SPEED_COMMAND" 'BEGIN { printf "%.3f", speed }')"

prompt_bool ADAPTIVE_STEERING_SPEED_ENABLED \
  "Adaptive steering speed" true
if [[ "$ADAPTIVE_STEERING_SPEED_ENABLED" == "true" ]]; then
  prompt_float STEERING_SLOWDOWN_START_ANGLE \
    "Steering slowdown start angle" 20.0 0.0 42.0
  prompt_float STEERING_FULL_SLOWDOWN_ANGLE \
    "Steering full slowdown angle" 42.0 0.0 42.0
  prompt_float STEERING_TURN_SPEED_COMMAND \
    "Full-steering speed command" 8.0 0.0 30.0
  if ! awk \
    -v start="$STEERING_SLOWDOWN_START_ANGLE" \
    -v full="$STEERING_FULL_SLOWDOWN_ANGLE" \
    'BEGIN { exit !(full >= start) }'; then
    echo "ERROR: full slowdown angle must be >= start angle." >&2
    exit 2
  fi
else
  STEERING_SLOWDOWN_START_ANGLE=20.000
  STEERING_FULL_SLOWDOWN_ANGLE=42.000
  STEERING_TURN_SPEED_COMMAND=8.000
fi

if [[ -z "$START_WAYPOINT" ]]; then
  if [[ -t 0 ]]; then
    read -r -p "Start target waypoint number [1-6, default 1]: " START_WAYPOINT
  fi
  START_WAYPOINT="${START_WAYPOINT:-1}"
fi
if [[ ! "$START_WAYPOINT" =~ ^[1-6]$ ]]; then
  echo "ERROR: start waypoint must be an integer from 1 to 6." >&2
  exit 2
fi
if [[ "$RUN_MODE" != "rule" ]]; then
  echo "ERROR: this SLAM-free stack supports only 'rule' mode." >&2
  exit 2
fi

if [[ -z "$LOOKAHEAD_DISTANCE" ]]; then
  if [[ -t 0 ]]; then
    read -r -p "Curve lookahead distance [m, default 0.3]: " \
      LOOKAHEAD_DISTANCE
  fi
  LOOKAHEAD_DISTANCE="${LOOKAHEAD_DISTANCE:-0.3}"
fi
LOOKAHEAD_DISTANCE="${LOOKAHEAD_DISTANCE/,/.}"
if [[ ! "$LOOKAHEAD_DISTANCE" =~ ^[0-9]+([.][0-9]+)?$ ]] || \
  ! awk -v value="$LOOKAHEAD_DISTANCE" \
    'BEGIN { exit !(value >= 0.1 && value <= 5.0) }'; then
  echo "ERROR: lookahead distance must be from 0.1 to 5.0m." >&2
  exit 2
fi
LOOKAHEAD_DISTANCE="$(awk -v value="$LOOKAHEAD_DISTANCE" \
  'BEGIN { printf "%.3f", value }')"

if [[ -z "$STANLEY_PERCENT" ]]; then
  if [[ -t 0 ]]; then
    read -r -p "Curve Stanley percentage [0-100, default 20]: " \
      STANLEY_PERCENT
  fi
  STANLEY_PERCENT="${STANLEY_PERCENT:-20}"
fi
STANLEY_PERCENT="${STANLEY_PERCENT/,/.}"
if [[ ! "$STANLEY_PERCENT" =~ ^[0-9]+([.][0-9]+)?$ ]] || \
  ! awk -v value="$STANLEY_PERCENT" \
    'BEGIN { exit !(value >= 0.0 && value <= 100.0) }'; then
  echo "ERROR: Stanley percentage must be from 0 to 100." >&2
  exit 2
fi
STANLEY_PERCENT="$(awk -v value="$STANLEY_PERCENT" \
  'BEGIN { printf "%.3f", value }')"
PURE_PURSUIT_WEIGHT="$(awk -v stanley="$STANLEY_PERCENT" \
  'BEGIN { printf "%.6f", 1.0 - (stanley / 100.0) }')"

prompt_float PURE_PURSUIT_CONTROL_X_M \
  "Pure Pursuit control X [m]" -0.08 -1.0 1.0
prompt_float STANLEY_CONTROL_X_M \
  "Stanley control X [m]" 0.16 -1.0 1.0
prompt_float STANLEY_GAIN \
  "Curve Stanley cross-track gain" 1.15 0.0 10.0
prompt_float STANLEY_SOFTENING_MPS \
  "Curve Stanley softening [m/s]" 0.35 0.01 10.0
prompt_float STRAIGHT_STANLEY_PERCENT \
  "Straight Stanley percentage" 90.0 0.0 100.0
prompt_float STRAIGHT_STANLEY_GAIN \
  "Straight Stanley cross-track gain" 0.65 0.0 10.0
prompt_float STRAIGHT_STANLEY_SOFTENING_MPS \
  "Straight Stanley softening [m/s]" 0.65 0.01 10.0
prompt_float OPPOSED_STANLEY_PERCENT \
  "Opposed-term Stanley percentage" 70.0 0.0 100.0
prompt_float CONTROL_LATENCY_PREVIEW_SEC \
  "Control latency preview [s]" 0.30 0.0 2.0

STRAIGHT_PURE_PURSUIT_WEIGHT="$(awk \
  -v stanley="$STRAIGHT_STANLEY_PERCENT" \
  'BEGIN { printf "%.6f", 1.0 - (stanley / 100.0) }')"
OPPOSED_STANLEY_WEIGHT="$(awk -v stanley="$OPPOSED_STANLEY_PERCENT" \
  'BEGIN { printf "%.6f", stanley / 100.0 }')"

if [[ -z "$LEFT_OFFSET_CM" ]]; then
  if [[ -t 0 ]]; then
    read -r -p "Left target correction [cm, default 9]: " LEFT_OFFSET_CM
  fi
  LEFT_OFFSET_CM="${LEFT_OFFSET_CM:-9}"
fi
LEFT_OFFSET_CM="${LEFT_OFFSET_CM/,/.}"
if [[ ! "$LEFT_OFFSET_CM" =~ ^[0-9]+([.][0-9]+)?$ ]] || \
  ! awk -v value="$LEFT_OFFSET_CM" \
    'BEGIN { exit !(value >= 0.0 && value <= 100.0) }'; then
  echo "ERROR: left correction must be from 0 to 100cm." >&2
  exit 2
fi
LEFT_OFFSET_CM="$(awk -v value="$LEFT_OFFSET_CM" \
  'BEGIN { printf "%.1f", value }')"
LEFT_OFFSET_M="$(awk -v value="$LEFT_OFFSET_CM" \
  'BEGIN { printf "%.6f", value / 100.0 }')"

run_config_tmp="${RUN_CONFIG_FILE}.tmp.$$"
cat >"$run_config_tmp" <<EOF
recorded_at: "$(date --iso-8601=seconds)"
run_mode: "$RUN_MODE"
speed_command: $SPEED_COMMAND
start_waypoint_number: $START_WAYPOINT
lookahead_distance_m: $LOOKAHEAD_DISTANCE
stanley_percent: $STANLEY_PERCENT
pure_pursuit_weight: $PURE_PURSUIT_WEIGHT
pure_pursuit_control_x_m: $PURE_PURSUIT_CONTROL_X_M
stanley_control_x_m: $STANLEY_CONTROL_X_M
stanley_gain: $STANLEY_GAIN
stanley_softening_mps: $STANLEY_SOFTENING_MPS
straight_stanley_percent: $STRAIGHT_STANLEY_PERCENT
straight_pure_pursuit_weight: $STRAIGHT_PURE_PURSUIT_WEIGHT
straight_stanley_gain: $STRAIGHT_STANLEY_GAIN
straight_stanley_softening_mps: $STRAIGHT_STANLEY_SOFTENING_MPS
opposed_stanley_percent: $OPPOSED_STANLEY_PERCENT
opposed_stanley_weight: $OPPOSED_STANLEY_WEIGHT
control_latency_preview_sec: $CONTROL_LATENCY_PREVIEW_SEC
target_left_offset_cm: $LEFT_OFFSET_CM
target_left_offset_m: $LEFT_OFFSET_M
cone_speed_command: $CONE_SPEED_COMMAND
test_profile: "$TEST_PROFILE"
enable_rviz: $ENABLE_RVIZ
start_cone: $START_CONE
steering_only: $STEERING_ONLY
adaptive_steering_speed_enabled: $ADAPTIVE_STEERING_SPEED_ENABLED
turn_speed_command: $STEERING_TURN_SPEED_COMMAND
slowdown_start_angle_command: $STEERING_SLOWDOWN_START_ANGLE
full_slowdown_angle_command: $STEERING_FULL_SLOWDOWN_ANGLE
vehicle_yolo_min_confidence: $VEHICLE_YOLO_MIN_CONFIDENCE
cone_as_vehicle_obstacle: $CONE_AS_VEHICLE_OBSTACLE
cone_as_vehicle_min_confidence: $CONE_AS_VEHICLE_MIN_CONFIDENCE
vehicle_yolo_required_frames: $VEHICLE_YOLO_REQUIRED_FRAMES
vehicle_yolo_timeout_sec: $VEHICLE_YOLO_TIMEOUT_SEC
vehicle_camera_lidar_hfov_deg: $VEHICLE_CAMERA_LIDAR_HFOV_DEG
vehicle_camera_lidar_padding_deg: $VEHICLE_CAMERA_LIDAR_PADDING_DEG
vehicle_lidar_min_points: $VEHICLE_LIDAR_MIN_POINTS
vehicle_lidar_sector_memory_sec: $VEHICLE_LIDAR_SECTOR_MEMORY_SEC
vehicle_lidar_association_angle_margin_deg: $VEHICLE_LIDAR_ASSOCIATION_ANGLE_MARGIN_DEG
vehicle_lidar_association_distance_tolerance_m: $VEHICLE_LIDAR_ASSOCIATION_DISTANCE_TOLERANCE_M
vehicle_avoidance_immediate_on_yolo: $VEHICLE_AVOIDANCE_IMMEDIATE_ON_YOLO
vehicle_avoidance_entry_distance_m: $VEHICLE_AVOIDANCE_ENTRY_DISTANCE_M
vehicle_minimum_side_clearance_m: $VEHICLE_MINIMUM_SIDE_CLEARANCE_M
vehicle_left_offset_m: $VEHICLE_LEFT_OFFSET_M
vehicle_right_offset_m: $VEHICLE_RIGHT_OFFSET_M
vehicle_offset_rate_mps: $VEHICLE_OFFSET_RATE_MPS
vehicle_avoidance_speed_limit_command: $VEHICLE_AVOIDANCE_SPEED_LIMIT_COMMAND
vehicle_minimum_avoid_sec: $VEHICLE_MINIMUM_AVOID_SEC
vehicle_clear_hold_sec: $VEHICLE_CLEAR_HOLD_SEC
vehicle_return_hold_sec: $VEHICLE_RETURN_HOLD_SEC
vehicle_return_deadband_m: $VEHICLE_RETURN_DEADBAND_M
vehicle_body_length_m: $VEHICLE_BODY_LENGTH_M
vehicle_body_width_m: $VEHICLE_BODY_WIDTH_M
lidar_obstacle_detect_distance_m: $LIDAR_OBSTACLE_DETECT_DISTANCE_M
lidar_obstacle_minimum_distance_m: $LIDAR_OBSTACLE_MINIMUM_DISTANCE_M
lidar_obstacle_path_half_width_m: $LIDAR_OBSTACLE_PATH_HALF_WIDTH_M
lidar_obstacle_minimum_cluster_points: $LIDAR_OBSTACLE_MINIMUM_CLUSTER_POINTS
lidar_obstacle_maximum_scan_index_gap: $LIDAR_OBSTACLE_MAXIMUM_SCAN_INDEX_GAP
lidar_obstacle_maximum_cluster_gap_m: $LIDAR_OBSTACLE_MAXIMUM_CLUSTER_GAP_M
lidar_obstacle_minimum_cluster_width_m: $LIDAR_OBSTACLE_MINIMUM_CLUSTER_WIDTH_M
lidar_obstacle_maximum_cluster_width_m: $LIDAR_OBSTACLE_MAXIMUM_CLUSTER_WIDTH_M
lidar_obstacle_side_probe_inner_m: $LIDAR_OBSTACLE_SIDE_PROBE_INNER_M
lidar_obstacle_side_probe_outer_m: $LIDAR_OBSTACLE_SIDE_PROBE_OUTER_M
lidar_obstacle_lidar_x_m: $LIDAR_OBSTACLE_LIDAR_X_M
lidar_obstacle_lidar_y_m: $LIDAR_OBSTACLE_LIDAR_Y_M
lidar_obstacle_lidar_yaw_deg: $LIDAR_OBSTACLE_LIDAR_YAW_DEG
EOF
mv "$run_config_tmp" "$RUN_CONFIG_FILE"

set +u
source /opt/ros/humble/setup.bash
source "$WORKSPACE/install/setup.bash"
set -u

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-7}"
unset ROS_NAMESPACE || true

launch_pid=""

control_problem() {
  local title="$1"
  local topic="$2"
  local action="$3"
  echo >&2
  echo "[문제: $title] $topic 데이터가 준비되지 않았습니다." >&2
  echo "[확인 방법] $action" >&2
}

wait_for_control_message() {
  local topic="$1"
  local label="$2"
  local action="$3"
  local attempt
  printf '  [확인 중] %-12s %s\n' "$label" "$topic"
  for attempt in $(seq 1 20); do
    if ! kill -0 "$launch_pid" 2>/dev/null; then
      echo "[문제: 주행 제어 종료] 제어 launch가 준비 도중 종료되었습니다." >&2
      echo "[제어 로그] $CONTROL_LOG" >&2
      tail -n 40 "$CONTROL_LOG" >&2 || true
      return 1
    fi
    if timeout --signal=INT --kill-after=1s 3s \
      ros2 topic echo "$topic" --once \
      --qos-reliability best_effort >/dev/null 2>&1; then
      printf '  [OK] %-12s %s\n' "$label" "$topic"
      return 0
    fi
  done
  control_problem "$label" "$topic" "$action"
  return 1
}

cleanup() {
  set +e
  if [[ -n "$launch_pid" ]] && kill -0 -- "-$launch_pid" 2>/dev/null; then
    kill -TERM -- "-$launch_pid" 2>/dev/null
    for _ in $(seq 1 30); do
      kill -0 -- "-$launch_pid" 2>/dev/null || break
      sleep 0.1
    done
    if kill -0 -- "-$launch_pid" 2>/dev/null; then
      kill -KILL -- "-$launch_pid" 2>/dev/null
    fi
    wait "$launch_pid" 2>/dev/null || true
  fi
}
trap cleanup EXIT INT TERM

echo "Starting RULE base controller with mission overrides in shadow mode."
echo "Model and waypoint source switching are disabled."
if [[ "$TEST_PROFILE" == "cone_obstacle" ]]; then
  echo "Test profile: CONE-AS-VEHICLE YOLO+LiDAR AVOIDANCE > RULE."
elif [[ "$TEST_PROFILE" == "avoidance_only" ]]; then
  echo "Test profile: YOLO+LiDAR AVOIDANCE > RULE (cone disabled)."
else
  echo "Priority: CONE > YOLO+LiDAR AVOIDANCE > RULE."
fi
echo "Selected speed limit: $SPEED_COMMAND"
if [[ "$ADAPTIVE_STEERING_SPEED_ENABLED" == "true" ]]; then
  echo "Steering speed: <=${STEERING_SLOWDOWN_START_ANGLE}deg cap, to ${STEERING_FULL_SLOWDOWN_ANGLE}deg linear, then command ${STEERING_TURN_SPEED_COMMAND}"
else
  echo "Steering speed: OFF"
fi
if [[ "$STEERING_ONLY" == "true" ]]; then
  echo "Steering-only: enabled (/xycar_motor speed is always 0.0)"
fi
echo "Selected start target: WP$START_WAYPOINT"
echo "Curve control: LD=${LOOKAHEAD_DISTANCE}m, Stanley=${STANLEY_PERCENT}%"
echo "Control points: PP X=${PURE_PURSUIT_CONTROL_X_M}m, Stanley X=${STANLEY_CONTROL_X_M}m"
echo "Curve Stanley: gain=$STANLEY_GAIN, soft=${STANLEY_SOFTENING_MPS}m/s"
echo "Straight Stanley: ${STRAIGHT_STANLEY_PERCENT}%, gain=$STRAIGHT_STANLEY_GAIN, soft=${STRAIGHT_STANLEY_SOFTENING_MPS}m/s"
echo "Opposed Stanley: ${OPPOSED_STANLEY_PERCENT}%, latency preview=${CONTROL_LATENCY_PREVIEW_SEC}s"
echo "Left target correction: ${LEFT_OFFSET_CM}cm"
echo "Cone speed command: $CONE_SPEED_COMMAND"
echo "Avoidance: YOLO>=${VEHICLE_YOLO_MIN_CONFIDENCE}, entry=${VEHICLE_AVOIDANCE_ENTRY_DISTANCE_M}m, offsets=L${VEHICLE_LEFT_OFFSET_M}/R${VEHICLE_RIGHT_OFFSET_M}m"
if [[ "$CONE_AS_VEHICLE_OBSTACLE" == "true" ]]; then
  echo "Cone substitute: enabled, YOLO>=${CONE_AS_VEHICLE_MIN_CONFIDENCE} (slalom disabled)"
fi
echo "Avoidance speed cap: $VEHICLE_AVOIDANCE_SPEED_LIMIT_COMMAND | RViz: $ENABLE_RVIZ"
echo "Run settings: $RUN_CONFIG_FILE"
echo "Control detail log: $CONTROL_LOG"

setsid ros2 launch xycar_map_nav real_sequential_hybrid_drive.launch.py \
  drive_enabled:=false \
  steering_only:="$STEERING_ONLY" \
  gate_arming_required:=true \
  force_rule_only:=true \
  enable_rviz:="$ENABLE_RVIZ" \
  speed_command:="$SPEED_COMMAND" \
  cone_speed_command:="$CONE_SPEED_COMMAND" \
  lookahead_distance_m:="$LOOKAHEAD_DISTANCE" \
  pure_pursuit_weight:="$PURE_PURSUIT_WEIGHT" \
  pure_pursuit_control_x_m:="$PURE_PURSUIT_CONTROL_X_M" \
  stanley_control_x_m:="$STANLEY_CONTROL_X_M" \
  stanley_gain:="$STANLEY_GAIN" \
  stanley_softening_mps:="$STANLEY_SOFTENING_MPS" \
  straight_pure_pursuit_weight:="$STRAIGHT_PURE_PURSUIT_WEIGHT" \
  straight_stanley_gain:="$STRAIGHT_STANLEY_GAIN" \
  straight_stanley_softening_mps:="$STRAIGHT_STANLEY_SOFTENING_MPS" \
  opposed_stanley_weight:="$OPPOSED_STANLEY_WEIGHT" \
  control_latency_preview_sec:="$CONTROL_LATENCY_PREVIEW_SEC" \
  target_left_offset_m:="$LEFT_OFFSET_M" \
  perception_max_output_rate_hz:=15.0 \
  start_waypoint_number:="$START_WAYPOINT" \
  maximum_speed_command:=30.0 \
  start_cone:="$START_CONE" \
  start_object_detection:=true \
  vehicle_avoidance_enabled:=true \
  vehicle_yolo_min_confidence:="$VEHICLE_YOLO_MIN_CONFIDENCE" \
  cone_as_vehicle_obstacle:="$CONE_AS_VEHICLE_OBSTACLE" \
  cone_as_vehicle_min_confidence:="$CONE_AS_VEHICLE_MIN_CONFIDENCE" \
  vehicle_yolo_required_frames:="$VEHICLE_YOLO_REQUIRED_FRAMES" \
  vehicle_yolo_timeout_sec:="$VEHICLE_YOLO_TIMEOUT_SEC" \
  vehicle_camera_lidar_hfov_deg:="$VEHICLE_CAMERA_LIDAR_HFOV_DEG" \
  vehicle_camera_lidar_padding_deg:="$VEHICLE_CAMERA_LIDAR_PADDING_DEG" \
  vehicle_lidar_min_points:="$VEHICLE_LIDAR_MIN_POINTS" \
  vehicle_lidar_sector_memory_sec:="$VEHICLE_LIDAR_SECTOR_MEMORY_SEC" \
  vehicle_lidar_association_angle_margin_deg:="$VEHICLE_LIDAR_ASSOCIATION_ANGLE_MARGIN_DEG" \
  vehicle_lidar_association_distance_tolerance_m:="$VEHICLE_LIDAR_ASSOCIATION_DISTANCE_TOLERANCE_M" \
  vehicle_avoidance_immediate_on_yolo:="$VEHICLE_AVOIDANCE_IMMEDIATE_ON_YOLO" \
  vehicle_avoidance_entry_distance_m:="$VEHICLE_AVOIDANCE_ENTRY_DISTANCE_M" \
  vehicle_minimum_side_clearance_m:="$VEHICLE_MINIMUM_SIDE_CLEARANCE_M" \
  vehicle_left_offset_m:="$VEHICLE_LEFT_OFFSET_M" \
  vehicle_right_offset_m:="$VEHICLE_RIGHT_OFFSET_M" \
  vehicle_offset_rate_mps:="$VEHICLE_OFFSET_RATE_MPS" \
  vehicle_avoidance_speed_limit_command:="$VEHICLE_AVOIDANCE_SPEED_LIMIT_COMMAND" \
  vehicle_minimum_avoid_sec:="$VEHICLE_MINIMUM_AVOID_SEC" \
  vehicle_clear_hold_sec:="$VEHICLE_CLEAR_HOLD_SEC" \
  vehicle_return_hold_sec:="$VEHICLE_RETURN_HOLD_SEC" \
  vehicle_return_deadband_m:="$VEHICLE_RETURN_DEADBAND_M" \
  vehicle_body_length_m:="$VEHICLE_BODY_LENGTH_M" \
  vehicle_body_width_m:="$VEHICLE_BODY_WIDTH_M" \
  lidar_obstacle_detect_distance_m:="$LIDAR_OBSTACLE_DETECT_DISTANCE_M" \
  lidar_obstacle_minimum_distance_m:="$LIDAR_OBSTACLE_MINIMUM_DISTANCE_M" \
  lidar_obstacle_path_half_width_m:="$LIDAR_OBSTACLE_PATH_HALF_WIDTH_M" \
  lidar_obstacle_minimum_cluster_points:="$LIDAR_OBSTACLE_MINIMUM_CLUSTER_POINTS" \
  lidar_obstacle_maximum_scan_index_gap:="$LIDAR_OBSTACLE_MAXIMUM_SCAN_INDEX_GAP" \
  lidar_obstacle_maximum_cluster_gap_m:="$LIDAR_OBSTACLE_MAXIMUM_CLUSTER_GAP_M" \
  lidar_obstacle_minimum_cluster_width_m:="$LIDAR_OBSTACLE_MINIMUM_CLUSTER_WIDTH_M" \
  lidar_obstacle_maximum_cluster_width_m:="$LIDAR_OBSTACLE_MAXIMUM_CLUSTER_WIDTH_M" \
  lidar_obstacle_side_probe_inner_m:="$LIDAR_OBSTACLE_SIDE_PROBE_INNER_M" \
  lidar_obstacle_side_probe_outer_m:="$LIDAR_OBSTACLE_SIDE_PROBE_OUTER_M" \
  lidar_obstacle_lidar_x_m:="$LIDAR_OBSTACLE_LIDAR_X_M" \
  lidar_obstacle_lidar_y_m:="$LIDAR_OBSTACLE_LIDAR_Y_M" \
  lidar_obstacle_lidar_yaw_deg:="$LIDAR_OBSTACLE_LIDAR_YAW_DEG" \
  >"$CONTROL_LOG" 2>&1 &
launch_pid=$!

sleep 1
if ! kill -0 "$launch_pid" 2>/dev/null; then
  wait "$launch_pid"
  exit 1
fi

echo
echo "========== 주행 제어 준비 확인 =========="
wait_for_control_message \
  /perception/canonical_road_image \
  "차선 인지" \
  "카메라 영상과 lane_seg_lraspp_inference_node의 ERROR를 확인하세요."
wait_for_control_message \
  /hybrid/rule_candidate \
  "룰베이스" \
  "canonical 영상과 canonical_stanley_pursuit_driver를 확인하세요."
wait_for_control_message \
  /my_rule/object_detections \
  "객체 YOLO" \
  "best.pt 모델 경로와 my_rule_object_detection_node를 확인하세요."
wait_for_control_message \
  /hybrid_gate/status \
  "주행 선택기" \
  "LiDAR /scan과 sequential_hybrid_driver를 확인하세요."

echo
echo "========== READY | MODE=$RUN_MODE | SPEED=$SPEED_COMMAND | WP$START_WAYPOINT | LD=$LOOKAHEAD_DISTANCE | STANLEY=$STANLEY_PERCENT% | LEFT=${LEFT_OFFSET_CM}cm =========="
echo "Press SPACE once to RUN. Press SPACE again to STOP."

ros2 run xycar_map_nav space_drive_gate --ros-args \
  -p speed_command:="$SPEED_COMMAND" \
  -p maximum_speed_command:=30.0 \
  -p steering_only:="$STEERING_ONLY" \
  -p adaptive_steering_speed_enabled:="$ADAPTIVE_STEERING_SPEED_ENABLED" \
  -p turn_speed_command:="$STEERING_TURN_SPEED_COMMAND" \
  -p slowdown_start_angle_command:="$STEERING_SLOWDOWN_START_ANGLE" \
  -p full_slowdown_angle_command:="$STEERING_FULL_SLOWDOWN_ANGLE" \
  -p avoidance_target_label:="$AVOIDANCE_TARGET_LABEL" \
  -p avoidance_yolo_min_confidence:="$AVOIDANCE_DISPLAY_CONFIDENCE" \
  -p avoidance_entry_distance_m:="$VEHICLE_AVOIDANCE_ENTRY_DISTANCE_M" \
  -p avoidance_minimum_side_clearance_m:="$VEHICLE_MINIMUM_SIDE_CLEARANCE_M"
