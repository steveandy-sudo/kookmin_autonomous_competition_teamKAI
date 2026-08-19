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
SPEED_COMMAND="${1:-${SPEED_COMMAND:-25.0}}"
CURVATURE_SPEED_CONTROL_ENABLED="${CURVATURE_SPEED_CONTROL_ENABLED:-true}"
# Leave these empty unless the operator explicitly overrides them. Their safe
# defaults depend on SPEED_COMMAND and are calculated after that value is
# validated (for example, speed 3 must produce curve/degraded defaults of 3).
CURVE_SPEED_COMMAND="${CURVE_SPEED_COMMAND:-}"
DEGRADED_PATH_SPEED_COMMAND="${DEGRADED_PATH_SPEED_COMMAND:-}"
CURVE_SPEED_EXIT_THRESHOLD_PER_M="${CURVE_SPEED_EXIT_THRESHOLD_PER_M:-0.12}"
CURVE_SPEED_CONFIRMATION_FRAMES="${CURVE_SPEED_CONFIRMATION_FRAMES:-2}"
CURVE_SPEED_RELEASE_FRAMES="${CURVE_SPEED_RELEASE_FRAMES:-3}"
DEGRADED_PATH_MINIMUM_SPAN_M="${DEGRADED_PATH_MINIMUM_SPAN_M:-0.60}"
LOOKAHEAD_DISTANCE="${2:-${LOOKAHEAD_DISTANCE:-0.30}}"
STANLEY_PERCENT="${3:-${STANLEY_PERCENT:-20}}"
LEFT_OFFSET_CM="${4:-${LEFT_OFFSET_CM:-0}}"
STRAIGHT_RIGHT_OFFSET_CM="${STRAIGHT_RIGHT_OFFSET_CM:-5.0}"
PURE_PURSUIT_CONTROL_X_M="${PURE_PURSUIT_CONTROL_X_M:-}"
STANLEY_CONTROL_X_M="${STANLEY_CONTROL_X_M:-}"
STANLEY_GAIN="${STANLEY_GAIN:-}"
STANLEY_SOFTENING_MPS="${STANLEY_SOFTENING_MPS:-}"
STRAIGHT_STANLEY_PERCENT="${STRAIGHT_STANLEY_PERCENT:-}"
STRAIGHT_STANLEY_GAIN="${STRAIGHT_STANLEY_GAIN:-}"
STRAIGHT_STANLEY_SOFTENING_MPS="${STRAIGHT_STANLEY_SOFTENING_MPS:-}"
OPPOSED_STANLEY_PERCENT="${OPPOSED_STANLEY_PERCENT:-}"
CONTROL_LATENCY_PREVIEW_SEC="${CONTROL_LATENCY_PREVIEW_SEC:-}"
CURVE_CONTROL_LATENCY_PREVIEW_SEC="${CURVE_CONTROL_LATENCY_PREVIEW_SEC:-}"
CURVE_CONTROL_LATENCY_MINIMUM_HOLD_SEC="${CURVE_CONTROL_LATENCY_MINIMUM_HOLD_SEC:-}"
STRAIGHT_PATH_CURVATURE_THRESHOLD="${STRAIGHT_PATH_CURVATURE_THRESHOLD:-0.24}"
STEERING_CURRENT_WEIGHT="${STEERING_CURRENT_WEIGHT:-0.35}"
STEERING_CURVE_CURRENT_WEIGHT="${STEERING_CURVE_CURRENT_WEIGHT:-0.80}"
STEERING_RATE_LIMIT_CMD_PER_SEC="${STEERING_RATE_LIMIT_CMD_PER_SEC:-180.0}"
STEERING_CURVE_RATE_LIMIT_CMD_PER_SEC="${STEERING_CURVE_RATE_LIMIT_CMD_PER_SEC:-300.0}"
STEERING_LEAD_TIME_SEC="${STEERING_LEAD_TIME_SEC:-0.08}"
STEERING_MAX_LEAD_COMMAND="${STEERING_MAX_LEAD_COMMAND:-6.0}"
CURVE_DETECTION_NEAR_X_M="${CURVE_DETECTION_NEAR_X_M:-0.20}"
CURVE_DETECTION_FAR_X_M="${CURVE_DETECTION_FAR_X_M:-1.20}"
CURVE_DETECTION_SEGMENT_COUNT="${CURVE_DETECTION_SEGMENT_COUNT:-1}"
CURVE_STEERING_MULTIPLIER_ENABLED="${CURVE_STEERING_MULTIPLIER_ENABLED:-false}"
CURVE_STEERING_MULTIPLIER_ACTIVATION_COMMAND="${CURVE_STEERING_MULTIPLIER_ACTIVATION_COMMAND:-20.0}"
CURVE_STEERING_MULTIPLIER="${CURVE_STEERING_MULTIPLIER:-1.5}"
ADAPTIVE_STEERING_SPEED_ENABLED="${ADAPTIVE_STEERING_SPEED_ENABLED:-}"
STEERING_TURN_SPEED_COMMAND="${STEERING_TURN_SPEED_COMMAND:-}"
STEERING_SLOWDOWN_START_ANGLE="${STEERING_SLOWDOWN_START_ANGLE:-}"
STEERING_FULL_SLOWDOWN_ANGLE="${STEERING_FULL_SLOWDOWN_ANGLE:-}"
CONTROL_LOG="/tmp/xycar_hybrid_control_$(date +%Y%m%d_%H%M%S).log"
RUN_CONFIG_FILE="${XYCAR_HYBRID_RUN_CONFIG_FILE:-/tmp/xycar_hybrid_run_config.yaml}"
CONE_SPEED_COMMAND="8.0"
CONE_SENSOR_PRESENCE_TIMEOUT_SEC="${CONE_SENSOR_PRESENCE_TIMEOUT_SEC:-0.5}"
SHORTCUT_W1_STEERING_START_DELAY_FRAMES="${SHORTCUT_W1_STEERING_START_DELAY_FRAMES:-4}"
SHORTCUT_W1_STEERING_DELAY_MISSING_TOLERANCE_FRAMES="${SHORTCUT_W1_STEERING_DELAY_MISSING_TOLERANCE_FRAMES:-2}"
SHORTCUT_MINIMUM_ENTRY_PROGRESS_M="${SHORTCUT_MINIMUM_ENTRY_PROGRESS_M:-0.50}"
SHORTCUT_PAIR_TRACK_HANDOFF_REQUIRED_FRAMES="${SHORTCUT_PAIR_TRACK_HANDOFF_REQUIRED_FRAMES:-2}"
SHORTCUT_W1_LOSS_HANDOFF_ENABLED="${SHORTCUT_W1_LOSS_HANDOFF_ENABLED:-true}"
SHORTCUT_MAXIMUM_ENTRY_STEERING_SEC="${SHORTCUT_MAXIMUM_ENTRY_STEERING_SEC:-1.5}"
SHORTCUT_W1_STEERING_HOLD_SEC="${SHORTCUT_W1_STEERING_HOLD_SEC:-1.0}"
SHORTCUT_ENTRY_DIRECTION_HOLD_COMMAND="${SHORTCUT_ENTRY_DIRECTION_HOLD_COMMAND:--30.0}"
SHORTCUT_ENTRY_SPEED_COMMAND="${SHORTCUT_ENTRY_SPEED_COMMAND:-9.0}"
TEST_PROFILE="${XYCAR_TEST_PROFILE:-integrated}"
ENABLE_RVIZ="${XYCAR_ENABLE_RVIZ:-false}"
START_CONE="${XYCAR_START_CONE:-true}"
STEERING_ONLY="${XYCAR_STEERING_ONLY:-false}"
VEHICLE_YOLO_MIN_CONFIDENCE="${VEHICLE_YOLO_MIN_CONFIDENCE:-0.45}"
CONE_AS_VEHICLE_OBSTACLE="${CONE_AS_VEHICLE_OBSTACLE:-false}"
CONE_AS_VEHICLE_MIN_CONFIDENCE="${CONE_AS_VEHICLE_MIN_CONFIDENCE:-0.50}"
VEHICLE_YOLO_REQUIRED_FRAMES="${VEHICLE_YOLO_REQUIRED_FRAMES:-1}"
VEHICLE_PREFERRED_SIDE_REQUIRED_FRAMES="${VEHICLE_PREFERRED_SIDE_REQUIRED_FRAMES:-1}"
VEHICLE_YOLO_TIMEOUT_SEC="${VEHICLE_YOLO_TIMEOUT_SEC:-1.00}"
VEHICLE_CAMERA_LIDAR_HFOV_DEG="${VEHICLE_CAMERA_LIDAR_HFOV_DEG:-60.0}"
VEHICLE_CAMERA_LIDAR_PADDING_DEG="${VEHICLE_CAMERA_LIDAR_PADDING_DEG:-3.0}"
VEHICLE_LIDAR_MIN_POINTS="${VEHICLE_LIDAR_MIN_POINTS:-2}"
VEHICLE_LIDAR_SECTOR_MEMORY_SEC="${VEHICLE_LIDAR_SECTOR_MEMORY_SEC:-0.50}"
VEHICLE_LIDAR_ASSOCIATION_ANGLE_MARGIN_DEG="${VEHICLE_LIDAR_ASSOCIATION_ANGLE_MARGIN_DEG:-2.0}"
VEHICLE_LIDAR_ASSOCIATION_DISTANCE_TOLERANCE_M="${VEHICLE_LIDAR_ASSOCIATION_DISTANCE_TOLERANCE_M:-0.35}"
RULE_PERCEPTION_BACKEND="${XYCAR_RULE_PERCEPTION_BACKEND:-canonical}"
LANE_PERCEPTION_LAUNCH="${XYCAR_LANE_PERCEPTION_LAUNCH:-lane_seg_far_centerline_extended_real.launch.py}"
CANONICAL_FORWARD_RANGE_M="${XYCAR_CANONICAL_FORWARD_RANGE_M:-}"
PERCEPTION_MAX_OUTPUT_RATE_HZ="${XYCAR_PERCEPTION_MAX_OUTPUT_RATE_HZ:-20.0}"
DIRECT_BEV_MODEL_PATH="${DIRECT_BEV_MODEL_PATH:-$WORKSPACE/src/lane_seg_control/models/best_512.onnx}"
DIRECT_BEV_IMAGE_SIZE="${DIRECT_BEV_IMAGE_SIZE:-512}"
DIRECT_BEV_CONFIDENCE="${DIRECT_BEV_CONFIDENCE:-0.20}"
DIRECT_BEV_YELLOW_CONFIDENCE="${DIRECT_BEV_YELLOW_CONFIDENCE:-0.40}"
DIRECT_BEV_CPU_THREADS="${DIRECT_BEV_CPU_THREADS:-4}"
DIRECT_BEV_COMMAND_RATE_HZ="${DIRECT_BEV_COMMAND_RATE_HZ:-10.0}"
DIRECT_BEV_PATH_TIMEOUT_SEC="${DIRECT_BEV_PATH_TIMEOUT_SEC:-1.50}"
VEHICLE_AVOIDANCE_IMMEDIATE_ON_YOLO="${VEHICLE_AVOIDANCE_IMMEDIATE_ON_YOLO:-true}"
VEHICLE_AVOIDANCE_ENTRY_DISTANCE_M="${VEHICLE_AVOIDANCE_ENTRY_DISTANCE_M:-1.20}"
VEHICLE_MINIMUM_SIDE_CLEARANCE_M="${VEHICLE_MINIMUM_SIDE_CLEARANCE_M:-0.70}"
VEHICLE_LEFT_OFFSET_M="${VEHICLE_LEFT_OFFSET_M:-0.20}"
VEHICLE_RIGHT_OFFSET_M="${VEHICLE_RIGHT_OFFSET_M:-0.20}"
VEHICLE_OFFSET_RATE_MPS="${VEHICLE_OFFSET_RATE_MPS:-0.50}"
VEHICLE_AVOIDANCE_SPEED_LIMIT_COMMAND="${VEHICLE_AVOIDANCE_SPEED_LIMIT_COMMAND:-8.0}"
VEHICLE_MINIMUM_AVOID_SEC="${VEHICLE_MINIMUM_AVOID_SEC:-0.50}"
VEHICLE_CLEAR_HOLD_SEC="${VEHICLE_CLEAR_HOLD_SEC:-0.50}"
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

case "$RULE_PERCEPTION_BACKEND" in
  canonical)
    if [[ ! -f "$WORKSPACE/src/lane_seg_control/launch/$LANE_PERCEPTION_LAUNCH" ]]; then
      echo "ERROR: lane perception launch not found: $LANE_PERCEPTION_LAUNCH" >&2
      exit 2
    fi
    if [[ -z "$CANONICAL_FORWARD_RANGE_M" ]]; then
      if [[ "$LANE_PERCEPTION_LAUNCH" == "lane_seg_far_centerline_extended_real.launch.py" ]]; then
        CANONICAL_FORWARD_RANGE_M=2.5
      else
        CANONICAL_FORWARD_RANGE_M=1.5
      fi
    fi
    if [[ ! "$CANONICAL_FORWARD_RANGE_M" =~ ^[0-9]+([.][0-9]+)?$ ]] || \
      ! awk -v value="$CANONICAL_FORWARD_RANGE_M" \
        'BEGIN { exit !(value >= 0.5 && value <= 5.0) }'; then
      echo "ERROR: canonical forward range must be from 0.5 to 5.0m." >&2
      exit 2
    fi
    if [[ ! "$PERCEPTION_MAX_OUTPUT_RATE_HZ" =~ ^[0-9]+([.][0-9]+)?$ ]] || \
      ! awk -v value="$PERCEPTION_MAX_OUTPUT_RATE_HZ" \
        'BEGIN { exit !(value >= 1.0 && value <= 30.0) }'; then
      echo "ERROR: perception output rate must be from 1.0 to 30.0Hz." >&2
      exit 2
    fi
    START_CANONICAL_PERCEPTION=true
    START_CANONICAL_RULE=true
    START_DIRECT_BEV_RULE=false
    RULE_READY_TOPIC=/perception/canonical_road_image
    RULE_READY_LABEL="canonical 차선 인지"
    ;;
  direct_bev)
    START_CANONICAL_PERCEPTION=false
    START_CANONICAL_RULE=false
    START_DIRECT_BEV_RULE=true
    RULE_READY_TOPIC=/lane_seg/source_image
    RULE_READY_LABEL="best_512 차선 인지"
    if [[ ! -f "$DIRECT_BEV_MODEL_PATH" ]]; then
      echo "ERROR: direct BEV model not found: $DIRECT_BEV_MODEL_PATH" >&2
      exit 2
    fi
    ;;
  *)
    echo "ERROR: XYCAR_RULE_PERCEPTION_BACKEND must be canonical or direct_bev." >&2
    exit 2
    ;;
esac

if [[ -z "$SPEED_COMMAND" ]]; then
  if [[ -t 0 ]]; then
    read -r -p "Driving speed command [3.0-30.0, default 25.0]: " SPEED_COMMAND
  fi
  SPEED_COMMAND="${SPEED_COMMAND:-25.0}"
fi
if [[ "$STEERING_ONLY" == "true" ]]; then
  SPEED_COMMAND=0.0
elif [[ ! "$SPEED_COMMAND" =~ ^[0-9]+([.][0-9]+)?$ ]] || \
  ! awk -v speed="$SPEED_COMMAND" 'BEGIN { exit !(speed >= 3.0 && speed <= 30.0) }'; then
    echo "ERROR: speed command must be a number from 3.0 to 30.0." >&2
    exit 2
fi
SPEED_COMMAND="$(awk -v speed="$SPEED_COMMAND" 'BEGIN { printf "%.3f", speed }')"

if [[ "$STEERING_ONLY" == "true" ]]; then
  CURVATURE_SPEED_CONTROL_ENABLED=false
  CURVE_SPEED_COMMAND=0.000
  DEGRADED_PATH_SPEED_COMMAND=0.000
else
  prompt_bool CURVATURE_SPEED_CONTROL_ENABLED \
    "Separate straight/curve speed" true
  if [[ "$CURVATURE_SPEED_CONTROL_ENABLED" == "true" ]]; then
    curve_default="$(awk -v speed="$SPEED_COMMAND" \
      'BEGIN { printf "%.3f", (speed < 16.0 ? speed : 16.0) }')"
    prompt_float CURVE_SPEED_COMMAND \
      "Confirmed curve speed command" "$curve_default" 3.0 "$SPEED_COMMAND"
    degraded_default="$(awk -v curve="$CURVE_SPEED_COMMAND" \
      'BEGIN { printf "%.3f", (curve < 15.0 ? curve : 15.0) }')"
    prompt_float DEGRADED_PATH_SPEED_COMMAND \
      "Short/remembered path speed command" "$degraded_default" 3.0 \
      "$CURVE_SPEED_COMMAND"
  else
    CURVE_SPEED_COMMAND="$SPEED_COMMAND"
    DEGRADED_PATH_SPEED_COMMAND="$SPEED_COMMAND"
  fi
fi

prompt_float CONE_SENSOR_PRESENCE_TIMEOUT_SEC \
  "Cone sensor-loss hold [s]" 0.5 0.1 10.0

prompt_bool ADAPTIVE_STEERING_SPEED_ENABLED \
  "Adaptive steering speed" true
if [[ "$ADAPTIVE_STEERING_SPEED_ENABLED" == "true" ]]; then
  prompt_float STEERING_SLOWDOWN_START_ANGLE \
    "Steering slowdown start angle" 18.0 0.0 42.0
  prompt_float STEERING_FULL_SLOWDOWN_ANGLE \
    "Steering full slowdown angle" 42.0 0.0 42.0
  prompt_float STEERING_TURN_SPEED_COMMAND \
    "Full-steering speed command" 12.0 0.0 30.0
  if ! awk \
    -v start="$STEERING_SLOWDOWN_START_ANGLE" \
    -v full="$STEERING_FULL_SLOWDOWN_ANGLE" \
    'BEGIN { exit !(full >= start) }'; then
    echo "ERROR: full slowdown angle must be >= start angle." >&2
    exit 2
  fi
else
  STEERING_SLOWDOWN_START_ANGLE=18.000
  STEERING_FULL_SLOWDOWN_ANGLE=42.000
  STEERING_TURN_SPEED_COMMAND=12.000
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
  "Curve Stanley cross-track gain" 1.20 0.0 10.0
prompt_float STANLEY_SOFTENING_MPS \
  "Curve Stanley softening [m/s]" 0.35 0.01 10.0
prompt_float STRAIGHT_STANLEY_PERCENT \
  "Straight Stanley percentage" 90.0 0.0 100.0
prompt_float STRAIGHT_STANLEY_GAIN \
  "Straight Stanley cross-track gain" 0.50 0.0 10.0
prompt_float STRAIGHT_STANLEY_SOFTENING_MPS \
  "Straight Stanley softening [m/s]" 0.65 0.01 10.0
prompt_float OPPOSED_STANLEY_PERCENT \
  "Opposed-term Stanley percentage" 70.0 0.0 100.0
prompt_float CONTROL_LATENCY_PREVIEW_SEC \
  "Straight control latency preview [s]" 0.20 0.0 2.0
prompt_float CURVE_CONTROL_LATENCY_PREVIEW_SEC \
  "Curve control latency preview [s]" 0.35 0.0 2.0
prompt_float CURVE_CONTROL_LATENCY_MINIMUM_HOLD_SEC \
  "Curve latency preview minimum hold [s]" 0.50 0.0 5.0
prompt_float STRAIGHT_PATH_CURVATURE_THRESHOLD \
  "Straight/curve curvature threshold [rad/m]" 0.24 0.0 5.0
prompt_float STEERING_CURRENT_WEIGHT \
  "Straight steering current weight" 0.35 0.0 1.0
prompt_float STEERING_CURVE_CURRENT_WEIGHT \
  "Curve steering current weight" 0.80 0.0 1.0
prompt_float STEERING_RATE_LIMIT_CMD_PER_SEC \
  "Straight steering rate [command/s]" 180.0 0.0 1000.0
prompt_float STEERING_CURVE_RATE_LIMIT_CMD_PER_SEC \
  "Curve steering rate [command/s]" 300.0 0.0 1000.0
prompt_float STEERING_LEAD_TIME_SEC \
  "Steering lead time [s]" 0.08 0.0 1.0
prompt_float STEERING_MAX_LEAD_COMMAND \
  "Maximum steering lead command" 6.0 0.0 42.0

STRAIGHT_PURE_PURSUIT_WEIGHT="$(awk \
  -v stanley="$STRAIGHT_STANLEY_PERCENT" \
  'BEGIN { printf "%.6f", 1.0 - (stanley / 100.0) }')"
OPPOSED_STANLEY_WEIGHT="$(awk -v stanley="$OPPOSED_STANLEY_PERCENT" \
  'BEGIN { printf "%.6f", stanley / 100.0 }')"

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
STRAIGHT_RIGHT_OFFSET_CM="${STRAIGHT_RIGHT_OFFSET_CM/,/.}"
if [[ ! "$STRAIGHT_RIGHT_OFFSET_CM" =~ ^[0-9]+([.][0-9]+)?$ ]] || \
  ! awk -v value="$STRAIGHT_RIGHT_OFFSET_CM" \
    'BEGIN { exit !(value >= 0.0 && value <= 20.0) }'; then
  echo "ERROR: straight right correction must be from 0 to 20cm." >&2
  exit 2
fi
STRAIGHT_RIGHT_OFFSET_CM="$(awk -v value="$STRAIGHT_RIGHT_OFFSET_CM" \
  'BEGIN { printf "%.1f", value }')"
STRAIGHT_RIGHT_OFFSET_M="$(awk -v value="$STRAIGHT_RIGHT_OFFSET_CM" \
  'BEGIN { printf "%.6f", value / 100.0 }')"

run_config_tmp="${RUN_CONFIG_FILE}.tmp.$$"
cat >"$run_config_tmp" <<EOF
recorded_at: "$(date --iso-8601=seconds)"
run_mode: "rule"
rule_perception_backend: "$RULE_PERCEPTION_BACKEND"
lane_perception_launch: "$LANE_PERCEPTION_LAUNCH"
canonical_forward_range_m: $CANONICAL_FORWARD_RANGE_M
perception_max_output_rate_hz: $PERCEPTION_MAX_OUTPUT_RATE_HZ
direct_bev_model_path: "$DIRECT_BEV_MODEL_PATH"
direct_bev_image_size: $DIRECT_BEV_IMAGE_SIZE
direct_bev_confidence: $DIRECT_BEV_CONFIDENCE
direct_bev_yellow_confidence: $DIRECT_BEV_YELLOW_CONFIDENCE
direct_bev_path_timeout_sec: $DIRECT_BEV_PATH_TIMEOUT_SEC
speed_command: $SPEED_COMMAND
curvature_speed_control_enabled: $CURVATURE_SPEED_CONTROL_ENABLED
curve_speed_command: $CURVE_SPEED_COMMAND
degraded_path_speed_command: $DEGRADED_PATH_SPEED_COMMAND
curve_speed_exit_threshold_per_m: $CURVE_SPEED_EXIT_THRESHOLD_PER_M
curve_speed_confirmation_frames: $CURVE_SPEED_CONFIRMATION_FRAMES
curve_speed_release_frames: $CURVE_SPEED_RELEASE_FRAMES
degraded_path_minimum_span_m: $DEGRADED_PATH_MINIMUM_SPAN_M
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
curve_control_latency_preview_sec: $CURVE_CONTROL_LATENCY_PREVIEW_SEC
curve_control_latency_minimum_hold_sec: $CURVE_CONTROL_LATENCY_MINIMUM_HOLD_SEC
straight_path_curvature_threshold: $STRAIGHT_PATH_CURVATURE_THRESHOLD
steering_current_weight: $STEERING_CURRENT_WEIGHT
steering_curve_current_weight: $STEERING_CURVE_CURRENT_WEIGHT
steering_rate_limit_cmd_per_sec: $STEERING_RATE_LIMIT_CMD_PER_SEC
steering_curve_rate_limit_cmd_per_sec: $STEERING_CURVE_RATE_LIMIT_CMD_PER_SEC
steering_lead_time_sec: $STEERING_LEAD_TIME_SEC
steering_max_lead_command: $STEERING_MAX_LEAD_COMMAND
curve_detection_near_x_m: $CURVE_DETECTION_NEAR_X_M
curve_detection_far_x_m: $CURVE_DETECTION_FAR_X_M
curve_detection_segment_count: $CURVE_DETECTION_SEGMENT_COUNT
curve_steering_multiplier_enabled: $CURVE_STEERING_MULTIPLIER_ENABLED
curve_steering_multiplier_activation_command: $CURVE_STEERING_MULTIPLIER_ACTIVATION_COMMAND
curve_steering_multiplier: $CURVE_STEERING_MULTIPLIER
target_left_offset_cm: $LEFT_OFFSET_CM
target_left_offset_m: $LEFT_OFFSET_M
straight_target_right_offset_cm: $STRAIGHT_RIGHT_OFFSET_CM
straight_target_right_offset_m: $STRAIGHT_RIGHT_OFFSET_M
cone_speed_command: $CONE_SPEED_COMMAND
cone_sensor_presence_timeout_sec: $CONE_SENSOR_PRESENCE_TIMEOUT_SEC
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
vehicle_preferred_side_required_frames: $VEHICLE_PREFERRED_SIDE_REQUIRED_FRAMES
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
mission_log_pid=""
RUN_LOCK_FILE="${XYCAR_HYBRID_RUN_LOCK_FILE:-/tmp/xycar_hybrid_run.lock}"

# Keep one authoritative controller stack. A stale stack can continue
# publishing an older speed even when this invocation records a new setting.
exec 9>"$RUN_LOCK_FILE"
if ! flock -n 9; then
  echo "[문제: 통합 주행 중복 실행] 다른 통합 주행 스크립트가 아직 실행 중입니다." >&2
  echo "[확인 방법] 기존 주행 터미널에서 Ctrl+C로 종료한 뒤 다시 실행하세요." >&2
  exit 1
fi

control_problem() {
  local title="$1"
  local topic="$2"
  local action="$3"
  echo >&2
  echo "[문제: $title] $topic 데이터가 준비되지 않았습니다." >&2
  echo "[확인 방법] $action" >&2
}

check_existing_control_stack() {
  local nodes
  local node
  local conflicts=()
  nodes="$(ros2 node list 2>/dev/null || true)"
  for node in \
    /canonical_stanley_pursuit_driver \
    /sequential_hybrid_driver \
    /space_drive_gate \
    /my_rule_object_detection_node \
    /my_rule_cone_node; do
    if grep -Fxq "$node" <<<"$nodes"; then
      conflicts+=("$node")
    fi
  done
  if (( ${#conflicts[@]} > 0 )); then
    echo "[문제: 이전 주행 제어기 잔존] 새 설정을 덮어쓸 노드가 이미 실행 중입니다." >&2
    printf '  - %s\n' "${conflicts[@]}" >&2
    echo "[확인 방법] 기존 주행 터미널에서 Ctrl+C로 종료하고, 위 노드가 사라진 뒤 다시 실행하세요." >&2
    return 1
  fi
}

read_double_parameter() {
  local node="$1"
  local parameter="$2"
  ros2 param get "$node" "$parameter" 2>/dev/null \
    | awk '/value is:/ { print $NF; exit }'
}

read_parameter_value() {
  local node="$1"
  local parameter="$2"
  local attempt
  local value
  for attempt in $(seq 1 10); do
    value="$(
      ros2 param get "$node" "$parameter" 2>/dev/null \
        | awk '/value is:/ { print $NF; exit }'
    )"
    if [[ -n "$value" ]]; then
      printf '%s\n' "$value"
      return 0
    fi
    sleep 0.1
  done
  return 1
}

verify_runtime_parameter() {
  local node="$1"
  local parameter="$2"
  local expected="$3"
  local label="$4"
  local actual
  actual="$(read_parameter_value "$node" "$parameter" || true)"
  if [[ -z "$actual" ]]; then
    echo "[문제: $label 확인 실패] $node 의 $parameter 값을 읽지 못했습니다." >&2
    return 1
  fi
  if [[ "${actual,,}" != "${expected,,}" ]]; then
    echo "[문제: $label 불일치] 기대=$expected, 실제=$actual" >&2
    echo "[확인 방법] 현재 workspace만 source한 새 터미널에서 다시 실행하세요." >&2
    return 1
  fi
  printf '  [OK] %-20s %s=%s\n' "$label" "$parameter" "$actual"
}

verify_runtime_cruise_speed() {
  local actual
  actual="$(read_double_parameter \
    /canonical_stanley_pursuit_driver cruise_speed_command)"
  if [[ -z "$actual" ]]; then
    echo "[문제: 직선 속도 확인 실패] cruise_speed_command를 읽지 못했습니다." >&2
    echo "[확인 방법] canonical_stanley_pursuit_driver의 파라미터 서비스를 확인하세요." >&2
    return 1
  fi
  if ! awk -v requested="$SPEED_COMMAND" -v actual="$actual" \
    'BEGIN { difference=requested-actual; if (difference<0) difference=-difference; exit !(difference < 0.001) }'; then
    echo "[문제: 직선 속도 불일치] 입력=$SPEED_COMMAND, 실제 노드=$actual" >&2
    echo "[확인 방법] 이전 주행 노드를 모두 종료하고 이 스크립트만 다시 실행하세요." >&2
    return 1
  fi
  printf '  [OK] 직선 복귀 속도 입력=%s, 실제=%0.3f\n' \
    "$SPEED_COMMAND" "$actual"
  printf 'runtime_cruise_speed_command: %.3f\n' "$actual" \
    >>"$RUN_CONFIG_FILE"
}

verify_runtime_control_contract() {
  verify_runtime_parameter \
    /sequential_hybrid_driver drive_enabled False \
    "선택기 직접출력 차단"
  verify_runtime_parameter \
    /sequential_hybrid_driver gate_arming_required True \
    "SPACE 게이트 연동"
  verify_runtime_parameter \
    /sequential_hybrid_driver shadow_motor_topic \
    /hybrid_gate/xycar_motor_shadow "선택기 shadow 출력"
  verify_runtime_parameter \
    /sequential_hybrid_driver scan_topic /scan "선택기 LiDAR 입력"
  if [[ "$RULE_PERCEPTION_BACKEND" == "canonical" ]]; then
    verify_runtime_parameter \
      /canonical_stanley_pursuit_driver drive_enabled False \
      "룰 직접출력 차단"
    verify_runtime_parameter \
      /canonical_stanley_pursuit_driver shadow_motor_topic \
      /hybrid/rule_candidate "룰 candidate 출력"
    verify_runtime_cruise_speed
  fi
}

wait_for_control_message() {
  local topic="$1"
  local label="$2"
  local action="$3"
  local field="${4:-}"
  local attempt
  printf '  [확인 중] %-12s %s\n' "$label" "$topic"
  for attempt in $(seq 1 6); do
    if ! kill -0 "$launch_pid" 2>/dev/null; then
      echo "[문제: 주행 제어 종료] 제어 launch가 준비 도중 종료되었습니다." >&2
      echo "[제어 로그] $CONTROL_LOG" >&2
      tail -n 40 "$CONTROL_LOG" >&2 || true
      return 1
    fi
    local echo_command=(
      ros2 topic echo "$topic" --once
      --qos-reliability best_effort
    )
    if [[ -n "$field" ]]; then
      echo_command+=(--field "$field")
    fi
    if timeout --signal=INT --kill-after=1s 8s \
      "${echo_command[@]}" >/dev/null 2>&1; then
      printf '  [OK] %-12s %s\n' "$label" "$topic"
      return 0
    fi
    printf '  [대기 %d/6] %s 메시지를 기다리는 중입니다.\n' \
      "$attempt" "$label"
  done
  control_problem "$label" "$topic" "$action"
  return 1
}

cleanup() {
  set +e
  if [[ -n "$mission_log_pid" ]] && \
    kill -0 -- "-$mission_log_pid" 2>/dev/null; then
    kill -TERM -- "-$mission_log_pid" 2>/dev/null
  fi
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

check_existing_control_stack

echo "Starting RULE base controller with mission overrides in shadow mode."
echo "Model and waypoint source switching are disabled."
if [[ "$RULE_PERCEPTION_BACKEND" == "direct_bev" ]]; then
  echo "RULE perception: direct BEV / best_512 ONNX (canonical normalization bypassed)."
  echo "RULE model: $DIRECT_BEV_MODEL_PATH"
  echo "RULE path hold: ${DIRECT_BEV_PATH_TIMEOUT_SEC}s after the last valid BEV path."
else
  echo "RULE perception: canonical LR-ASPP."
fi
if [[ "$TEST_PROFILE" == "cone_obstacle" ]]; then
  echo "Test profile: CONE-AS-VEHICLE YOLO+LiDAR AVOIDANCE > RULE."
elif [[ "$TEST_PROFILE" == "avoidance_only" ]]; then
  echo "Test profile: YOLO+LiDAR AVOIDANCE > RULE (cone disabled)."
else
  echo "Priority: TRAFFIC > SHORTCUT > CONE > YOLO+LiDAR AVOIDANCE > RULE."
fi
echo "Selected speed limit: $SPEED_COMMAND"
if [[ "$CURVATURE_SPEED_CONTROL_ENABLED" == "true" ]]; then
  echo "Path speed: straight $SPEED_COMMAND, curve $CURVE_SPEED_COMMAND, degraded $DEGRADED_PATH_SPEED_COMMAND"
  echo "Curve speed latch: enter ${CURVE_SPEED_CONFIRMATION_FRAMES} frames, release ${CURVE_SPEED_RELEASE_FRAMES} frames, exit ${CURVE_SPEED_EXIT_THRESHOLD_PER_M}rad/m"
else
  echo "Path speed: OFF"
fi
if [[ "$ADAPTIVE_STEERING_SPEED_ENABLED" == "true" ]]; then
  echo "Steering speed: <=${STEERING_SLOWDOWN_START_ANGLE}deg cap, to ${STEERING_FULL_SLOWDOWN_ANGLE}deg linear, then command ${STEERING_TURN_SPEED_COMMAND}"
else
  echo "Steering speed: OFF"
fi
if [[ "$STEERING_ONLY" == "true" ]]; then
  echo "Steering-only: enabled (/xycar_motor speed is always 0.0)"
fi
echo "Curve control: LD=${LOOKAHEAD_DISTANCE}m, Stanley=${STANLEY_PERCENT}%"
echo "Control points: PP X=${PURE_PURSUIT_CONTROL_X_M}m, Stanley X=${STANLEY_CONTROL_X_M}m"
echo "Curve Stanley: gain=$STANLEY_GAIN, soft=${STANLEY_SOFTENING_MPS}m/s"
echo "Straight Stanley: ${STRAIGHT_STANLEY_PERCENT}%, gain=$STRAIGHT_STANLEY_GAIN, soft=${STRAIGHT_STANLEY_SOFTENING_MPS}m/s"
echo "Opposed Stanley: ${OPPOSED_STANLEY_PERCENT}%, latency preview=straight ${CONTROL_LATENCY_PREVIEW_SEC}s/curve ${CURVE_CONTROL_LATENCY_PREVIEW_SEC}s, curve minimum hold=${CURVE_CONTROL_LATENCY_MINIMUM_HOLD_SEC}s"
echo "Straight/curve threshold: ${STRAIGHT_PATH_CURVATURE_THRESHOLD}rad/m"
echo "Steering smoothing: straight ${STEERING_CURRENT_WEIGHT}/${STEERING_RATE_LIMIT_CMD_PER_SEC}, curve ${STEERING_CURVE_CURRENT_WEIGHT}/${STEERING_CURVE_RATE_LIMIT_CMD_PER_SEC}"
echo "Steering lead: ${STEERING_LEAD_TIME_SEC}s, max ${STEERING_MAX_LEAD_COMMAND} command"
echo "Curve detection: ${CURVE_DETECTION_NEAR_X_M}-${CURVE_DETECTION_FAR_X_M}m, ${CURVE_DETECTION_SEGMENT_COUNT} segments"
echo "Curve steering multiplier: ${CURVE_STEERING_MULTIPLIER_ENABLED}, |angle|>=${CURVE_STEERING_MULTIPLIER_ACTIVATION_COMMAND} x${CURVE_STEERING_MULTIPLIER}, clamp +/-42"
echo "Left target correction: ${LEFT_OFFSET_CM}cm"
echo "Straight-only right correction: ${STRAIGHT_RIGHT_OFFSET_CM}cm"
echo "Shortcut entry speed cap: ${SHORTCUT_ENTRY_SPEED_COMMAND}"
echo "Shortcut W1 steering start delay: ${SHORTCUT_W1_STEERING_START_DELAY_FRAMES} valid frames, missing tolerance ${SHORTCUT_W1_STEERING_DELAY_MISSING_TOLERANCE_FRAMES} frames"
echo "Shortcut handoff: progress ${SHORTCUT_MINIMUM_ENTRY_PROGRESS_M}m, pair ${SHORTCUT_PAIR_TRACK_HANDOFF_REQUIRED_FRAMES} frames, W1-loss fallback ${SHORTCUT_W1_LOSS_HANDOFF_ENABLED}"
echo "Shortcut W1 steering maximum active time: ${SHORTCUT_MAXIMUM_ENTRY_STEERING_SEC}s"
echo "Cone speed command: $CONE_SPEED_COMMAND"
echo "Cone sensor-loss hold: ${CONE_SENSOR_PRESENCE_TIMEOUT_SEC}s"
if [[ "$VEHICLE_AVOIDANCE_IMMEDIATE_ON_YOLO" == "true" ]]; then
  echo "Avoidance: YOLO>=${VEHICLE_YOLO_MIN_CONFIDENCE}, one-frame immediate entry (LiDAR distance=telemetry), offsets=L${VEHICLE_LEFT_OFFSET_M}/R${VEHICLE_RIGHT_OFFSET_M}m"
else
  echo "Avoidance: YOLO>=${VEHICLE_YOLO_MIN_CONFIDENCE}, entry=${VEHICLE_AVOIDANCE_ENTRY_DISTANCE_M}m, offsets=L${VEHICLE_LEFT_OFFSET_M}/R${VEHICLE_RIGHT_OFFSET_M}m"
fi
if [[ "$CONE_AS_VEHICLE_OBSTACLE" == "true" ]]; then
  echo "Cone substitute: enabled, YOLO>=${CONE_AS_VEHICLE_MIN_CONFIDENCE} (slalom disabled)"
fi
echo "Avoidance speed cap: $VEHICLE_AVOIDANCE_SPEED_LIMIT_COMMAND | RViz: $ENABLE_RVIZ"
echo "Lane perception: $LANE_PERCEPTION_LAUNCH | forward=${CANONICAL_FORWARD_RANGE_M}m | max=${PERCEPTION_MAX_OUTPUT_RATE_HZ}Hz"
echo "Run settings: $RUN_CONFIG_FILE"
echo "Control detail log: $CONTROL_LOG"

setsid ros2 launch xycar_map_nav real_sequential_hybrid_drive.launch.py \
  drive_enabled:=false \
  steering_only:="$STEERING_ONLY" \
  gate_arming_required:=true \
  force_rule_only:=true \
  enable_rviz:="$ENABLE_RVIZ" \
  start_perception:="$START_CANONICAL_PERCEPTION" \
  lane_perception_launch:="$LANE_PERCEPTION_LAUNCH" \
  start_rule:="$START_CANONICAL_RULE" \
  start_direct_bev_rule:="$START_DIRECT_BEV_RULE" \
  direct_bev_model_path:="$DIRECT_BEV_MODEL_PATH" \
  direct_bev_image_size:="$DIRECT_BEV_IMAGE_SIZE" \
  direct_bev_confidence:="$DIRECT_BEV_CONFIDENCE" \
  direct_bev_yellow_confidence:="$DIRECT_BEV_YELLOW_CONFIDENCE" \
  direct_bev_cpu_threads:="$DIRECT_BEV_CPU_THREADS" \
  direct_bev_command_rate_hz:="$DIRECT_BEV_COMMAND_RATE_HZ" \
  direct_bev_path_timeout_sec:="$DIRECT_BEV_PATH_TIMEOUT_SEC" \
  speed_command:="$SPEED_COMMAND" \
  curvature_speed_control_enabled:="$CURVATURE_SPEED_CONTROL_ENABLED" \
  curve_speed_command:="$CURVE_SPEED_COMMAND" \
  degraded_path_speed_command:="$DEGRADED_PATH_SPEED_COMMAND" \
  curve_speed_exit_threshold_per_m:="$CURVE_SPEED_EXIT_THRESHOLD_PER_M" \
  curve_speed_confirmation_frames:="$CURVE_SPEED_CONFIRMATION_FRAMES" \
  curve_speed_release_frames:="$CURVE_SPEED_RELEASE_FRAMES" \
  degraded_path_minimum_span_m:="$DEGRADED_PATH_MINIMUM_SPAN_M" \
  selector_minimum_speed_command:=3.0 \
  cone_speed_command:="$CONE_SPEED_COMMAND" \
  cone_sensor_presence_timeout_sec:="$CONE_SENSOR_PRESENCE_TIMEOUT_SEC" \
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
  curve_control_latency_preview_sec:="$CURVE_CONTROL_LATENCY_PREVIEW_SEC" \
  curve_control_latency_minimum_hold_sec:="$CURVE_CONTROL_LATENCY_MINIMUM_HOLD_SEC" \
  straight_path_curvature_threshold:="$STRAIGHT_PATH_CURVATURE_THRESHOLD" \
  steering_current_weight:="$STEERING_CURRENT_WEIGHT" \
  steering_curve_current_weight:="$STEERING_CURVE_CURRENT_WEIGHT" \
  steering_rate_limit_cmd_per_sec:="$STEERING_RATE_LIMIT_CMD_PER_SEC" \
  steering_curve_rate_limit_cmd_per_sec:="$STEERING_CURVE_RATE_LIMIT_CMD_PER_SEC" \
  steering_lead_time_sec:="$STEERING_LEAD_TIME_SEC" \
  steering_max_lead_command:="$STEERING_MAX_LEAD_COMMAND" \
  curve_detection_near_x_m:="$CURVE_DETECTION_NEAR_X_M" \
  curve_detection_far_x_m:="$CURVE_DETECTION_FAR_X_M" \
  curve_detection_segment_count:="$CURVE_DETECTION_SEGMENT_COUNT" \
  curve_steering_multiplier_enabled:="$CURVE_STEERING_MULTIPLIER_ENABLED" \
  curve_steering_multiplier_activation_command:="$CURVE_STEERING_MULTIPLIER_ACTIVATION_COMMAND" \
  curve_steering_multiplier:="$CURVE_STEERING_MULTIPLIER" \
  target_left_offset_m:="$LEFT_OFFSET_M" \
  straight_target_right_offset_m:="$STRAIGHT_RIGHT_OFFSET_M" \
  perception_max_output_rate_hz:="$PERCEPTION_MAX_OUTPUT_RATE_HZ" \
  canonical_forward_range_m:="$CANONICAL_FORWARD_RANGE_M" \
  maximum_speed_command:=30.0 \
  start_cone:="$START_CONE" \
  start_object_detection:=true \
  start_shortcut:=true \
  shortcut_handoff_to_rule:=true \
  shortcut_w1_steering_start_delay_frames:="$SHORTCUT_W1_STEERING_START_DELAY_FRAMES" \
  shortcut_w1_steering_delay_missing_tolerance_frames:="$SHORTCUT_W1_STEERING_DELAY_MISSING_TOLERANCE_FRAMES" \
  shortcut_minimum_entry_progress_m:="$SHORTCUT_MINIMUM_ENTRY_PROGRESS_M" \
  shortcut_pair_track_handoff_required_frames:="$SHORTCUT_PAIR_TRACK_HANDOFF_REQUIRED_FRAMES" \
  shortcut_w1_loss_handoff_enabled:="$SHORTCUT_W1_LOSS_HANDOFF_ENABLED" \
  shortcut_maximum_entry_steering_sec:="$SHORTCUT_MAXIMUM_ENTRY_STEERING_SEC" \
  shortcut_w1_steering_hold_sec:="$SHORTCUT_W1_STEERING_HOLD_SEC" \
  shortcut_entry_direction_hold_command:="$SHORTCUT_ENTRY_DIRECTION_HOLD_COMMAND" \
  shortcut_entry_speed_command:="$SHORTCUT_ENTRY_SPEED_COMMAND" \
  vehicle_avoidance_enabled:=true \
  vehicle_yolo_min_confidence:="$VEHICLE_YOLO_MIN_CONFIDENCE" \
  cone_as_vehicle_obstacle:="$CONE_AS_VEHICLE_OBSTACLE" \
  cone_as_vehicle_min_confidence:="$CONE_AS_VEHICLE_MIN_CONFIDENCE" \
  vehicle_yolo_required_frames:="$VEHICLE_YOLO_REQUIRED_FRAMES" \
  vehicle_preferred_side_required_frames:="$VEHICLE_PREFERRED_SIDE_REQUIRED_FRAMES" \
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

# Keep the complete launch output in CONTROL_LOG while forwarding only
# failures that need immediate operator attention. Stable driving-mode changes
# and stop reasons are printed by space_drive_gate itself.
setsid bash -c '
  log_file="$1"
  owner_pid="$2"
  stdbuf -oL tail --pid="$owner_pid" -n 0 -F "$log_file" 2>/dev/null |
    stdbuf -oL grep --line-buffered -E \
      "W1 spatial gate arrived after search timeout|W1 search timeout|candidate stale|safe stop|process has died|Traceback|ERROR"
' _ "$CONTROL_LOG" "$launch_pid" &
mission_log_pid=$!

echo
echo "========== 주행 제어 준비 확인 =========="
wait_for_control_message \
  "$RULE_READY_TOPIC" \
  "$RULE_READY_LABEL" \
  "카메라 영상과 선택한 lane segmentation 노드의 ERROR를 확인하세요." \
  header.stamp
verify_runtime_control_contract
wait_for_control_message \
  /scan \
  "LiDAR" \
  "xycar_lidar_node와 /scan 10Hz 출력을 확인하세요."
wait_for_control_message \
  /hybrid/rule_candidate \
  "룰베이스" \
  "$RULE_READY_TOPIC 과 Stanley/Pure Pursuit 제어기를 확인하세요."
wait_for_control_message \
  /my_rule/object_detections \
  "객체 YOLO" \
  "best.pt 모델 경로와 my_rule_object_detection_node를 확인하세요."
wait_for_control_message \
  /hybrid_gate/status \
  "주행 선택기" \
  "LiDAR /scan과 sequential_hybrid_driver를 확인하세요."
echo
echo "========== READY | MODE=RULE | SPEED=$SPEED_COMMAND | LD=$LOOKAHEAD_DISTANCE | STANLEY=$STANLEY_PERCENT% | LEFT=${LEFT_OFFSET_CM}cm =========="
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
