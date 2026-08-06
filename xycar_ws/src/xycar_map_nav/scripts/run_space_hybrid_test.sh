#!/usr/bin/env bash

set -eo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "$(readlink -f "${BASH_SOURCE[0]}")")" && pwd)"
WORKSPACE="${XYCAR_WS:-$(cd -- "$SCRIPT_DIR/../../.." && pwd)}"
SPEED_COMMAND="${1:-}"
START_WAYPOINT="${2:-}"
RUN_MODE="${3:-rule}"
MODEL_PROFILE="${4:-speed100}"
LOOKAHEAD_DISTANCE="${5:-}"
STANLEY_PERCENT="${6:-}"
LEFT_OFFSET_CM="${7:-}"
CONTROL_LOG="/tmp/xycar_hybrid_control_$(date +%Y%m%d_%H%M%S).log"
RUN_CONFIG_FILE="${XYCAR_HYBRID_RUN_CONFIG_FILE:-/tmp/xycar_hybrid_run_config.yaml}"
CONE_SPEED_COMMAND="6.0"

if [[ -z "$SPEED_COMMAND" ]]; then
  if [[ -t 0 ]]; then
    read -r -p "Driving speed command [3.0-30.0, default 3.0]: " SPEED_COMMAND
  fi
  SPEED_COMMAND="${SPEED_COMMAND:-3.0}"
fi
if [[ ! "$SPEED_COMMAND" =~ ^[0-9]+([.][0-9]+)?$ ]] || \
  ! awk -v speed="$SPEED_COMMAND" 'BEGIN { exit !(speed >= 3.0 && speed <= 30.0) }'; then
  echo "ERROR: speed command must be a number from 3.0 to 30.0." >&2
  exit 2
fi
SPEED_COMMAND="$(awk -v speed="$SPEED_COMMAND" 'BEGIN { printf "%.3f", speed }')"

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
target_left_offset_cm: $LEFT_OFFSET_CM
target_left_offset_m: $LEFT_OFFSET_M
cone_speed_command: $CONE_SPEED_COMMAND
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
echo "Priority: CONE > YOLO+LiDAR AVOIDANCE > RULE."
echo "Selected speed limit: $SPEED_COMMAND"
echo "Selected start target: WP$START_WAYPOINT"
echo "Curve control: LD=${LOOKAHEAD_DISTANCE}m, Stanley=${STANLEY_PERCENT}%"
echo "Left target correction: ${LEFT_OFFSET_CM}cm"
echo "Cone speed command: $CONE_SPEED_COMMAND"
echo "Run settings: $RUN_CONFIG_FILE"
echo "Control detail log: $CONTROL_LOG"

setsid ros2 launch xycar_map_nav real_sequential_hybrid_drive.launch.py \
  drive_enabled:=false \
  gate_arming_required:=true \
  force_rule_only:=true \
  speed_command:="$SPEED_COMMAND" \
  cone_speed_command:="$CONE_SPEED_COMMAND" \
  lookahead_distance_m:="$LOOKAHEAD_DISTANCE" \
  pure_pursuit_weight:="$PURE_PURSUIT_WEIGHT" \
  target_left_offset_m:="$LEFT_OFFSET_M" \
  perception_max_output_rate_hz:=10.0 \
  start_waypoint_number:="$START_WAYPOINT" \
  maximum_speed_command:=30.0 \
  start_cone:=true \
  start_object_detection:=true \
  vehicle_avoidance_enabled:=true \
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
  -p maximum_speed_command:=30.0
