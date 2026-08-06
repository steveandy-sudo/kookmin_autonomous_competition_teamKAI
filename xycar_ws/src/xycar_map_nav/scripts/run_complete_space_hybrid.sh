#!/usr/bin/env bash

set -eo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "$(readlink -f "${BASH_SOURCE[0]}")")" && pwd)"
SOURCE_WORKSPACE="$(cd -- "$SCRIPT_DIR/../../.." && pwd)"
WORKSPACE="${XYCAR_WS:-$SOURCE_WORKSPACE}"
if [[ ! -x "$WORKSPACE/src/xycar_map_nav/scripts/run_space_hybrid_test.sh" ]]; then
  if [[ -n "${XYCAR_WS:-}" ]]; then
    echo "[경고] XYCAR_WS=$XYCAR_WS 에 현재 통합 주행 코드가 없습니다."
    echo "[자동 복구] 이 스크립트의 workspace를 사용합니다: $SOURCE_WORKSPACE"
  fi
  WORKSPACE="$SOURCE_WORKSPACE"
fi
export XYCAR_WS="$WORKSPACE"
CAMERA_DEVICE="/dev/v4l/by-id/usb-HD_USB_Camera_HD_USB_Camera-video-index0"
SPEED_COMMAND="${1:-}"
START_WAYPOINT="${2:-}"
RUN_MODE="${3:-${RUN_MODE:-rule}}"
MODEL_PROFILE="${4:-${MODEL_PROFILE:-speed100}}"
LOOKAHEAD_DISTANCE="${5:-${LOOKAHEAD_DISTANCE:-}}"
STANLEY_PERCENT="${6:-${STANLEY_PERCENT:-}}"
LEFT_OFFSET_CM="${7:-${LEFT_OFFSET_CM:-}}"
PURE_PURSUIT_CONTROL_X_M="${PURE_PURSUIT_CONTROL_X_M:-}"
STANLEY_CONTROL_X_M="${STANLEY_CONTROL_X_M:-}"
STANLEY_GAIN="${STANLEY_GAIN:-}"
STANLEY_SOFTENING_MPS="${STANLEY_SOFTENING_MPS:-}"
STRAIGHT_STANLEY_PERCENT="${STRAIGHT_STANLEY_PERCENT:-}"
STRAIGHT_STANLEY_GAIN="${STRAIGHT_STANLEY_GAIN:-}"
STRAIGHT_STANLEY_SOFTENING_MPS="${STRAIGHT_STANLEY_SOFTENING_MPS:-}"
OPPOSED_STANLEY_PERCENT="${OPPOSED_STANLEY_PERCENT:-}"
CONTROL_LATENCY_PREVIEW_SEC="${CONTROL_LATENCY_PREVIEW_SEC:-}"
STEERING_ONLY="${XYCAR_STEERING_ONLY:-false}"
SENSOR_LOG="/tmp/xycar_hybrid_sensors_$(date +%Y%m%d_%H%M%S).log"
SENSOR_PID=""

problem() {
  local title="$1"
  local detail="$2"
  local action="$3"
  echo >&2
  echo "[문제: $title] $detail" >&2
  echo "[확인 방법] $action" >&2
}

prompt_float() {
  local variable_name="$1"
  local prompt="$2"
  local default_value="$3"
  local minimum="$4"
  local maximum="$5"
  local value="${!variable_name:-}"
  if [[ -z "$value" ]]; then
    read -r -p "$prompt [기본 $default_value]: " value
    value="${value:-$default_value}"
  fi
  value="${value/,/.}"
  if [[ ! "$value" =~ ^-?[0-9]+([.][0-9]+)?$ ]] || \
    ! awk -v value="$value" -v minimum="$minimum" -v maximum="$maximum" \
      'BEGIN { exit !(value >= minimum && value <= maximum) }'; then
    problem \
      "제어 파라미터 입력 오류" \
      "'$value'은 $prompt 값으로 사용할 수 없습니다." \
      "$minimum 부터 $maximum 사이 숫자를 입력하세요."
    exit 2
  fi
  printf -v "$variable_name" '%.3f' "$value"
}

if [[ "$STEERING_ONLY" == "true" ]]; then
  SPEED_COMMAND=0.0
elif [[ -z "$SPEED_COMMAND" ]]; then
  read -r -p "주행 속도 command [3.0-30.0, 기본 3.0]: " SPEED_COMMAND
  SPEED_COMMAND="${SPEED_COMMAND:-3.0}"
fi
if [[ "$STEERING_ONLY" != "true" ]]; then
  if [[ ! "$SPEED_COMMAND" =~ ^[0-9]+([.][0-9]+)?$ ]] || \
    ! awk -v speed="$SPEED_COMMAND" \
      'BEGIN { exit !(speed >= 3.0 && speed <= 30.0) }'; then
    problem \
      "속도 입력 오류" \
      "'$SPEED_COMMAND'은 사용할 수 없는 속도입니다." \
      "3.0부터 30.0 사이의 숫자 하나를 입력하세요. 예: 5"
    exit 2
  fi
fi
SPEED_COMMAND="$(awk -v speed="$SPEED_COMMAND" 'BEGIN { printf "%.3f", speed }')"

if [[ "$RUN_MODE" != "rule" ]]; then
  problem \
    "주행 모드 오류" \
    "이 SLAM-free 통합본은 RULE 모드만 지원합니다." \
    "RUN_MODE를 지정하지 않거나 rule로 지정하세요."
  exit 2
fi
START_WAYPOINT=1
if [[ ! "$START_WAYPOINT" =~ ^[1-6]$ ]]; then
  problem \
    "웨이포인트 입력 오류" \
    "'$START_WAYPOINT'은 존재하지 않는 웨이포인트입니다." \
    "WP 글자를 붙이지 말고 1부터 6 중 숫자 하나만 입력하세요. 예: 1"
  exit 2
fi

if [[ -z "$LOOKAHEAD_DISTANCE" ]]; then
  read -r -p "곡선 Lookahead distance [m, 기본 0.3]: " LOOKAHEAD_DISTANCE
  LOOKAHEAD_DISTANCE="${LOOKAHEAD_DISTANCE:-0.3}"
fi
LOOKAHEAD_DISTANCE="${LOOKAHEAD_DISTANCE/,/.}"
if [[ ! "$LOOKAHEAD_DISTANCE" =~ ^[0-9]+([.][0-9]+)?$ ]] || \
  ! awk -v value="$LOOKAHEAD_DISTANCE" \
    'BEGIN { exit !(value >= 0.1 && value <= 5.0) }'; then
  problem \
    "LD 입력 오류" \
    "'$LOOKAHEAD_DISTANCE'은 사용할 수 없는 Lookahead distance입니다." \
    "0.1부터 5.0 사이 숫자만 입력하세요. 예: 0.7"
  exit 2
fi
LOOKAHEAD_DISTANCE="$(awk -v value="$LOOKAHEAD_DISTANCE" \
  'BEGIN { printf "%.3f", value }')"

if [[ -z "$STANLEY_PERCENT" ]]; then
  read -r -p "곡선 Stanley 비율 [%, 기본 20]: " STANLEY_PERCENT
  STANLEY_PERCENT="${STANLEY_PERCENT:-20}"
fi
STANLEY_PERCENT="${STANLEY_PERCENT/,/.}"
if [[ ! "$STANLEY_PERCENT" =~ ^[0-9]+([.][0-9]+)?$ ]] || \
  ! awk -v value="$STANLEY_PERCENT" \
    'BEGIN { exit !(value >= 0.0 && value <= 100.0) }'; then
  problem \
    "Stanley 비율 입력 오류" \
    "'$STANLEY_PERCENT'은 사용할 수 없는 비율입니다." \
    "0부터 100 사이 퍼센트 숫자만 입력하세요. 예: 20"
  exit 2
fi
STANLEY_PERCENT="$(awk -v value="$STANLEY_PERCENT" \
  'BEGIN { printf "%.3f", value }')"

prompt_float PURE_PURSUIT_CONTROL_X_M \
  "Pure Pursuit 제어점 X [m]" -0.08 -1.0 1.0
prompt_float STANLEY_CONTROL_X_M \
  "Stanley 제어점 X [m]" 0.16 -1.0 1.0
prompt_float STANLEY_GAIN \
  "곡선 Stanley 횡오차 gain" 1.15 0.0 10.0
prompt_float STANLEY_SOFTENING_MPS \
  "곡선 Stanley 저속 완화값 [m/s]" 0.35 0.01 10.0
prompt_float STRAIGHT_STANLEY_PERCENT \
  "직선 Stanley 비율 [%]" 90.0 0.0 100.0
prompt_float STRAIGHT_STANLEY_GAIN \
  "직선 Stanley 횡오차 gain" 0.65 0.0 10.0
prompt_float STRAIGHT_STANLEY_SOFTENING_MPS \
  "직선 Stanley 저속 완화값 [m/s]" 0.65 0.01 10.0
prompt_float OPPOSED_STANLEY_PERCENT \
  "PP와 Stanley 방향 상충 시 Stanley 비율 [%]" 70.0 0.0 100.0
prompt_float CONTROL_LATENCY_PREVIEW_SEC \
  "제어 지연 예측 시간 [s]" 0.30 0.0 2.0

if [[ -z "$LEFT_OFFSET_CM" ]]; then
  read -r -p "좌측 주행 보정 거리 [cm, 기본 9]: " LEFT_OFFSET_CM
  LEFT_OFFSET_CM="${LEFT_OFFSET_CM:-9}"
fi
LEFT_OFFSET_CM="${LEFT_OFFSET_CM/,/.}"
if [[ ! "$LEFT_OFFSET_CM" =~ ^[0-9]+([.][0-9]+)?$ ]] || \
  ! awk -v value="$LEFT_OFFSET_CM" \
    'BEGIN { exit !(value >= 0.0 && value <= 100.0) }'; then
  problem \
    "좌측 보정 입력 오류" \
    "'$LEFT_OFFSET_CM'은 사용할 수 없는 거리입니다." \
    "0부터 100 사이 cm 숫자만 입력하세요. 예: 15"
  exit 2
fi
LEFT_OFFSET_CM="$(awk -v value="$LEFT_OFFSET_CM" \
  'BEGIN { printf "%.1f", value }')"

if awk -v speed="$SPEED_COMMAND" 'BEGIN { exit !(speed > 10.0) }'; then
  echo "[주의] command $SPEED_COMMAND은 기존 실차 시험 상한 10을 초과합니다."
fi
set +u
source /opt/ros/humble/setup.bash
source "$WORKSPACE/install/setup.bash"
set -u

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-7}"
unset ROS_NAMESPACE || true

cleanup() {
  set +e
  if [[ -n "$SENSOR_PID" ]] && kill -0 -- "-$SENSOR_PID" 2>/dev/null; then
    kill -TERM -- "-$SENSOR_PID" 2>/dev/null
    for _ in $(seq 1 30); do
      kill -0 -- "-$SENSOR_PID" 2>/dev/null || break
      sleep 0.1
    done
    if kill -0 -- "-$SENSOR_PID" 2>/dev/null; then
      kill -KILL -- "-$SENSOR_PID" 2>/dev/null
    fi
    wait "$SENSOR_PID" 2>/dev/null || true
  fi
}
trap cleanup EXIT INT TERM

check_device() {
  local label="$1"
  local path="$2"
  local hint="$3"
  if [[ ! -e "$path" ]]; then
    problem \
      "$label 연결 안 됨" \
      "$path 장치를 찾을 수 없습니다." \
      "$hint"
    return 1
  fi
  printf '  [OK] %-12s %s -> %s\n' \
    "$label USB" "$path" "$(readlink -f "$path")"
}

check_duplicate_nodes() {
  local nodes
  local node
  nodes="$(ros2 node list 2>/dev/null || true)"
  for node in /wide_camera /xycar_lidar_node /xycar_vesc_driver; do
    if grep -Fxq "$node" <<<"$nodes"; then
      problem \
        "센서 중복 실행" \
        "$node 노드가 이미 실행 중입니다." \
        "기존 센서 터미널을 Ctrl+C로 종료한 뒤 이 통합 명령만 실행하세요."
      return 1
    fi
  done
}

topic_failure_help() {
  local label="$1"
  local topic="$2"
  case "$label" in
    CAMERA)
      problem \
        "카메라 영상 없음" \
        "$topic 영상이 20초 동안 들어오지 않았습니다." \
        "카메라 USB 연결과 $CAMERA_DEVICE를 확인하세요."
      ;;
    LIDAR)
      problem \
        "LiDAR 스캔 없음" \
        "$topic 데이터가 20초 동안 들어오지 않았습니다." \
        "/dev/ttyLIDAR 연결과 센서 로그의 'Now lidar is scanning'을 확인하세요."
      ;;
    VESC)
      problem \
        "VESC 텔레메트리 없음" \
        "$topic 데이터가 20초 동안 들어오지 않았습니다." \
        "/dev/ttyMOTOR, 모터 배터리 전압, VESC USB 연결을 확인하세요."
      ;;
  esac
  echo "[센서 로그] $SENSOR_LOG" >&2
  echo "[센서 로그 마지막 내용]" >&2
  tail -n 30 "$SENSOR_LOG" >&2 || true
}

wait_for_message() {
  local topic="$1"
  local label="$2"
  local attempt
  printf '  [확인 중] %-12s %s\n' "$label" "$topic"
  for attempt in $(seq 1 20); do
    if ! kill -0 "$SENSOR_PID" 2>/dev/null; then
      problem \
        "센서 실행 종료" \
        "센서 launch가 준비 도중 종료되었습니다." \
        "아래 센서 로그에서 가장 마지막 ERROR를 확인하세요."
      tail -n 40 "$SENSOR_LOG" >&2 || true
      return 1
    fi
    if timeout --signal=INT --kill-after=1s 3s \
      ros2 topic echo "$topic" --once \
      --qos-reliability best_effort >/dev/null 2>&1; then
      printf '  [OK] %-12s %s\n' "$label" "$topic"
      return 0
    fi
  done
  topic_failure_help "$label" "$topic"
  return 1
}

echo
echo "========== USB 연결 확인 =========="
check_duplicate_nodes
check_device CAMERA "$CAMERA_DEVICE" \
  "카메라를 다시 연결하고 ls -l /dev/v4l/by-id/ 를 확인하세요."
check_device LIDAR /dev/ttyLIDAR \
  "LiDAR USB를 다시 연결하고 readlink -f /dev/ttyLIDAR를 확인하세요."
check_device VESC /dev/ttyMOTOR \
  "VESC USB와 모터 배터리를 확인하고 readlink -f /dev/ttyMOTOR를 실행하세요."

echo
echo "========== 센서 시작 =========="
echo "센서 로그: $SENSOR_LOG"
setsid ros2 launch xycar_map_nav real_hybrid_test_sensors.launch.py \
  vesc_drive_enabled:=true >"$SENSOR_LOG" 2>&1 &
SENSOR_PID=$!

wait_for_message /wide_camera_mjpeg/image_raw/compressed CAMERA
wait_for_message /scan LIDAR
wait_for_message /vehicle/vesc_state VESC

echo
echo "========== 모든 센서 정상 =========="
echo "속도 상한: $SPEED_COMMAND | 시작 목표: WP$START_WAYPOINT"
if [[ "$STEERING_ONLY" == "true" ]]; then
  echo "조향 전용: ON | 최종 속도 command는 항상 0.0"
fi
echo "곡선 제어: LD=${LOOKAHEAD_DISTANCE}m | Stanley=${STANLEY_PERCENT}%"
echo "제어점: PP X=${PURE_PURSUIT_CONTROL_X_M}m | Stanley X=${STANLEY_CONTROL_X_M}m"
echo "곡선 Stanley: gain=${STANLEY_GAIN} | soft=${STANLEY_SOFTENING_MPS}m/s"
echo "직선 Stanley: ${STRAIGHT_STANLEY_PERCENT}% | gain=${STRAIGHT_STANLEY_GAIN} | soft=${STRAIGHT_STANLEY_SOFTENING_MPS}m/s"
echo "상충 Stanley: ${OPPOSED_STANLEY_PERCENT}% | 지연 예측=${CONTROL_LATENCY_PREVIEW_SEC}s"
echo "좌측 주행 보정: ${LEFT_OFFSET_CM}cm"
echo "제어기를 준비합니다. 아직 차량은 정지 상태입니다."
echo

export PURE_PURSUIT_CONTROL_X_M STANLEY_CONTROL_X_M
export STANLEY_GAIN STANLEY_SOFTENING_MPS
export STRAIGHT_STANLEY_PERCENT STRAIGHT_STANLEY_GAIN
export STRAIGHT_STANLEY_SOFTENING_MPS OPPOSED_STANLEY_PERCENT
export CONTROL_LATENCY_PREVIEW_SEC

"$WORKSPACE/src/xycar_map_nav/scripts/run_space_hybrid_test.sh" \
  "$SPEED_COMMAND" "$START_WAYPOINT" "$RUN_MODE" "$MODEL_PROFILE" \
  "$LOOKAHEAD_DISTANCE" "$STANLEY_PERCENT" "$LEFT_OFFSET_CM"
