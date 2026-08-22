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
INSTALL_PREFIX="${XYCAR_INSTALL_PREFIX:-$WORKSPACE/install_xycar_only}"
if [[ ! -f "$INSTALL_PREFIX/setup.bash" ]]; then
  echo "[오류] xycar_ws 단독 설치 결과가 없습니다: $INSTALL_PREFIX/setup.bash" >&2
  echo "먼저 다음 명령을 실행하세요:" >&2
  echo "  bash $WORKSPACE/src/xycar_map_nav/scripts/build_xycar_only.sh" >&2
  exit 1
fi
export XYCAR_INSTALL_PREFIX="$INSTALL_PREFIX"

# Keep the real-car one-terminal launcher compatible with the shortcut
# selector exposed by run_space_hybrid_test.sh. Positional driving parameters
# may appear before or after these named options.
SHORTCUT_STRATEGY="${SHORTCUT_STRATEGY:-w1}"
SHORTCUT_YELLOW_COUNT_TARGET="${SHORTCUT_YELLOW_COUNT_TARGET:-2}"
SHORTCUT_YELLOW_COUNT_FORCE_ANGLE="${SHORTCUT_YELLOW_COUNT_FORCE_ANGLE:--42.0}"
SHORTCUT_YELLOW_COUNT_RETURN_SEC="${SHORTCUT_YELLOW_COUNT_RETURN_SEC:-0.7}"
declare -a POSITIONAL_ARGS=()
while (( $# > 0 )); do
  case "$1" in
    --shortcut-mode)
      if (( $# < 2 )); then
        echo "ERROR: --shortcut-mode requires w1 or yellow_count." >&2
        exit 2
      fi
      SHORTCUT_STRATEGY="$2"
      shift 2
      ;;
    --shortcut-mode=*)
      SHORTCUT_STRATEGY="${1#*=}"
      shift
      ;;
    --shortcut-yellow-count)
      if (( $# < 2 )); then
        echo "ERROR: --shortcut-yellow-count requires 1 or 2." >&2
        exit 2
      fi
      SHORTCUT_YELLOW_COUNT_TARGET="$2"
      shift 2
      ;;
    --shortcut-yellow-count=*)
      SHORTCUT_YELLOW_COUNT_TARGET="${1#*=}"
      shift
      ;;
    --shortcut-angle)
      if (( $# < 2 )); then
        echo "ERROR: --shortcut-angle requires a value from -42 to 0." >&2
        exit 2
      fi
      SHORTCUT_YELLOW_COUNT_FORCE_ANGLE="$2"
      shift 2
      ;;
    --shortcut-angle=*)
      SHORTCUT_YELLOW_COUNT_FORCE_ANGLE="${1#*=}"
      shift
      ;;
    --shortcut-return-sec)
      if (( $# < 2 )); then
        echo "ERROR: --shortcut-return-sec requires seconds." >&2
        exit 2
      fi
      SHORTCUT_YELLOW_COUNT_RETURN_SEC="$2"
      shift 2
      ;;
    --shortcut-return-sec=*)
      SHORTCUT_YELLOW_COUNT_RETURN_SEC="${1#*=}"
      shift
      ;;
    --help|-h)
      echo "Usage: $0 [speed] [lookahead] [stanley_percent] [left_offset_cm] [options]"
      echo "  --shortcut-mode w1|yellow_count"
      echo "  --shortcut-yellow-count 1|2     (yellow_count only, default: 2)"
      echo "  --shortcut-angle -42..0          (yellow_count only)"
      echo "  --shortcut-return-sec 0.1..5.0   (yellow_count only)"
      exit 0
      ;;
    --)
      shift
      while (( $# > 0 )); do
        POSITIONAL_ARGS+=("$1")
        shift
      done
      ;;
    -*)
      echo "ERROR: unknown option: $1" >&2
      exit 2
      ;;
    *)
      POSITIONAL_ARGS+=("$1")
      shift
      ;;
  esac
done
set -- "${POSITIONAL_ARGS[@]}"

case "$SHORTCUT_STRATEGY" in
  w1|yellow_count) ;;
  *)
    echo "ERROR: --shortcut-mode must be w1 or yellow_count." >&2
    exit 2
    ;;
esac
case "$SHORTCUT_YELLOW_COUNT_TARGET" in
  1|2) ;;
  *)
    echo "ERROR: --shortcut-yellow-count must be 1 or 2." >&2
    exit 2
    ;;
esac
if [[ ! "$SHORTCUT_YELLOW_COUNT_FORCE_ANGLE" =~ ^-?[0-9]+([.][0-9]+)?$ ]] || \
  ! awk -v value="$SHORTCUT_YELLOW_COUNT_FORCE_ANGLE" \
    'BEGIN { exit !(value >= -42.0 && value <= 0.0) }'; then
  echo "ERROR: --shortcut-angle must be from -42 to 0." >&2
  exit 2
fi
if [[ ! "$SHORTCUT_YELLOW_COUNT_RETURN_SEC" =~ ^[0-9]+([.][0-9]+)?$ ]] || \
  ! awk -v value="$SHORTCUT_YELLOW_COUNT_RETURN_SEC" \
    'BEGIN { exit !(value >= 0.1 && value <= 5.0) }'; then
  echo "ERROR: --shortcut-return-sec must be from 0.1 to 5.0 seconds." >&2
  exit 2
fi

CAMERA_DEVICE="/dev/v4l/by-id/usb-HD_USB_Camera_HD_USB_Camera-video-index0"
SPEED_COMMAND="${1:-${SPEED_COMMAND:-20.0}}"
OVERALL_SPEED_LIMIT_COMMAND="${OVERALL_SPEED_LIMIT_COMMAND:-15.0}"
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
YELLOW_CURVE_REVERSAL_PREVIEW_FAR_X_M="${YELLOW_CURVE_REVERSAL_PREVIEW_FAR_X_M:-1.00}"
STRAIGHT_PATH_CURVATURE_THRESHOLD="${STRAIGHT_PATH_CURVATURE_THRESHOLD:-0.24}"
STEERING_CURRENT_WEIGHT="${STEERING_CURRENT_WEIGHT:-0.35}"
STEERING_CURVE_CURRENT_WEIGHT="${STEERING_CURVE_CURRENT_WEIGHT:-0.80}"
STEERING_RATE_LIMIT_CMD_PER_SEC="${STEERING_RATE_LIMIT_CMD_PER_SEC:-180.0}"
STEERING_CURVE_RATE_LIMIT_CMD_PER_SEC="${STEERING_CURVE_RATE_LIMIT_CMD_PER_SEC:-300.0}"
STEERING_LEAD_TIME_SEC="${STEERING_LEAD_TIME_SEC:-0.08}"
STEERING_MAX_LEAD_COMMAND="${STEERING_MAX_LEAD_COMMAND:-6.0}"
VEHICLE_LEFT_OFFSET_M="${VEHICLE_LEFT_OFFSET_M:-}"
VEHICLE_RIGHT_OFFSET_M="${VEHICLE_RIGHT_OFFSET_M:-}"
CURVE_STEERING_MULTIPLIER_ENABLED="${CURVE_STEERING_MULTIPLIER_ENABLED:-}"
CURVE_STEERING_MULTIPLIER_ACTIVATION_COMMAND="${CURVE_STEERING_MULTIPLIER_ACTIVATION_COMMAND:-}"
CURVE_STEERING_MULTIPLIER="${CURVE_STEERING_MULTIPLIER:-}"
ADAPTIVE_STEERING_SPEED_ENABLED="${ADAPTIVE_STEERING_SPEED_ENABLED:-}"
STEERING_TURN_SPEED_COMMAND="${STEERING_TURN_SPEED_COMMAND:-}"
STEERING_SLOWDOWN_START_ANGLE="${STEERING_SLOWDOWN_START_ANGLE:-}"
STEERING_FULL_SLOWDOWN_ANGLE="${STEERING_FULL_SLOWDOWN_ANGLE:-}"
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

prompt_bool() {
  local variable_name="$1"
  local prompt="$2"
  local default_value="$3"
  local value="${!variable_name:-}"
  if [[ -z "$value" ]]; then
    local prompt_hint="[Y/n]"
    if [[ "${default_value,,}" =~ ^(n|no|false|0)$ ]]; then
      prompt_hint="[y/N]"
    fi
    read -r -p "$prompt $prompt_hint: " value
    value="${value:-$default_value}"
  fi
  case "${value,,}" in
    y|yes|true|1) printf -v "$variable_name" '%s' true ;;
    n|no|false|0) printf -v "$variable_name" '%s' false ;;
    *)
      problem \
        "ON/OFF 입력 오류" \
        "'$value'은 $prompt 응답으로 사용할 수 없습니다." \
        "켜려면 y, 끄려면 n을 입력하세요."
      exit 2
      ;;
  esac
}

# Ask for the curve/S-bend gain before every other interactive driving value.
# An environment override still skips the prompt for scripted repeatability.
prompt_float CURVE_STEERING_MULTIPLIER \
  "S자/곡선 조향 배수 (1.0=증폭 없음)" 1.0 0.0 3.0
prompt_bool CURVE_STEERING_MULTIPLIER_ENABLED \
  "곡선에서 큰 조향 명령 배수 적용" true
if [[ "$CURVE_STEERING_MULTIPLIER_ENABLED" == "true" ]]; then
  prompt_float CURVE_STEERING_MULTIPLIER_ACTIVATION_COMMAND \
    "배수를 시작할 절대 조향 명령" 20.0 0.0 42.0
else
  CURVE_STEERING_MULTIPLIER_ACTIVATION_COMMAND="${CURVE_STEERING_MULTIPLIER_ACTIVATION_COMMAND:-20.0}"
fi

if [[ "$STEERING_ONLY" == "true" ]]; then
  SPEED_COMMAND=0.0
elif [[ -z "$SPEED_COMMAND" ]]; then
  read -r -p "주행 속도 command [3.0-30.0, 기본 20.0]: " SPEED_COMMAND
  SPEED_COMMAND="${SPEED_COMMAND:-20.0}"
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
OVERALL_SPEED_LIMIT_COMMAND="${OVERALL_SPEED_LIMIT_COMMAND/,/.}"
if [[ ! "$OVERALL_SPEED_LIMIT_COMMAND" =~ ^[0-9]+([.][0-9]+)?$ ]] || \
  ! awk -v speed="$OVERALL_SPEED_LIMIT_COMMAND" \
    'BEGIN { exit !(speed >= 0.0 && speed <= 30.0) }'; then
  problem \
    "전체 속도 상한 입력 오류" \
    "'$OVERALL_SPEED_LIMIT_COMMAND'은 사용할 수 없는 속도 상한입니다." \
    "0.0부터 30.0 사이 숫자를 사용하세요. 예: 15"
  exit 2
fi
OVERALL_SPEED_LIMIT_COMMAND="$(awk -v speed="$OVERALL_SPEED_LIMIT_COMMAND" \
  'BEGIN { printf "%.3f", speed }')"

if [[ "$STEERING_ONLY" == "true" ]]; then
  CURVATURE_SPEED_CONTROL_ENABLED=false
  CURVE_SPEED_COMMAND=0.000
  DEGRADED_PATH_SPEED_COMMAND=0.000
else
  prompt_bool CURVATURE_SPEED_CONTROL_ENABLED \
    "직선/곡선 속도 분리 사용" true
  if [[ "$CURVATURE_SPEED_CONTROL_ENABLED" == "true" ]]; then
    curve_default="$(awk -v speed="$SPEED_COMMAND" \
      'BEGIN { printf "%.3f", (speed < 12.0 ? speed : 12.0) }')"
    prompt_float CURVE_SPEED_COMMAND \
      "곡선 확정 시 속도 command" "$curve_default" 3.0 "$SPEED_COMMAND"
    degraded_default="$(awk -v curve="$CURVE_SPEED_COMMAND" \
      'BEGIN { printf "%.3f", (curve < 11.0 ? curve : 11.0) }')"
    prompt_float DEGRADED_PATH_SPEED_COMMAND \
      "짧거나 기억된 경로의 속도 command" "$degraded_default" 3.0 \
      "$CURVE_SPEED_COMMAND"
  else
    CURVE_SPEED_COMMAND="$SPEED_COMMAND"
    DEGRADED_PATH_SPEED_COMMAND="$SPEED_COMMAND"
  fi
fi

prompt_bool ADAPTIVE_STEERING_SPEED_ENABLED \
  "조향각 기반 속도 가감속 사용" true
if [[ "$ADAPTIVE_STEERING_SPEED_ENABLED" == "true" ]]; then
  prompt_float STEERING_SLOWDOWN_START_ANGLE \
    "속도 감속을 시작할 절대 조향각" 18.0 0.0 42.0
  prompt_float STEERING_FULL_SLOWDOWN_ANGLE \
    "최저속도에 도달할 절대 조향각" 42.0 0.0 42.0
  prompt_float STEERING_TURN_SPEED_COMMAND \
    "최대 조향 시 속도 command" 12.0 0.0 30.0
  if ! awk \
    -v start="$STEERING_SLOWDOWN_START_ANGLE" \
    -v full="$STEERING_FULL_SLOWDOWN_ANGLE" \
    'BEGIN { exit !(full >= start) }'; then
    problem \
      "조향 감속 각도 오류" \
      "최저속도 도달각이 감속 시작각보다 작습니다." \
      "예: 시작각 20, 도달각 42"
    exit 2
  fi
else
  STEERING_SLOWDOWN_START_ANGLE=18.000
  STEERING_FULL_SLOWDOWN_ANGLE=42.000
  STEERING_TURN_SPEED_COMMAND=12.000
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
  "곡선 Stanley 횡오차 gain" 1.30 0.0 10.0
prompt_float STANLEY_SOFTENING_MPS \
  "곡선 Stanley 저속 완화값 [m/s]" 0.35 0.01 10.0
prompt_float STRAIGHT_STANLEY_PERCENT \
  "직선 Stanley 비율 [%]" 90.0 0.0 100.0
prompt_float STRAIGHT_STANLEY_GAIN \
  "직선 Stanley 횡오차 gain" 0.50 0.0 10.0
prompt_float STRAIGHT_STANLEY_SOFTENING_MPS \
  "직선 Stanley 저속 완화값 [m/s]" 0.65 0.01 10.0
prompt_float OPPOSED_STANLEY_PERCENT \
  "PP와 Stanley 방향 상충 시 Stanley 비율 [%]" 70.0 0.0 100.0
prompt_float CONTROL_LATENCY_PREVIEW_SEC \
  "직선 제어 지연 예측 시간 [s]" 0.20 0.0 2.0
prompt_float CURVE_CONTROL_LATENCY_PREVIEW_SEC \
  "곡선 제어 지연 예측 시간 [s]" 0.35 0.0 2.0
prompt_float CURVE_CONTROL_LATENCY_MINIMUM_HOLD_SEC \
  "곡선 지연 예측 최소 유지 시간 [s]" 0.50 0.0 5.0
prompt_float YELLOW_CURVE_REVERSAL_PREVIEW_FAR_X_M \
  "S자 반대 곡선 미리보기 거리 [m]" 1.00 0.50 2.50
prompt_float STRAIGHT_PATH_CURVATURE_THRESHOLD \
  "직선/곡선 곡률 기준 [rad/m]" 0.24 0.0 5.0
prompt_float STEERING_CURRENT_WEIGHT \
  "직선 조향 현재값 비율" 0.35 0.0 1.0
prompt_float STEERING_CURVE_CURRENT_WEIGHT \
  "곡선 조향 현재값 비율" 0.80 0.0 1.0
prompt_float STEERING_RATE_LIMIT_CMD_PER_SEC \
  "직선 조향 변화율 [command/s]" 180.0 0.0 1000.0
prompt_float STEERING_CURVE_RATE_LIMIT_CMD_PER_SEC \
  "곡선 조향 변화율 [command/s]" 300.0 0.0 1000.0
prompt_float STEERING_LEAD_TIME_SEC \
  "조향 lead time [s]" 0.08 0.0 1.0
prompt_float STEERING_MAX_LEAD_COMMAND \
  "최대 조향 lead command" 6.0 0.0 42.0
prompt_float VEHICLE_LEFT_OFFSET_M \
  "오른쪽 장애물 감지 시 왼쪽 회피 이동량 [m]" 0.13 0.0 1.5
prompt_float VEHICLE_RIGHT_OFFSET_M \
  "왼쪽 장애물 감지 시 오른쪽 회피 이동량 [m]" 0.13 0.0 1.5

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
STRAIGHT_RIGHT_OFFSET_CM="${STRAIGHT_RIGHT_OFFSET_CM/,/.}"
if [[ ! "$STRAIGHT_RIGHT_OFFSET_CM" =~ ^[0-9]+([.][0-9]+)?$ ]] || \
  ! awk -v value="$STRAIGHT_RIGHT_OFFSET_CM" \
    'BEGIN { exit !(value >= 0.0 && value <= 20.0) }'; then
  problem \
    "직선 우측 보정값 오류" \
    "'$STRAIGHT_RIGHT_OFFSET_CM'은 사용할 수 없는 거리입니다." \
    "0부터 20 사이 cm 숫자를 사용하세요."
  exit 2
fi
STRAIGHT_RIGHT_OFFSET_CM="$(awk -v value="$STRAIGHT_RIGHT_OFFSET_CM" \
  'BEGIN { printf "%.1f", value }')"

if awk -v speed="$SPEED_COMMAND" 'BEGIN { exit !(speed > 10.0) }'; then
  echo "[주의] command $SPEED_COMMAND은 기존 실차 시험 상한 10을 초과합니다."
fi
set +u
# Do not inherit another colcon workspace (for example ~/xycar_ws) from
# .bashrc or from a previously sourced terminal.  install_xycar_only was built
# with /opt/ros/humble as its only underlay.
unset AMENT_PREFIX_PATH COLCON_PREFIX_PATH CMAKE_PREFIX_PATH
unset PYTHONPATH LD_LIBRARY_PATH PKG_CONFIG_PATH ROS_PACKAGE_PATH
unset _colcon_cd_root
clean_workspace_path=""
IFS=: read -r -a path_entries <<< "${PATH:-}"
for path_entry in "${path_entries[@]}"; do
  case "$path_entry" in
    /home/xytron/*/install/*|/home/xytron/*/install_*/*) continue ;;
  esac
  clean_workspace_path="${clean_workspace_path:+$clean_workspace_path:}$path_entry"
done
export PATH="$clean_workspace_path"
unset clean_workspace_path path_entries path_entry
source /opt/ros/humble/setup.bash
source "$INSTALL_PREFIX/setup.bash"
set -u

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-7}"
unset ROS_NAMESPACE || true

for required_package in \
  xycar_map_nav xycar_rule_drive lane_seg_control my_rule wide_camera \
  xycar_vesc_driver; do
  if ! package_prefix="$(ros2 pkg prefix "$required_package" 2>/dev/null)" || \
    [[ "$package_prefix" != "$INSTALL_PREFIX/"* ]]; then
    echo "[오류] $required_package 패키지가 xycar_ws 단독 빌드에서 해석되지 않습니다." >&2
    echo "현재 경로: ${package_prefix:-찾을 수 없음}" >&2
    echo "다시 빌드: bash $WORKSPACE/src/xycar_map_nav/scripts/build_xycar_only.sh" >&2
    exit 1
  fi
done
echo "xycar_ws 단독 실행 환경: $INSTALL_PREFIX"

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
  local field="${3:-}"
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
    local echo_command=(
      ros2 topic echo "$topic" --once
      --qos-reliability best_effort
    )
    if [[ -n "$field" ]]; then
      echo_command+=(--field "$field")
    fi
    if timeout --signal=INT --kill-after=1s 3s \
      "${echo_command[@]}" >/dev/null 2>&1; then
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

wait_for_message /wide_camera_mjpeg/image_raw/compressed CAMERA header.stamp
wait_for_message /scan LIDAR
wait_for_message /vehicle/vesc_state VESC

echo
echo "========== 모든 센서 정상 =========="
echo "전체 최종 속도 상한: $OVERALL_SPEED_LIMIT_COMMAND | 기본 주행: RULE"
if [[ "$CURVATURE_SPEED_CONTROL_ENABLED" == "true" ]]; then
  echo "경로별 속도: 직선 $SPEED_COMMAND | 곡선 $CURVE_SPEED_COMMAND | 짧음/기억 $DEGRADED_PATH_SPEED_COMMAND"
  echo "곡선 속도 전환: 진입 ${CURVE_SPEED_CONFIRMATION_FRAMES}프레임 | 복귀 ${CURVE_SPEED_RELEASE_FRAMES}프레임 | 복귀 기준 ${CURVE_SPEED_EXIT_THRESHOLD_PER_M}rad/m"
else
  echo "경로별 속도: OFF"
fi
if [[ "$ADAPTIVE_STEERING_SPEED_ENABLED" == "true" ]]; then
  echo "조향 감속: ${STEERING_SLOWDOWN_START_ANGLE}도부터 ${STEERING_FULL_SLOWDOWN_ANGLE}도까지 command ${STEERING_TURN_SPEED_COMMAND}으로 감속"
else
  echo "조향 감속: OFF"
fi
if [[ "$STEERING_ONLY" == "true" ]]; then
  echo "조향 전용: ON | 최종 속도 command는 항상 0.0"
fi
echo "곡선 제어: LD=${LOOKAHEAD_DISTANCE}m | Stanley=${STANLEY_PERCENT}%"
echo "제어점: PP X=${PURE_PURSUIT_CONTROL_X_M}m | Stanley X=${STANLEY_CONTROL_X_M}m"
echo "곡선 Stanley: gain=${STANLEY_GAIN} | soft=${STANLEY_SOFTENING_MPS}m/s"
echo "직선 Stanley: ${STRAIGHT_STANLEY_PERCENT}% | gain=${STRAIGHT_STANLEY_GAIN} | soft=${STRAIGHT_STANLEY_SOFTENING_MPS}m/s"
echo "상충 Stanley: ${OPPOSED_STANLEY_PERCENT}% | 지연 예측=직선 ${CONTROL_LATENCY_PREVIEW_SEC}s/곡선 ${CURVE_CONTROL_LATENCY_PREVIEW_SEC}s | 곡선 최소 유지=${CURVE_CONTROL_LATENCY_MINIMUM_HOLD_SEC}s"
echo "직선/곡선 기준: ${STRAIGHT_PATH_CURVATURE_THRESHOLD}rad/m"
echo "조향 smoothing: 직선 ${STEERING_CURRENT_WEIGHT}/${STEERING_RATE_LIMIT_CMD_PER_SEC}, 곡선 ${STEERING_CURVE_CURRENT_WEIGHT}/${STEERING_CURVE_RATE_LIMIT_CMD_PER_SEC}"
echo "S자 반대 조향 미리보기 거리: ${YELLOW_CURVE_REVERSAL_PREVIEW_FAR_X_M}m"
echo "조향 lead: ${STEERING_LEAD_TIME_SEC}s | 최대 ${STEERING_MAX_LEAD_COMMAND} command"
echo "곡선 조향 배수: ${CURVE_STEERING_MULTIPLIER_ENABLED} | ${CURVE_STEERING_MULTIPLIER_ACTIVATION_COMMAND} 이상 x${CURVE_STEERING_MULTIPLIER} | 최대 +/-42"
echo "좌측 주행 보정: ${LEFT_OFFSET_CM}cm"
echo "직선 전용 우측 보정: ${STRAIGHT_RIGHT_OFFSET_CM}cm"
echo "차량 회피 이동량: 오른쪽 장애물 -> 왼쪽 ${VEHICLE_LEFT_OFFSET_M}m | 왼쪽 장애물 -> 오른쪽 ${VEHICLE_RIGHT_OFFSET_M}m"
echo "제어기를 준비합니다. 아직 차량은 정지 상태입니다."
echo

export PURE_PURSUIT_CONTROL_X_M STANLEY_CONTROL_X_M
export STANLEY_GAIN STANLEY_SOFTENING_MPS
export STRAIGHT_STANLEY_PERCENT STRAIGHT_STANLEY_GAIN
export STRAIGHT_STANLEY_SOFTENING_MPS OPPOSED_STANLEY_PERCENT
export CONTROL_LATENCY_PREVIEW_SEC CURVE_CONTROL_LATENCY_PREVIEW_SEC
export CURVE_CONTROL_LATENCY_MINIMUM_HOLD_SEC
export YELLOW_CURVE_REVERSAL_PREVIEW_FAR_X_M
export STRAIGHT_PATH_CURVATURE_THRESHOLD
export STEERING_CURRENT_WEIGHT STEERING_CURVE_CURRENT_WEIGHT
export STEERING_RATE_LIMIT_CMD_PER_SEC
export STEERING_CURVE_RATE_LIMIT_CMD_PER_SEC
export STEERING_LEAD_TIME_SEC STEERING_MAX_LEAD_COMMAND
export VEHICLE_LEFT_OFFSET_M VEHICLE_RIGHT_OFFSET_M
export CURVE_STEERING_MULTIPLIER_ENABLED
export CURVE_STEERING_MULTIPLIER_ACTIVATION_COMMAND
export CURVE_STEERING_MULTIPLIER
export CURVATURE_SPEED_CONTROL_ENABLED CURVE_SPEED_COMMAND
export DEGRADED_PATH_SPEED_COMMAND CURVE_SPEED_EXIT_THRESHOLD_PER_M
export OVERALL_SPEED_LIMIT_COMMAND
export CURVE_SPEED_CONFIRMATION_FRAMES CURVE_SPEED_RELEASE_FRAMES
export DEGRADED_PATH_MINIMUM_SPAN_M
export STRAIGHT_RIGHT_OFFSET_CM
export ADAPTIVE_STEERING_SPEED_ENABLED STEERING_TURN_SPEED_COMMAND
export STEERING_SLOWDOWN_START_ANGLE STEERING_FULL_SLOWDOWN_ANGLE
export SHORTCUT_STRATEGY SHORTCUT_YELLOW_COUNT_FORCE_ANGLE
export SHORTCUT_YELLOW_COUNT_RETURN_SEC SHORTCUT_YELLOW_COUNT_TARGET

"$WORKSPACE/src/xycar_map_nav/scripts/run_space_hybrid_test.sh" \
  "$SPEED_COMMAND" "$LOOKAHEAD_DISTANCE" "$STANLEY_PERCENT" \
  "$LEFT_OFFSET_CM"
