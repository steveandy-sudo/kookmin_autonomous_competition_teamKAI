#!/usr/bin/env bash
set -e
set +u

WS=/home/xytron/kookmin_ty/integrated_rule_drive_latest/xycar_ws
cd "$WS"
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=7
unset ROS_NAMESPACE

read_number() {
  local name="$1"
  local prompt="$2"
  local default="$3"
  local minimum="$4"
  local maximum="$5"
  local value
  while true; do
    read -r -p "$prompt [$default]: " value
    value="${value:-$default}"
    if awk -v value="$value" -v min="$minimum" -v max="$maximum" \
      'BEGIN { exit !(value ~ /^-?[0-9]+([.][0-9]+)?$/ && value >= min && value <= max) }'; then
      printf -v "$name" '%s' "$value"
      return
    fi
    echo "입력 오류: $minimum 이상 $maximum 이하의 숫자를 입력하세요."
  done
}

read_yes_no() {
  local name="$1"
  local prompt="$2"
  local default="$3"
  local value
  while true; do
    read -r -p "$prompt [$default]: " value
    value="${value:-$default}"
    case "${value,,}" in
      y|yes) printf -v "$name" '%s' true; return ;;
      n|no) printf -v "$name" '%s' false; return ;;
      *) echo "입력 오류: y 또는 n을 입력하세요." ;;
    esac
  done
}

echo "============================================================"
echo " 직접 BEV 차선 주행 설정"
echo " 카메라 -> 보정 -> YOLO -> BEV -> PP+Stanley -> VESC"
echo "============================================================"

while true; do
  read -r -p "차선 모델 [1=best_512, 2=YOLO11n_256] [1]: " MODEL_CHOICE
  MODEL_CHOICE="${MODEL_CHOICE:-1}"
  case "$MODEL_CHOICE" in
    1)
      MODEL="$(ros2 pkg prefix --share lane_seg_control)/models/best_512.onnx"
      IMAGE_SIZE=512
      MODEL_NAME=best_512
      break
      ;;
    2)
      MODEL="$(ros2 pkg prefix --share xycar_perception)/models/kookmin_lane_yolo11n_256.onnx"
      IMAGE_SIZE=256
      MODEL_NAME=YOLO11n_256
      break
      ;;
    *) echo "입력 오류: 1 또는 2를 입력하세요." ;;
  esac
done

read_number SPEED "목표 속도 command (0이면 조향만)" 3.0 0.0 30.0
read_number LOOKAHEAD "Pure Pursuit lookahead LD [m]" 0.30 0.10 1.50
read_number PP_PERCENT "Pure Pursuit 비율 [%]" 80 0 100
read_number STANLEY_GAIN "곡선 Stanley gain" 1.20 0.0 5.0
read_number LEFT_CM "좌측 목표 보정 [cm]" 12.0 -50.0 50.0
read_number MAX_STEER "좌우 최대 조향 command" 42.0 1.0 42.0
read_number COMMAND_HZ "제어 명령 발행 Hz" 10.0 1.0 30.0
read_number CURVE_FAR "곡선 판단 최대 전방 거리 [m]" 0.70 0.20 1.50
read_number CURVE_THRESHOLD "직선/곡선 곡률 기준 [rad/m]" 0.16 0.01 2.0
read_yes_no CURVE_MULTIPLIER_ENABLED "곡선 조향 배수 사용 [y/n]" n

CURVE_MULTIPLIER=1.0
CURVE_MULTIPLIER_ACTIVATION=20.0
if [[ "$CURVE_MULTIPLIER_ENABLED" == true ]]; then
  read_number CURVE_MULTIPLIER_ACTIVATION \
    "조향 배수 적용 시작 command" 20.0 0.0 42.0
  read_number CURVE_MULTIPLIER "곡선 조향 배수" 1.20 1.0 2.0
fi

read_yes_no DRIVE_ENABLED "실제 모터 명령 발행 [y/n]" n
read_yes_no START_VESC "VESC 드라이버도 시작 [y/n]" y
read_yes_no START_CAMERA "카메라도 시작 [y/n]" y
read_yes_no ACCELERATION_SLEW "모터 가속 완화 사용 [y/n]" y

PP_WEIGHT=$(awk -v value="$PP_PERCENT" 'BEGIN { printf "%.4f", value / 100.0 }')
LEFT_OFFSET=$(awk -v value="$LEFT_CM" 'BEGIN { printf "%.4f", value / 100.0 }')
ANGLE_MIN=$(awk -v value="$MAX_STEER" 'BEGIN { printf "%.4f", -value }')
STEERING_ONLY=false
if awk -v value="$SPEED" 'BEGIN { exit !(value == 0.0) }'; then
  STEERING_ONLY=true
fi

echo
echo "================ 실행 설정 ================"
echo "모델: $MODEL_NAME ($IMAGE_SIZE)"
echo "속도: $SPEED | 실제 발행: $DRIVE_ENABLED"
echo "LD: ${LOOKAHEAD}m | PP: ${PP_PERCENT}% | Stanley: $STANLEY_GAIN"
echo "좌측 보정: ${LEFT_CM}cm | 조향: ${ANGLE_MIN}..${MAX_STEER}"
echo "곡선 판단: 0.16..${CURVE_FAR}m, 기준=${CURVE_THRESHOLD}rad/m"
echo "곡선 배수: $CURVE_MULTIPLIER_ENABLED x$CURVE_MULTIPLIER"
echo "제어: ${COMMAND_HZ}Hz | VESC: $START_VESC | 카메라: $START_CAMERA"
echo "============================================"
read -r -p "ENTER를 누르면 시작합니다. 취소는 Ctrl+C: "

exec ros2 launch lane_seg_control direct_bev_stanley_pursuit.launch.py \
  start_vesc:="$START_VESC" \
  start_camera:="$START_CAMERA" \
  acceleration_slew_enabled:="$ACCELERATION_SLEW" \
  model_path:="$MODEL" \
  image_size:="$IMAGE_SIZE" \
  drive_enabled:="$DRIVE_ENABLED" \
  steering_only:="$STEERING_ONLY" \
  cruise_speed_command:="$SPEED" \
  minimum_speed_command:="$SPEED" \
  command_rate_hz:="$COMMAND_HZ" \
  lookahead_distance_m:="$LOOKAHEAD" \
  pure_pursuit_weight:="$PP_WEIGHT" \
  stanley_gain:="$STANLEY_GAIN" \
  target_left_offset_m:="$LEFT_OFFSET" \
  angle_command_min:="$ANGLE_MIN" \
  angle_command_max:="$MAX_STEER" \
  curve_detection_far_x_m:="$CURVE_FAR" \
  straight_path_curvature_threshold:="$CURVE_THRESHOLD" \
  curve_steering_multiplier_enabled:="$CURVE_MULTIPLIER_ENABLED" \
  curve_steering_multiplier_activation_command:="$CURVE_MULTIPLIER_ACTIVATION" \
  curve_steering_multiplier:="$CURVE_MULTIPLIER"
