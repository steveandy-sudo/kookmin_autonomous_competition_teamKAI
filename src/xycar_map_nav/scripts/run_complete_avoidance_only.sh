#!/usr/bin/env bash

set -eo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "$(readlink -f "${BASH_SOURCE[0]}")")" && pwd)"
WORKSPACE="$(cd -- "$SCRIPT_DIR/../../.." && pwd)"
export XYCAR_WS="$WORKSPACE"
CONE_AS_VEHICLE_OBSTACLE="${XYCAR_CONE_AS_VEHICLE_OBSTACLE:-false}"
AVOIDANCE_IMMEDIATE_DEFAULT="${XYCAR_AVOIDANCE_IMMEDIATE_DEFAULT:-true}"
STEERING_ONLY="${XYCAR_STEERING_ONLY:-false}"

prompt_float() {
  local variable_name="$1"
  local label="$2"
  local default_value="$3"
  local minimum="$4"
  local maximum="$5"
  local value

  while true; do
    read -r -p "$label [${minimum}-${maximum}, 기본 ${default_value}]: " value
    value="${value:-$default_value}"
    value="${value/,/.}"
    if [[ "$value" =~ ^-?[0-9]+([.][0-9]+)?$ ]] && \
      awk -v value="$value" -v minimum="$minimum" -v maximum="$maximum" \
        'BEGIN { exit !(value >= minimum && value <= maximum) }'; then
      printf -v "$variable_name" '%s' "$value"
      return
    fi
    echo "[입력 오류] ${minimum}부터 ${maximum} 사이 숫자를 입력하세요."
  done
}

prompt_integer() {
  local variable_name="$1"
  local label="$2"
  local default_value="$3"
  local minimum="$4"
  local maximum="$5"
  local value

  while true; do
    read -r -p "$label [${minimum}-${maximum}, 기본 ${default_value}]: " value
    value="${value:-$default_value}"
    if [[ "$value" =~ ^[0-9]+$ ]] && \
      (( value >= minimum && value <= maximum )); then
      printf -v "$variable_name" '%s' "$value"
      return
    fi
    echo "[입력 오류] ${minimum}부터 ${maximum} 사이 정수를 입력하세요."
  done
}

prompt_bool() {
  local variable_name="$1"
  local label="$2"
  local default_value="$3"
  local answer

  while true; do
    if [[ "$default_value" == "true" ]]; then
      read -r -p "$label [Y/n]: " answer
      answer="${answer:-y}"
    else
      read -r -p "$label [y/N]: " answer
      answer="${answer:-n}"
    fi
    case "${answer,,}" in
      y|yes)
        printf -v "$variable_name" '%s' true
        return
        ;;
      n|no)
        printf -v "$variable_name" '%s' false
        return
        ;;
      *) echo "[입력 오류] y 또는 n을 입력하세요." ;;
    esac
  done
}

echo
if [[ "$CONE_AS_VEHICLE_OBSTACLE" == "true" ]]; then
  echo "========== 라바콘 차량 대용 회피 시험 설정 =========="
  echo "라바콘을 차량 장애물처럼 추적하며 콘 슬라럼은 실행하지 않습니다."
  echo "주행 우선순위: 라바콘 YOLO+LiDAR 회피 > RULE"
else
  echo "========== 회피 전용 실차 시험 설정 =========="
  echo "차선 RULE은 기본 경로만 만들고 콘·모델 전환은 사용하지 않습니다."
  echo "주행 우선순위: YOLO+LiDAR 차량 회피 > RULE"
fi
echo

if [[ "$STEERING_ONLY" == "true" ]]; then
  SPEED_COMMAND=0.0
  echo "조향 전용 시험: 속도 command는 0.0으로 고정됩니다."
else
  prompt_float SPEED_COMMAND "기본 주행 속도 command" 16.0 3.0 30.0
fi
prompt_float LOOKAHEAD_DISTANCE "RULE Lookahead [m]" 0.30 0.10 5.0
prompt_float STANLEY_PERCENT "RULE Stanley 비율 [%]" 20.0 0.0 100.0
prompt_float LEFT_OFFSET_CM "기본 좌측 보정 [cm]" 12.0 0.0 100.0

echo
echo "========== 회피 핵심 파라미터 =========="
prompt_float VEHICLE_YOLO_MIN_CONFIDENCE "차량 YOLO 최소 confidence" 0.45 0.0 1.0
if [[ "$CONE_AS_VEHICLE_OBSTACLE" == "true" ]]; then
  prompt_float CONE_AS_VEHICLE_MIN_CONFIDENCE \
    "차량 대신 놓은 라바콘 YOLO 최소 confidence" 0.50 0.0 1.0
else
  CONE_AS_VEHICLE_MIN_CONFIDENCE=0.50
fi
prompt_integer VEHICLE_YOLO_REQUIRED_FRAMES "YOLO 연속 확인 프레임" 1 1 30
VEHICLE_PREFERRED_SIDE_REQUIRED_FRAMES=1
prompt_float VEHICLE_YOLO_TIMEOUT_SEC "YOLO 검출 유지 시간 [s]" 0.50 0.05 10.0
prompt_bool VEHICLE_AVOIDANCE_IMMEDIATE_ON_YOLO \
  "YOLO 차량을 중앙선 좌우로 판단하면 LiDAR 승인 없이 즉시 회피" \
  "$AVOIDANCE_IMMEDIATE_DEFAULT"
prompt_float VEHICLE_AVOIDANCE_ENTRY_DISTANCE_M \
  "즉시 회피를 끈 경우 회피 진입 거리 [m]" 1.20 0.20 5.0
prompt_float VEHICLE_MINIMUM_SIDE_CLEARANCE_M \
  "회피할 쪽의 최소 빈 공간 [m]" 0.70 0.10 3.0
prompt_float VEHICLE_LEFT_OFFSET_M "왼쪽 회피 이동량 [m]" 0.20 0.0 1.5
prompt_float VEHICLE_RIGHT_OFFSET_M "오른쪽 회피 이동량 [m]" 0.20 0.0 1.5
prompt_float VEHICLE_OFFSET_RATE_MPS "횡이동 변화율 [m/s]" 0.65 0.01 3.0
prompt_float VEHICLE_AVOIDANCE_SPEED_LIMIT_COMMAND \
  "회피 중 속도 상한 command" 20.0 0.0 30.0
prompt_float VEHICLE_MINIMUM_AVOID_SEC "최소 회피 유지 시간 [s]" 0.50 0.0 10.0
prompt_float VEHICLE_CLEAR_HOLD_SEC "장애물 소실 확인 시간 [s]" 0.50 0.0 10.0
prompt_float VEHICLE_RETURN_HOLD_SEC "중앙 복귀 최소 시간 [s]" 0.30 0.0 10.0
prompt_float VEHICLE_RETURN_DEADBAND_M "중앙 복귀 완료 오차 [m]" 0.02 0.0 0.5
prompt_float LIDAR_OBSTACLE_DETECT_DISTANCE_M \
  "LiDAR 장애물 탐색 거리 [m]" 1.50 0.20 10.0
prompt_float LIDAR_OBSTACLE_PATH_HALF_WIDTH_M \
  "기본 경로 주변 LiDAR 탐색 반폭 [m]" 0.18 0.05 2.0

prompt_bool EDIT_ADVANCED "고급 카메라-LiDAR 군집 파라미터도 조정" false
if [[ "$EDIT_ADVANCED" == "true" ]]; then
  echo
  echo "========== 고급 회피 파라미터 =========="
  prompt_float VEHICLE_CAMERA_LIDAR_HFOV_DEG "카메라 수평 화각 [deg]" 60.0 10.0 180.0
  prompt_float VEHICLE_CAMERA_LIDAR_PADDING_DEG "LiDAR 연관 각도 여유 [deg]" 3.0 0.0 30.0
  prompt_integer VEHICLE_LIDAR_MIN_POINTS "차량 연관 최소 LiDAR 점 수" 2 1 100
  prompt_float VEHICLE_LIDAR_SECTOR_MEMORY_SEC "차량 LiDAR 섹터 유지 [s]" 0.50 0.0 5.0
  prompt_float VEHICLE_LIDAR_ASSOCIATION_ANGLE_MARGIN_DEG "연관 각도 margin [deg]" 2.0 0.0 30.0
  prompt_float VEHICLE_LIDAR_ASSOCIATION_DISTANCE_TOLERANCE_M "연관 거리 허용오차 [m]" 0.35 0.0 3.0
  prompt_float VEHICLE_BODY_LENGTH_M "장애 차량 길이 가정 [m]" 0.55 0.05 3.0
  prompt_float VEHICLE_BODY_WIDTH_M "장애 차량 폭 가정 [m]" 0.28 0.05 3.0
  prompt_float LIDAR_OBSTACLE_MINIMUM_DISTANCE_M "LiDAR 최소 유효거리 [m]" 0.18 0.0 3.0
  prompt_integer LIDAR_OBSTACLE_MINIMUM_CLUSTER_POINTS "장애물 군집 최소 점 수" 3 1 100
  prompt_integer LIDAR_OBSTACLE_MAXIMUM_SCAN_INDEX_GAP "군집 최대 scan index 간격" 2 0 20
  prompt_float LIDAR_OBSTACLE_MAXIMUM_CLUSTER_GAP_M "군집 점 최대 간격 [m]" 0.16 0.01 2.0
  prompt_float LIDAR_OBSTACLE_MINIMUM_CLUSTER_WIDTH_M "군집 최소 폭 [m]" 0.09 0.0 3.0
  prompt_float LIDAR_OBSTACLE_MAXIMUM_CLUSTER_WIDTH_M "군집 최대 폭 [m]" 0.70 0.01 5.0
  prompt_float LIDAR_OBSTACLE_SIDE_PROBE_INNER_M "측면 탐색 시작 거리 [m]" 0.18 0.0 3.0
  prompt_float LIDAR_OBSTACLE_SIDE_PROBE_OUTER_M "측면 탐색 끝 거리 [m]" 0.55 0.01 5.0
  prompt_float LIDAR_OBSTACLE_LIDAR_X_M "LiDAR x 위치 [m]" 0.065 -2.0 2.0
  prompt_float LIDAR_OBSTACLE_LIDAR_Y_M "LiDAR y 위치 [m]" 0.0 -2.0 2.0
  prompt_float LIDAR_OBSTACLE_LIDAR_YAW_DEG "LiDAR yaw [deg]" 0.0 -180.0 180.0
else
  VEHICLE_CAMERA_LIDAR_HFOV_DEG=60.0
  VEHICLE_CAMERA_LIDAR_PADDING_DEG=3.0
  VEHICLE_LIDAR_MIN_POINTS=2
  VEHICLE_LIDAR_SECTOR_MEMORY_SEC=0.50
  VEHICLE_LIDAR_ASSOCIATION_ANGLE_MARGIN_DEG=2.0
  VEHICLE_LIDAR_ASSOCIATION_DISTANCE_TOLERANCE_M=0.35
  VEHICLE_BODY_LENGTH_M=0.55
  VEHICLE_BODY_WIDTH_M=0.28
  LIDAR_OBSTACLE_MINIMUM_DISTANCE_M=0.18
  LIDAR_OBSTACLE_MINIMUM_CLUSTER_POINTS=3
  LIDAR_OBSTACLE_MAXIMUM_SCAN_INDEX_GAP=2
  LIDAR_OBSTACLE_MAXIMUM_CLUSTER_GAP_M=0.16
  LIDAR_OBSTACLE_MINIMUM_CLUSTER_WIDTH_M=0.09
  LIDAR_OBSTACLE_MAXIMUM_CLUSTER_WIDTH_M=0.70
  LIDAR_OBSTACLE_SIDE_PROBE_INNER_M=0.18
  LIDAR_OBSTACLE_SIDE_PROBE_OUTER_M=0.55
  LIDAR_OBSTACLE_LIDAR_X_M=0.065
  LIDAR_OBSTACLE_LIDAR_Y_M=0.0
  LIDAR_OBSTACLE_LIDAR_YAW_DEG=0.0
fi

if [[ "$CONE_AS_VEHICLE_OBSTACLE" == "true" ]]; then
  export XYCAR_TEST_PROFILE=cone_obstacle
else
  export XYCAR_TEST_PROFILE=avoidance_only
fi
export XYCAR_ENABLE_RVIZ=true
export XYCAR_START_CONE=false
export XYCAR_STEERING_ONLY="$STEERING_ONLY"
export XYCAR_HYBRID_RUN_CONFIG_FILE=/tmp/xycar_avoidance_run_config.yaml
export VEHICLE_YOLO_MIN_CONFIDENCE
export CONE_AS_VEHICLE_OBSTACLE
export CONE_AS_VEHICLE_MIN_CONFIDENCE
export VEHICLE_YOLO_REQUIRED_FRAMES
export VEHICLE_PREFERRED_SIDE_REQUIRED_FRAMES
export VEHICLE_YOLO_TIMEOUT_SEC
export VEHICLE_CAMERA_LIDAR_HFOV_DEG
export VEHICLE_CAMERA_LIDAR_PADDING_DEG
export VEHICLE_LIDAR_MIN_POINTS
export VEHICLE_LIDAR_SECTOR_MEMORY_SEC
export VEHICLE_LIDAR_ASSOCIATION_ANGLE_MARGIN_DEG
export VEHICLE_LIDAR_ASSOCIATION_DISTANCE_TOLERANCE_M
export VEHICLE_AVOIDANCE_IMMEDIATE_ON_YOLO
export VEHICLE_AVOIDANCE_ENTRY_DISTANCE_M
export VEHICLE_MINIMUM_SIDE_CLEARANCE_M
export VEHICLE_LEFT_OFFSET_M
export VEHICLE_RIGHT_OFFSET_M
export VEHICLE_OFFSET_RATE_MPS
export VEHICLE_AVOIDANCE_SPEED_LIMIT_COMMAND
export VEHICLE_MINIMUM_AVOID_SEC
export VEHICLE_CLEAR_HOLD_SEC
export VEHICLE_RETURN_HOLD_SEC
export VEHICLE_RETURN_DEADBAND_M
export VEHICLE_BODY_LENGTH_M
export VEHICLE_BODY_WIDTH_M
export LIDAR_OBSTACLE_DETECT_DISTANCE_M
export LIDAR_OBSTACLE_MINIMUM_DISTANCE_M
export LIDAR_OBSTACLE_PATH_HALF_WIDTH_M
export LIDAR_OBSTACLE_MINIMUM_CLUSTER_POINTS
export LIDAR_OBSTACLE_MAXIMUM_SCAN_INDEX_GAP
export LIDAR_OBSTACLE_MAXIMUM_CLUSTER_GAP_M
export LIDAR_OBSTACLE_MINIMUM_CLUSTER_WIDTH_M
export LIDAR_OBSTACLE_MAXIMUM_CLUSTER_WIDTH_M
export LIDAR_OBSTACLE_SIDE_PROBE_INNER_M
export LIDAR_OBSTACLE_SIDE_PROBE_OUTER_M
export LIDAR_OBSTACLE_LIDAR_X_M
export LIDAR_OBSTACLE_LIDAR_Y_M
export LIDAR_OBSTACLE_LIDAR_YAW_DEG

echo
echo "[준비] RViz, 카메라, LiDAR, VESC와 회피 전용 제어기를 시작합니다."
echo "[조작] READY 후 SPACE=주행, 다시 SPACE=정지, Ctrl+C=전체 종료"

exec "$SCRIPT_DIR/run_complete_space_hybrid.sh" \
  "$SPEED_COMMAND" "$LOOKAHEAD_DISTANCE" "$STANLEY_PERCENT" \
  "$LEFT_OFFSET_CM"
