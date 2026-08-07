#!/usr/bin/env bash

set -eo pipefail

ROOT_DIR="$(cd -- "$(dirname -- "$(readlink -f "${BASH_SOURCE[0]}")")" && pwd)"
WORKSPACE="$ROOT_DIR/xycar_ws"
SESSION_NAME="${1:-direct_bev_run}"
STAMP="$(date +%Y%m%d_%H%M%S)"
OUTPUT_ROOT="${XYCAR_BAG_ROOT:-$HOME/rosbags/integrated_direct_bev}"
SESSION_DIR="$OUTPUT_ROOT/${SESSION_NAME}_${STAMP}"
BAG_DIR="$SESSION_DIR/bag"

set +u
source /opt/ros/humble/setup.bash
source "$WORKSPACE/install/setup.bash"
set -u
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-7}"
unset ROS_NAMESPACE || true

publisher_count() {
  ros2 topic info "$1" 2>/dev/null \
    | awk '/Publisher count:/ { print $3; exit }'
}

require_topic() {
  local topic="$1"
  local count
  count="$(publisher_count "$topic")"
  if [[ -z "$count" || "$count" -lt 1 ]]; then
    echo "[문제: 필수 토픽 없음] $topic publisher가 없습니다." >&2
    echo "[확인 방법] 통합 주행을 READY 상태까지 먼저 실행하세요." >&2
    exit 2
  fi
}

case "$SESSION_NAME" in
  *[!A-Za-z0-9_.-]*|'')
    echo "[문제: 세션 이름 오류] 영문, 숫자, _, -, .만 사용할 수 있습니다." >&2
    exit 2
    ;;
esac

require_topic /wide_camera_mjpeg/image_raw/compressed
require_topic /lane_path/status
require_topic /hybrid/rule_candidate
require_topic /hybrid_gate/status

mkdir -p "$SESSION_DIR"

MODEL="$WORKSPACE/src/lane_seg_control/models/best_512.onnx"
{
  echo "recorded_at: $(date --iso-8601=seconds)"
  echo "workspace: $WORKSPACE"
  echo "session: $SESSION_NAME"
  echo "ros_domain_id: $ROS_DOMAIN_ID"
  echo "model: $MODEL"
  if [[ -f "$MODEL" ]]; then
    echo "model_sha256: $(sha256sum "$MODEL" | awk '{print $1}')"
  fi
  echo "git_head: $(git -C "$ROOT_DIR" rev-parse HEAD 2>/dev/null || echo unknown)"
} >"$SESSION_DIR/manifest.txt"

if [[ -f /tmp/xycar_hybrid_run_config.yaml ]]; then
  cp /tmp/xycar_hybrid_run_config.yaml "$SESSION_DIR/run_config.yaml"
fi

topics=(
  /wide_camera_mjpeg/image_raw/compressed
  /lane_seg/white_boundary_mask
  /lane_seg/yellow_centerline_mask
  /lane_seg_bev/white_mask
  /lane_seg_bev/yellow_mask
  /lane_path/path_pixels
  /lane_path/path_normalized
  /lane_path/diagnostics
  /lane_path/status
  /perception/bev_direct_centerline
  /rule_drive/connected_yellow_path
  /rule_drive/diagnostics
  /rl/action_applied
  /hybrid/rule_candidate
  /hybrid_gate/xycar_motor_shadow
  /hybrid_gate/mode
  /hybrid_gate/status
  /hybrid_gate/diagnostics
  /hybrid_gate/drive_armed
  /hybrid/avoidance_lateral_offset
  /hybrid/avoidance_debug
  /hybrid/avoidance_path_request
  /my_rule/object_detections
  /my_rule/cone_cmd
  /my_rule/cone_clusters
  /my_rule/cone_processing_enabled
  /scan
  /vehicle/vesc_state
  /vehicle/system_telemetry
  /xycar_motor
  /parameter_events
)

echo
echo "========== 통합 직접-BEV rosbag 기록 =========="
echo "세션: $SESSION_NAME"
echo "위치: $SESSION_DIR"
echo "영상: 압축 원본 MJPEG + 희소 차선/BEV 마스크"
echo "제어: 직접 경로 + RULE 후보 + 콘/회피 + 최종 모터/VESC"
echo "종료: Ctrl+C"
echo

set +e
ros2 bag record \
  --storage sqlite3 \
  --compression-mode file \
  --compression-format zstd \
  --compression-threads 1 \
  --compression-queue-size 2 \
  --max-cache-size 268435456 \
  --output "$BAG_DIR" \
  "${topics[@]}"
status=$?
set -e

if [[ "$status" -ne 0 && "$status" -ne 130 ]]; then
  echo "[문제: rosbag 저장 실패] 종료 상태=$status" >&2
  exit "$status"
fi

echo
echo "========== 저장 완료 =========="
echo "$SESSION_DIR"
ros2 bag info "$BAG_DIR" | tee "$SESSION_DIR/bag_info.txt"
