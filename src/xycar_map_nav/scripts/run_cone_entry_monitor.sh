#!/usr/bin/env bash

set -eo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "$(readlink -f "${BASH_SOURCE[0]}")")" && pwd)"
WORKSPACE="${XYCAR_WS:-$(cd -- "$SCRIPT_DIR/../../.." && pwd)}"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-7}"

source /opt/ros/humble/setup.bash
if [[ ! -f "$WORKSPACE/install/setup.bash" ]]; then
  echo "[오류] $WORKSPACE/install/setup.bash 가 없습니다." >&2
  echo "먼저 workspace를 colcon build 하세요." >&2
  exit 1
fi
source "$WORKSPACE/install/setup.bash"

exec 9>"/tmp/xycar_cone_entry_monitor.lock"
if ! flock -n 9; then
  echo "[오류] 라바콘 진입 감시가 이미 실행 중입니다." >&2
  exit 1
fi

nodes="$(ros2 node list 2>/dev/null || true)"
conflicts=()
for node in \
  /xycar_vesc_driver \
  /space_drive_gate \
  /sequential_hybrid_driver \
  /canonical_stanley_pursuit_driver \
  /my_rule_object_detection_node \
  /my_rule_cone_node \
  /wide_camera \
  /xycar_lidar_node; do
  if grep -Fxq "$node" <<<"$nodes"; then
    conflicts+=("$node")
  fi
done
if (( ${#conflicts[@]} > 0 )); then
  echo "[오류] 기존 센서/주행 노드가 남아 있어 안전하게 시작할 수 없습니다." >&2
  printf '  - %s\n' "${conflicts[@]}" >&2
  echo "기존 실행 터미널에서 Ctrl+C 후 다시 실행하세요." >&2
  exit 1
fi

echo "[안전 모드] VESC 노드를 시작하지 않습니다. 바퀴 구동 출력은 없습니다."
echo "[구성] 통합 차선→라바콘 선택 로직은 정상 계산하고 화면에만 표시합니다."
echo "[ROS_DOMAIN_ID] $ROS_DOMAIN_ID"

exec ros2 launch xycar_map_nav cone_entry_monitor.launch.py
