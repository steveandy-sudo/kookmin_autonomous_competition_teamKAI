#!/usr/bin/env bash

# Start the complete Gazebo mission stack in shadow mode, then expose the
# same SPACE-to-run terminal used on the real car.

set -eo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "$(readlink -f "${BASH_SOURCE[0]}")")" && pwd)"
if [[ -n "${XYCAR_WS:-}" ]]; then
  WORKSPACE="$(cd -- "$XYCAR_WS" && pwd)"
else
  source_tree_workspace="$(cd -- "$SCRIPT_DIR/../../.." && pwd)"
  installed_workspace="$(cd -- "$SCRIPT_DIR/../../../../.." && pwd)"
  if [[ -f "$source_tree_workspace/install/setup.bash" ]]; then
    WORKSPACE="$source_tree_workspace"
  elif [[ -f "$installed_workspace/install/setup.bash" ]]; then
    WORKSPACE="$installed_workspace"
  else
    echo "ERROR: cannot locate the xycar_ws workspace." >&2
    echo "Set XYCAR_WS to the workspace path and retry." >&2
    exit 2
  fi
fi
PROJECT_ROOT="${XYCAR_PROJECT_ROOT:-$(cd -- "$WORKSPACE/.." && pwd)}"
SPEED_COMMAND="${1:-3.0}"
CONE_SPEED_COMMAND="${2:-6.0}"
CONTROL_LOG="/tmp/xycar_sim_missions_$(date +%Y%m%d_%H%M%S).log"

if [[ ! "$SPEED_COMMAND" =~ ^[0-9]+([.][0-9]+)?$ ]] || \
  ! awk -v speed="$SPEED_COMMAND" \
    'BEGIN { exit !(speed >= 3.0 && speed <= 30.0) }'; then
  echo "ERROR: speed command must be from 3.0 to 30.0." >&2
  exit 2
fi
if [[ ! "$CONE_SPEED_COMMAND" =~ ^[0-9]+([.][0-9]+)?$ ]] || \
  ! awk -v speed="$CONE_SPEED_COMMAND" \
    'BEGIN { exit !(speed >= 3.0 && speed <= 12.0) }'; then
  echo "ERROR: cone speed command must be from 3.0 to 12.0." >&2
  exit 2
fi
SPEED_COMMAND="$(awk -v speed="$SPEED_COMMAND" \
  'BEGIN { printf "%.3f", speed }')"
CONE_SPEED_COMMAND="$(awk -v speed="$CONE_SPEED_COMMAND" \
  'BEGIN { printf "%.3f", speed }')"

set +u
source /opt/ros/humble/setup.bash
source "$WORKSPACE/install/setup.bash" 2>/dev/null
set -u

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-7}"
unset ROS_NAMESPACE || true

LOCK_FILE="/tmp/xycar_sim_missions_domain_${ROS_DOMAIN_ID}.lock"
exec 9>"$LOCK_FILE"
if ! flock -n 9; then
  echo "ERROR: this simulation is already running (ROS_DOMAIN_ID=$ROS_DOMAIN_ID)." >&2
  echo "Use the existing SPACE terminal, or close it with Q before restarting." >&2
  exit 3
fi

launch_pid=""

cleanup() {
  set +e
  timeout 2s ros2 topic pub --once \
    /xycar_motor std_msgs/msg/Float32MultiArray \
    "{data: [0.0, 0.0]}" >/dev/null 2>&1 || true
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

wait_for_topic() {
  local topic="$1"
  local label="$2"
  local attempt
  printf '  [확인 중] %-12s %s\n' "$label" "$topic"
  for attempt in $(seq 1 30); do
    if ! kill -0 "$launch_pid" 2>/dev/null; then
      echo "[문제] 시뮬레이션 launch가 준비 도중 종료되었습니다." >&2
      tail -n 50 "$CONTROL_LOG" >&2 || true
      return 1
    fi
    if timeout --signal=INT --kill-after=1s 2s \
      ros2 topic echo "$topic" --once \
      --qos-reliability best_effort >/dev/null 2>&1; then
      printf '  [OK] %-12s %s\n' "$label" "$topic"
      return 0
    fi
  done
  echo "[문제] $label 데이터가 준비되지 않았습니다: $topic" >&2
  echo "[제어 로그] $CONTROL_LOG" >&2
  return 1
}

echo "Gazebo RULE + CONE + YOLO-LiDAR avoidance를 정지 상태로 시작합니다."
echo "Priority: CONE > YOLO+LiDAR AVOIDANCE > RULE"
echo "Speed command: $SPEED_COMMAND | Cone speed: $CONE_SPEED_COMMAND"
echo "Control detail log: $CONTROL_LOG"

setsid ros2 launch xycar_map_nav sim_rule_missions.launch.py \
  project_root:="$PROJECT_ROOT" \
  headless:=gui \
  enable_rviz:=true \
  drive_enabled:=false \
  gate_arming_required:=true \
  speed_command:="$SPEED_COMMAND" \
  cone_speed_command:="$CONE_SPEED_COMMAND" \
  maximum_speed_command:=30.0 \
  >"$CONTROL_LOG" 2>&1 &
launch_pid=$!

sleep 1
if ! kill -0 "$launch_pid" 2>/dev/null; then
  wait "$launch_pid"
  exit 1
fi

echo
echo "========== 시뮬레이션 준비 확인 =========="
wait_for_topic /image_raw "카메라"
wait_for_topic /scan "LiDAR"
wait_for_topic /hybrid/rule_candidate "룰베이스"
wait_for_topic /hybrid_gate/status "주행 선택기"

echo
echo "========== READY | VEHICLE STOPPED =========="
echo "SPACE: RUN/STOP | Q 또는 ESC: 종료"

ros2 run xycar_map_nav space_drive_gate --ros-args \
  -p speed_command:="$SPEED_COMMAND" \
  -p maximum_speed_command:=30.0
