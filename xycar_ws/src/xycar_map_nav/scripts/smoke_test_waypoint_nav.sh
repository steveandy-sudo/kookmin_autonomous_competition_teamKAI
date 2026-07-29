#!/usr/bin/env bash
set -eo pipefail

PACKAGE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
WORKSPACE_DIR="$(cd "${PACKAGE_DIR}/../.." && pwd)"
MAP_YAML="${PACKAGE_DIR}/../xycar_gazebo_bridge/maps/slam_glass_balanced/slam_glass_balanced.yaml"
WAYPOINTS_YAML="${PACKAGE_DIR}/config/slam_glass_balanced_example_waypoints.yaml"
PARAMS_YAML="${PACKAGE_DIR}/config/waypoint_nav_real.yaml"
LOG_DIR="${WORKSPACE_DIR}/log/xycar_map_nav_smoke"
mkdir -p "${LOG_DIR}"

source /opt/ros/humble/setup.bash
source "${WORKSPACE_DIR}/install/setup.bash"
set -u
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-77}"
ros2 daemon stop >/dev/null 2>&1 || true

NAV_PID=""
TF_PID=""
NAV_LOG=""
EXTRA_PIDS=()

cleanup() {
  for pid in "${EXTRA_PIDS[@]:-}" "${NAV_PID:-}" "${TF_PID:-}"; do
    if [[ -n "${pid}" ]]; then
      kill "${pid}" 2>/dev/null || true
      wait "${pid}" 2>/dev/null || true
    fi
  done
}
trap cleanup EXIT

start_case() {
  local name="$1"
  local x="$2"
  local y="$3"
  local yaw="$4"
  cleanup
  EXTRA_PIDS=()
  ros2 run tf2_ros static_transform_publisher \
    --x "${x}" --y "${y}" --z 0 \
    --yaw "${yaw}" --pitch 0 --roll 0 \
    --frame-id map --child-frame-id base_footprint \
    >"${LOG_DIR}/${name}_tf.log" 2>&1 &
  TF_PID=$!
  NAV_LOG="${LOG_DIR}/${name}_nav.log"
  ros2 run xycar_map_nav waypoint_nav_node --ros-args \
    --params-file "${PARAMS_YAML}" \
    -p map_yaml:="${MAP_YAML}" \
    -p waypoints_yaml:="${WAYPOINTS_YAML}" \
    -p drive_enabled:=false \
    >"${NAV_LOG}" 2>&1 &
  NAV_PID=$!
  sleep 2
  kill -0 "${NAV_PID}"
}

read_mode() {
  local expected="$1"
  local value
  for _ in $(seq 1 50); do
    value="$(
      sed -n 's/.*control mode -> \([^:]*\):.*/\1/p' "${NAV_LOG}" \
        | tail -n 1
    )"
    if [[ "${value}" =~ ${expected} ]]; then
      echo "${value}"
      return 0
    fi
    sleep 0.1
  done
  echo "expected ${expected}, last mode was ${value}" >&2
  return 1
}

start_case global 8.6493 7.4200 -2.8523
GLOBAL_MODE="$(read_mode '^GLOBAL_PATH$')"
[[ "${GLOBAL_MODE}" == "GLOBAL_PATH" ]]
echo "global mode: ${GLOBAL_MODE}"

start_case dynamic 0.2025 2.7577 -1.0367
ros2 topic pub -r 10 /yolo_obstacle/stable_counts \
  std_msgs/msg/Int32MultiArray "{data: [1, 0, 0, 0]}" \
  >"${LOG_DIR}/dynamic_counts.log" 2>&1 &
EXTRA_PIDS+=("$!")
sleep 1
DYNAMIC_MODE="$(read_mode '^DYNAMIC_VEHICLE_RULE_AVOID_RIGHT$')"
[[ "${DYNAMIC_MODE}" == "DYNAMIC_VEHICLE_RULE_AVOID_RIGHT" ]]
echo "dynamic mode: ${DYNAMIC_MODE}"

start_case cone 8.4239 -0.3851 -0.0600
ros2 topic pub -r 10 /hybrid/mode std_msgs/msg/String \
  "{data: 'CONE_RULE | smoke'}" \
  >"${LOG_DIR}/cone_mode.log" 2>&1 &
EXTRA_PIDS+=("$!")
ros2 topic pub -r 10 /xycar_motor_shadow \
  std_msgs/msg/Float32MultiArray "{data: [12.0, 4.0]}" \
  >"${LOG_DIR}/cone_command.log" 2>&1 &
EXTRA_PIDS+=("$!")
sleep 1
CONE_MODE="$(read_mode '^CONE_RULE$')"
[[ "${CONE_MODE}" == "CONE_RULE" ]]
grep -q "control mode -> CONE_RULE: angle=12.00, speed=4.00" "${NAV_LOG}"
echo "cone mode: ${CONE_MODE}; command forwarded"
echo "waypoint navigation smoke: PASS"
