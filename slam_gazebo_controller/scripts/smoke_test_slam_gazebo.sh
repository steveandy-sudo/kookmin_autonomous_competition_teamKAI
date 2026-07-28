#!/usr/bin/env bash
set -eo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
workspace="${repo_root}/xycar_ws"
package_source="${workspace}/src/xycar_gazebo_bridge"
world="${package_source}/worlds/slam_glass_balanced.sdf"
log_file="$(mktemp /tmp/team_kai_slam_gazebo.XXXXXX.log)"
sim_pid=""

cleanup() {
  if [[ -n "${sim_pid}" ]] && kill -0 "${sim_pid}" 2>/dev/null; then
    kill -INT "${sim_pid}" 2>/dev/null || true
    for _ in $(seq 1 30); do
      if ! kill -0 "${sim_pid}" 2>/dev/null; then
        break
      fi
      sleep 0.1
    done
    if kill -0 "${sim_pid}" 2>/dev/null; then
      kill -KILL "${sim_pid}" 2>/dev/null || true
    fi
    wait "${sim_pid}" 2>/dev/null || true
  fi
  rm -f "${log_file}"
}
trap cleanup EXIT

source /opt/ros/humble/setup.bash
source "${workspace}/install/setup.bash"
set -u
export GZ_SIM_RESOURCE_PATH="${package_source}:${GZ_SIM_RESOURCE_PATH:-}"

gz sdf -k "${world}"
gz sim -s -r -v 2 "${world}" >"${log_file}" 2>&1 &
sim_pid=$!

service_name="/world/kookmin_xycar_track/set_pose"
for _ in $(seq 1 40); do
  if gz service -l 2>/dev/null | grep -Fxq "${service_name}"; then
    break
  fi
  if ! kill -0 "${sim_pid}" 2>/dev/null; then
    echo "Gazebo server exited before advertising ${service_name}" >&2
    sed -n '1,160p' "${log_file}" >&2
    exit 1
  fi
  sleep 0.5
done

if ! gz service -l | grep -Fxq "${service_name}"; then
  echo "Timed out waiting for ${service_name}" >&2
  sed -n '1,160p' "${log_file}" >&2
  exit 1
fi

ros2 run xycar_gazebo_bridge xycar_sim_control \
  set-pose xycar_ackermann --x 8.2 --y 7.1 --z 0.05 --yaw-deg -150
ros2 run xycar_gazebo_bridge xycar_sim_control \
  spawn smoke_box box --x 7.7 --y 7.0 --yaw-deg 15
ros2 run xycar_gazebo_bridge xycar_sim_control \
  move smoke_box --x 7.9 --y 7.2 --yaw-deg 30
ros2 run xycar_gazebo_bridge xycar_sim_control remove smoke_box

echo "PASS: SLAM world loaded and coordinate create/move/remove services succeeded."
