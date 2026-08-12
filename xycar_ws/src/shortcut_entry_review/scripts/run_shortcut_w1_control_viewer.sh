#!/usr/bin/env bash
set -eo pipefail

repo=/home/kai/kookmin_autonomous_competition_teamKAI
workspace=${repo}/xycar_ws
bag=/home/kai/Downloads/drive-download-20260811T090224Z-1-001

source /opt/ros/humble/setup.bash
source "${workspace}/install/setup.bash"

export ROS_DOMAIN_ID=48
export ROS_LOCALHOST_ONLY=1
export ROS_LOG_DIR=/tmp/kai_shortcut_w1_viewer_logs
mkdir -p "${ROS_LOG_DIR}"

ros2 launch shortcut_entry_review shortcut_entry_semantic_control.launch.py \
  default_enabled:=true \
  use_sim_time:=true \
  show_opencv_windows:=false \
  rule_candidate_topic:=/shortcut/review/recorded_rule_diagnostics \
  rule_angle_index:=9 \
  rule_speed_index:=10 \
  vehicle_state_topic:=/vehicle/vesc_state \
  full_control_distance_m:=0.15 \
  minimum_start_distance_m:=0.55 \
  maximum_start_distance_m:=1.20 \
  control_latency_sec:=0.25 \
  distance_margin_m:=0.08 \
  w1_path_weight:=0.60 \
  > /tmp/kai_shortcut_w1_stack.log 2>&1 &
stack_pid=$!

# LR-ASPP loads its TorchScript model before creating the camera subscription.
# Leave enough startup margin so the player's priming frame is never lost.
sleep 5

ros2 bag play "${bag}" \
  --topics /wide_camera_mjpeg/image_raw/compressed /rule_drive/diagnostics /vehicle/vesc_state \
  --remap /rule_drive/diagnostics:=/shortcut/review/recorded_rule_diagnostics \
  --rate 0.5 \
  --clock \
  --loop \
  --start-paused \
  --disable-keyboard-controls \
  > /tmp/kai_shortcut_w1_bag.log 2>&1 &
bag_pid=$!

cleanup() {
  kill -INT "${bag_pid}" "${stack_pid}" 2>/dev/null || true
  # Do not leave this wrapper stuck forever if a ROS launch child ignores INT.
  for _ in {1..30}; do
    if ! kill -0 "${bag_pid}" 2>/dev/null && ! kill -0 "${stack_pid}" 2>/dev/null; then
      break
    fi
    sleep 0.1
  done
  kill -TERM "${bag_pid}" "${stack_pid}" 2>/dev/null || true

  # TERM is bounded as well.  Escalate only the two known child process
  # groups so the final wait below cannot leave the viewer launcher hanging.
  for _ in {1..20}; do
    if ! kill -0 "${bag_pid}" 2>/dev/null && ! kill -0 "${stack_pid}" 2>/dev/null; then
      break
    fi
    sleep 0.1
  done
  kill -KILL "${bag_pid}" "${stack_pid}" 2>/dev/null || true
  wait "${bag_pid}" "${stack_pid}" 2>/dev/null || true
}
trap cleanup EXIT INT TERM

ros2 run shortcut_entry_review shortcut_w1_control_viewer --ros-args \
  -p player_controls_enabled:=true \
  -p initial_paused:=true \
  -p autoplay:=true \
  -p playback_rate:=0.5
