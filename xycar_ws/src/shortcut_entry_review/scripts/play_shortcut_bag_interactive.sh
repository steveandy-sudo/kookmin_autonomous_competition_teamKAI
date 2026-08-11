#!/usr/bin/env bash
set -eo pipefail

source /opt/ros/humble/setup.bash
source /home/kai/kookmin_autonomous_competition_teamKAI/xycar_ws/install/setup.bash

export ROS_LOG_DIR=/tmp/shortcut_entry_ros_logs
export ROS_DOMAIN_ID=47
export ROS_LOCALHOST_ONLY=1

bag_path=/home/kai/Downloads/drive-download-20260811T010528Z-1-001
source_topic=/wide_camera_mjpeg/image_raw/compressed
bag_start_offset=0.0

ros2 bag play \
  "${bag_path}" \
  --topics "${source_topic}" \
  --start-offset "${bag_start_offset}" \
  --rate 0.25 \
  --start-paused \
  --disable-keyboard-controls \
  --clock &
bag_player_pid=$!

cleanup() {
  kill -INT "${bag_player_pid}" 2>/dev/null || true
  wait "${bag_player_pid}" 2>/dev/null || true
}
trap cleanup EXIT INT TERM

ros2 run shortcut_entry_review bag_keyboard_controller \
  "${bag_path}" \
  --source-topic "${source_topic}" \
  --start-offset "${bag_start_offset}" \
  --autoplay
