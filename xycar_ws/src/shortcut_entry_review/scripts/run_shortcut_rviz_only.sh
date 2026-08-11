#!/usr/bin/env bash
set -eo pipefail

source /opt/ros/humble/setup.bash
source /home/kai/kookmin_autonomous_competition_teamKAI/xycar_ws/install/setup.bash

export ROS_LOG_DIR=/tmp/shortcut_entry_ros_logs
export ROS_DOMAIN_ID=47
export ROS_LOCALHOST_ONLY=1

exec rviz2 \
  -d /home/kai/kookmin_autonomous_competition_teamKAI/xycar_ws/install/shortcut_entry_review/share/shortcut_entry_review/rviz/shortcut_entry_bag_review.rviz \
  --ros-args -r __node:=shortcut_entry_review_rviz
