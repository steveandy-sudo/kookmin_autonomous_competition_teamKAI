#!/usr/bin/env bash
set -eo pipefail

source /opt/ros/humble/setup.bash
source /home/kai/kookmin_autonomous_competition_teamKAI/xycar_ws/install/setup.bash

export ROS_LOG_DIR=/tmp/shortcut_entry_ros_logs
export ROS_DOMAIN_ID=47
export ROS_LOCALHOST_ONLY=1

exec ros2 run shortcut_entry_review annotation_keyboard
