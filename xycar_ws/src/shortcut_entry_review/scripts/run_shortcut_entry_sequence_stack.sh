#!/usr/bin/env bash
set -eo pipefail

source /opt/ros/humble/setup.bash
source /home/kai/kookmin_autonomous_competition_teamKAI/xycar_ws/install/setup.bash

export ROS_LOG_DIR=/tmp/shortcut_entry_sequence_ros_logs
export ROS_DOMAIN_ID=47
export ROS_LOCALHOST_ONLY=1

ros2 launch shortcut_entry_review shortcut_entry_sequence_bag_review.launch.py
