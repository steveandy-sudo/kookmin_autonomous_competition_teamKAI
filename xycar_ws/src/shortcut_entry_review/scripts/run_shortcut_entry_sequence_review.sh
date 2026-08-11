#!/usr/bin/env bash
set -eo pipefail

workspace=/home/kai/kookmin_autonomous_competition_teamKAI/xycar_ws
scripts=${workspace}/src/shortcut_entry_review/scripts

source /opt/ros/humble/setup.bash
cd "${workspace}"
colcon build --packages-select \
  shortcut_entry_review \
  track_drive_sve \
  xycar_map_nav \
  xycar_rule_drive \
  --symlink-install

setsid -f gnome-terminal \
  --title=Shortcut-W1-Y1-Canonical-Control \
  -- /bin/bash "${scripts}/run_shortcut_entry_sequence_stack.sh"
sleep 4
setsid -f gnome-terminal \
  --title=Shortcut-rosbag-A-back-0.2sec \
  -- /bin/bash "${scripts}/play_shortcut_bag_interactive.sh"

echo "Shortcut W1/Y1 sequence review started."
echo "OpenCV: canonical model input + selected W1/Y1 path"
echo "rosbag: a=back 0.2 second, SPACE=play/pause"
