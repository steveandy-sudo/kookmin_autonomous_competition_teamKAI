#!/usr/bin/env bash
set -eo pipefail

workspace=/home/kai/kookmin_autonomous_competition_teamKAI/xycar_ws
scripts=${workspace}/src/shortcut_entry_review/scripts

source /opt/ros/humble/setup.bash
cd "${workspace}"
colcon build --packages-select shortcut_entry_review --symlink-install

setsid -f gnome-terminal \
  --title=Shortcut-RViz-stack \
  -- /bin/bash "${scripts}/run_shortcut_review_rviz.sh"
sleep 4
setsid -f gnome-terminal \
  --title=Shortcut-annotation-W-Y \
  -- /bin/bash "${scripts}/run_annotation_keyboard.sh"
setsid -f gnome-terminal \
  --title=Shortcut-rosbag-A-back-0.2sec \
  -- /bin/bash "${scripts}/play_shortcut_bag_interactive.sh"

echo "Shortcut annotation suite started."
echo "rosbag: a=back 0.2 second, SPACE=play/pause"
echo "annotation: w=W1/W2, y=Y1/Y2, s=save"
