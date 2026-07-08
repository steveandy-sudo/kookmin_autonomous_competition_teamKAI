#!/usr/bin/env bash
set -u

KEEP=0
if [ "${1:-}" = "--keep" ]; then
  KEEP=1
fi

set +u
if [ -f /opt/ros/humble/setup.bash ]; then
  # shellcheck disable=SC1091
  source /opt/ros/humble/setup.bash
fi

if [ -f "$HOME/xycar_ws/install/setup.bash" ]; then
  # shellcheck disable=SC1091
  source "$HOME/xycar_ws/install/setup.bash"
fi
set -u

if [ "$KEEP" -eq 0 ]; then
  rm -rf /tmp/il_test
  echo "removed /tmp/il_test"
else
  echo "keeping existing /tmp/il_test"
fi

cat <<'EOF'

Safe local recorder dry-run.
This test uses /test/image and /test/xycar_motor.
It never publishes to the real /xycar_motor topic.

Terminal 1:
  ros2 run il_data_tools il_mission_labeler

Terminal 2:
  ros2 launch il_data_tools record_drive_dataset.launch.py \
    session_name:=dummy_drive_test \
    output_root:=/tmp/il_test \
    camera_front_topic:=/test/image \
    motor_topic:=/test/xycar_motor \
    motor_msg_type:=float32_multi_array \
    mission_label_topic:=/il/mission_label

Terminal 3:
  ros2 run il_data_tools publish_dummy_il_stream.py \
    --image-topic /test/image \
    --motor-topic /test/xycar_motor \
    --label general_drive \
    --duration-sec 12

After stopping the recorder:
  find /tmp/il_test -maxdepth 6 -type f | sort | head -80
  SAMPLES=$(find /tmp/il_test -name "samples.csv" | tail -n 1)
  echo "$SAMPLES"
  head -5 "$SAMPLES"

EOF
