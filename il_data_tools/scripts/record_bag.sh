#!/usr/bin/env bash
set -euo pipefail

XYCAR_WS="${XYCAR_WS:-$HOME/xycar_ws}"
RUN_ID="$(date +%Y%m%d_%H%M%S)"
OUTPUT_DIR="${1:-${BAG_OUTPUT_DIR:-$XYCAR_WS/bags/il/run_$RUN_ID}}"

FRONT_CAMERA_TOPIC="${FRONT_CAMERA_TOPIC:-/usb_cam/image_raw/front}"
LEFT_CAMERA_TOPIC="${LEFT_CAMERA_TOPIC:-/usb_cam/image_raw/left}"
RIGHT_CAMERA_TOPIC="${RIGHT_CAMERA_TOPIC:-/usb_cam/image_raw/right}"
REAR_CAMERA_TOPIC="${REAR_CAMERA_TOPIC:-/usb_cam/image_raw/behind}"
SCAN_TOPIC="${SCAN_TOPIC:-/scan}"
IMU_TOPIC="${IMU_TOPIC:-/imu}"
ODOM_TOPIC="${ODOM_TOPIC:-/odom}"
MOTOR_TOPIC="${MOTOR_TOPIC:-/xycar_motor}"
MISSION_LABEL_TOPIC="${MISSION_LABEL_TOPIC:-/il/mission_label}"

source_if_exists() {
  local setup_file="$1"
  if [ -f "$setup_file" ]; then
    # shellcheck source=/dev/null
    source "$setup_file"
  fi
}

source_if_exists /opt/ros/humble/setup.bash
source_if_exists "$XYCAR_WS/install/setup.bash"

mkdir -p "$(dirname "$OUTPUT_DIR")"

echo "== Xycar IL raw bag recorder =="
echo "Output: $OUTPUT_DIR"
echo
echo "This script does not start, stop, or manage Docker."
echo "If motor/VESC is needed, start the bridge separately before recording."
echo

echo "== Current ROS topics =="
ros2 topic list || true
echo

echo "== Useful rate checks =="
echo "ros2 topic hz $FRONT_CAMERA_TOPIC"
echo "ros2 topic hz $SCAN_TOPIC"
echo "ros2 topic hz $IMU_TOPIC"
echo

echo "== Disk usage =="
df -h "$XYCAR_WS" || true
echo

echo "== Motor topic publisher check =="
MOTOR_INFO="$(ros2 topic info "$MOTOR_TOPIC" -v 2>/dev/null || true)"
echo "$MOTOR_INFO"
PUBLISHER_COUNT="$(printf '%s\n' "$MOTOR_INFO" | awk -F': ' '/Publisher count/ {print $2; exit}')"
PUBLISHER_COUNT="${PUBLISHER_COUNT:-0}"
if [ "$PUBLISHER_COUNT" -gt 1 ]; then
  echo "WARN: $MOTOR_TOPIC has $PUBLISHER_COUNT publishers. Only one controller must publish motor commands."
elif [ "$PUBLISHER_COUNT" -eq 1 ]; then
  echo "PASS: $MOTOR_TOPIC has one publisher."
else
  echo "WARN: $MOTOR_TOPIC has no publisher yet. This is OK before driving starts."
fi
echo

TOPICS=(
  "$FRONT_CAMERA_TOPIC"
  "$LEFT_CAMERA_TOPIC"
  "$RIGHT_CAMERA_TOPIC"
  "$REAR_CAMERA_TOPIC"
  "$SCAN_TOPIC"
  "$IMU_TOPIC"
  "$ODOM_TOPIC"
  "$MOTOR_TOPIC"
  "$MISSION_LABEL_TOPIC"
)

echo "== Recording topics =="
printf '  %s\n' "${TOPICS[@]}"
echo
echo "Press Ctrl-C to stop recording cleanly."
ros2 bag record -o "$OUTPUT_DIR" "${TOPICS[@]}"
