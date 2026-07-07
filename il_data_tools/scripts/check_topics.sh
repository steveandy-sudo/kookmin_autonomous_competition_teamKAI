#!/usr/bin/env bash
set -u

XYCAR_WS="${XYCAR_WS:-$HOME/xycar_ws}"
FRONT_CAMERA_TOPIC="${FRONT_CAMERA_TOPIC:-/usb_cam/image_raw/front}"
SCAN_TOPIC="${SCAN_TOPIC:-/scan}"
IMU_TOPIC="${IMU_TOPIC:-/imu}"
MOTOR_TOPIC="${MOTOR_TOPIC:-/xycar_motor}"

source_if_exists() {
  local setup_file="$1"
  if [ -f "$setup_file" ]; then
    # shellcheck source=/dev/null
    source "$setup_file"
  fi
}

pass() { echo "PASS: $*"; }
warn() { echo "WARN: $*"; }
fail() { echo "FAIL: $*"; STATUS=1; }

STATUS=0
source_if_exists /opt/ros/humble/setup.bash
source_if_exists "$XYCAR_WS/install/setup.bash"

echo "== ROS topic list =="
TOPIC_LIST="$(ros2 topic list 2>/dev/null)"
if [ -z "$TOPIC_LIST" ]; then
  fail "No ROS topics found. Check ROS_DOMAIN_ID, network, and launched nodes."
else
  pass "ROS graph is visible."
  printf '%s\n' "$TOPIC_LIST"
fi
echo

has_topic() {
  local topic="$1"
  printf '%s\n' "$TOPIC_LIST" | grep -qx "$topic"
}

for topic in "$FRONT_CAMERA_TOPIC" "$SCAN_TOPIC" "$IMU_TOPIC" "$MOTOR_TOPIC"; do
  if has_topic "$topic"; then
    pass "$topic exists."
  else
    warn "$topic is not visible right now."
  fi
done
echo

check_hz() {
  local topic="$1"
  echo "== ros2 topic hz $topic =="
  if timeout 6 ros2 topic hz "$topic" 2>&1 | sed -n '1,8p'; then
    pass "hz check command completed for $topic."
  else
    warn "hz check did not complete for $topic. Topic may be idle or timeout is unavailable."
  fi
  echo
}

check_hz "$FRONT_CAMERA_TOPIC"
check_hz "$SCAN_TOPIC"
check_hz "$IMU_TOPIC"

echo "== $MOTOR_TOPIC info =="
MOTOR_INFO="$(ros2 topic info "$MOTOR_TOPIC" -v 2>/dev/null)"
if [ -z "$MOTOR_INFO" ]; then
  warn "$MOTOR_TOPIC info is unavailable."
else
  echo "$MOTOR_INFO"
fi
PUBLISHER_COUNT="$(printf '%s\n' "$MOTOR_INFO" | awk -F': ' '/Publisher count/ {print $2; exit}')"
SUBSCRIPTION_COUNT="$(printf '%s\n' "$MOTOR_INFO" | awk -F': ' '/Subscription count/ {print $2; exit}')"
PUBLISHER_COUNT="${PUBLISHER_COUNT:-0}"
SUBSCRIPTION_COUNT="${SUBSCRIPTION_COUNT:-0}"

if [ "$PUBLISHER_COUNT" -gt 1 ]; then
  fail "$MOTOR_TOPIC has $PUBLISHER_COUNT publishers. Stop duplicate controllers before driving."
elif [ "$PUBLISHER_COUNT" -eq 1 ]; then
  pass "$MOTOR_TOPIC has exactly one publisher."
else
  warn "$MOTOR_TOPIC has no publisher. This is normal before launching manual/autonomous driving."
fi

if [ "$SUBSCRIPTION_COUNT" -gt 0 ]; then
  pass "$MOTOR_TOPIC has $SUBSCRIPTION_COUNT subscriber(s)."
else
  warn "$MOTOR_TOPIC has no subscriber. Check motor bridge/VESC path if motor control is needed."
fi
echo

echo "== Disk usage =="
if df -h "$XYCAR_WS"; then
  pass "Disk usage checked for $XYCAR_WS."
else
  warn "Could not check disk usage for $XYCAR_WS."
fi

exit "$STATUS"
