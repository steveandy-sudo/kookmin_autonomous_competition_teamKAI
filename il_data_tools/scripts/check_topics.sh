#!/usr/bin/env bash
set -u

pass() { printf 'PASS %s\n' "$*"; }
warn() { printf 'WARN %s\n' "$*"; }
fail() { printf 'FAIL %s\n' "$*"; }

check_topic_exists() {
  local topic="$1"
  if ros2 topic list | grep -qx "$topic"; then
    pass "topic exists: $topic"
    return 0
  fi
  fail "topic missing: $topic"
  return 1
}

check_hz() {
  local topic="$1"
  if ! ros2 topic list | grep -qx "$topic"; then
    warn "skip hz; topic missing: $topic"
    return 0
  fi
  if timeout 5s ros2 topic hz "$topic" >/tmp/il_topic_hz.txt 2>&1; then
    pass "topic hz responded: $topic"
  else
    warn "topic hz did not finish cleanly within 5s: $topic"
  fi
  sed 's/^/  /' /tmp/il_topic_hz.txt | tail -n 5
}

echo "== ros2 topic list =="
if ros2 topic list; then
  pass "ros2 topic list"
else
  fail "ros2 topic list failed"
fi

echo
check_topic_exists "/usb_cam/image_raw/front"
check_topic_exists "/scan"
check_topic_exists "/imu"
check_topic_exists "/xycar_motor"

echo
check_hz "/usb_cam/image_raw/front"
check_hz "/scan"
check_hz "/imu"

echo
echo "== /xycar_motor info =="
if ros2 topic info /xycar_motor -v; then
  pass "topic info /xycar_motor"
else
  fail "topic info /xycar_motor failed"
fi

echo
echo "== disk =="
if df -h "$HOME/xycar_ws"; then
  pass "df -h ~/xycar_ws"
else
  warn "df -h ~/xycar_ws failed"
fi
