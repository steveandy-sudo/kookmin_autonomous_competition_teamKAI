#!/usr/bin/env bash

set -eo pipefail
set +u

WORKSPACE="/home/xytron/kookmin_ty/slam_gazebo_controller/xycar_ws"
DRIVE_ENABLED="false"

case "${1:-}" in
  "")
    ;;
  --drive)
    DRIVE_ENABLED="true"
    ;;
  --shadow)
    ;;
  *)
    echo "Usage: motor [--shadow|--drive]"
    exit 2
    ;;
esac

echo "============================================================"
echo " Xycar native ROS2 VESC driver"
echo " workspace: ${WORKSPACE}"
echo " drive_enabled: ${DRIVE_ENABLED}"
echo "============================================================"

if [[ ! -e /dev/ttyMOTOR ]]; then
  echo "[ERROR] /dev/ttyMOTOR is missing."
  echo "Check VESC power, USB connection, and the persistent udev link."
  exit 1
fi

# The legacy stack must not retain the serial port or bridge motor commands.
docker stop ros1_container >/dev/null 2>&1 || true
pkill -TERM -f '[r]os1_bridge/dynamic_bridge' >/dev/null 2>&1 || true
pkill -TERM -f '[r]os2 run ros1_bridge dynamic_bridge' >/dev/null 2>&1 || true

cd "${WORKSPACE}"
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-7}"
unset ROS_NAMESPACE
unset ROS_MASTER_URI
unset ROS_HOSTNAME
unset ROS_IP

PREFIX="$(ros2 pkg prefix xycar_vesc_driver)"
EXPECTED="${WORKSPACE}/install/xycar_vesc_driver"
if [[ "${PREFIX}" != "${EXPECTED}" ]]; then
  echo "[ERROR] wrong xycar_vesc_driver overlay: ${PREFIX}"
  echo "Expected: ${EXPECTED}"
  exit 1
fi

exec ros2 launch xycar_vesc_driver xycar_vesc_driver.launch.py \
  port:=/dev/ttyMOTOR \
  drive_enabled:="${DRIVE_ENABLED}"
