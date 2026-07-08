#!/usr/bin/env bash
set -euo pipefail

OUT="${OUT:-$HOME/xycar_ws/bags/il/overtake_$(date +%Y%m%d_%H%M%S)}"
mkdir -p "$(dirname "$OUT")"

ros2 bag record \
  -o "$OUT" \
  /usb_cam/image_raw/front \
  /usb_cam/image_raw/left \
  /usb_cam/image_raw/right \
  /usb_cam/image_raw/behind \
  /scan \
  /imu \
  /odom \
  /xycar_motor \
  /il/mission_label
