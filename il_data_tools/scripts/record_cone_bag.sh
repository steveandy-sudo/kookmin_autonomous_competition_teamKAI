#!/usr/bin/env bash
set -euo pipefail

OUT="${OUT:-$HOME/xycar_ws/bags/il/cone_$(date +%Y%m%d_%H%M%S)}"
mkdir -p "$(dirname "$OUT")"

ros2 bag record \
  -o "$OUT" \
  /usb_cam/image_raw/front \
  /scan \
  /imu \
  /odom \
  /xycar_motor \
  /il/mission_label
