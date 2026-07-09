#!/usr/bin/env bash
set -euo pipefail

OUT="${OUT:-$HOME/xycar_ws/bags/il/overtake_$(date +%Y%m%d_%H%M%S)}"
mkdir -p "$(dirname "$OUT")"

ros2 bag record \
  -o "$OUT" \
  /image_raw \
  /scan \
  /imu \
  /odom \
  /xycar_motor \
  /il/mission_label
