#!/usr/bin/env bash
set -eo pipefail

PROJECT_ROOT="${1:-$HOME/kookmin_sim_to_real}"
SAMPLE_IMAGE="${2:-}"

# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash
# shellcheck disable=SC1091
source "$PROJECT_ROOT/install/setup.bash"
set -u

python3 - <<'PY'
import cv2
import numpy
import torch
import ultralytics
from cv_bridge import CvBridge

print(f"numpy={numpy.__version__}")
print(f"opencv={cv2.__version__}")
print(f"torch={torch.__version__} cuda={torch.cuda.is_available()}")
print(f"ultralytics={ultralytics.__version__}")
print(f"cv_bridge={CvBridge.__module__}")
PY

MODEL="$(ros2 pkg prefix xycar_perception)/share/xycar_perception/models/kookmin_lane_yolo11n_512.pt"
EXPECTED_SHA256="7cd02f180ce5e5f3d7066e634ea17b16418e02dc0c2e93465b3bfb3bf2a5c9fd"
ACTUAL_SHA256="$(sha256sum "$MODEL" | awk '{print $1}')"

printf 'model=%s\n' "$MODEL"
printf 'model_sha256=%s\n' "$ACTUAL_SHA256"
if [[ "$ACTUAL_SHA256" != "$EXPECTED_SHA256" ]]; then
  printf 'ERROR: lane model checksum mismatch\n' >&2
  exit 1
fi

if [[ -n "$SAMPLE_IMAGE" ]]; then
  ros2 run xycar_perception benchmark_yolo_lane \
    --model "$MODEL" \
    --source "$SAMPLE_IMAGE" \
    --device cpu \
    --image-size 512 \
    --cpu-threads 4 \
    --iterations 100
fi

printf 'ASUS YOLO runtime check passed.\n'
