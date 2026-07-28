#!/usr/bin/env bash
set -euo pipefail

usage() {
  cat <<'EOF'
Record the real camera input and every canonical-perception stage together.

Usage:
  ./scripts/record_real_camera_diagnostic.sh [LABEL] [DURATION_SEC]

Defaults:
  LABEL=stationary
  DURATION_SEC=15
  output root=$HOME/kookmin_camera_diagnostics

Start exactly one xycar_camera_perception node before running this script.
EOF
}

if [[ "${1:-}" == "-h" || "${1:-}" == "--help" ]]; then
  usage
  exit 0
fi

label="${1:-stationary}"
duration_sec="${2:-15}"
if [[ ! "$label" =~ ^[A-Za-z0-9_-]+$ ]]; then
  echo "ERROR: LABEL may contain only letters, numbers, _ and -." >&2
  exit 2
fi
if [[ ! "$duration_sec" =~ ^[1-9][0-9]*$ ]]; then
  echo "ERROR: DURATION_SEC must be a positive integer." >&2
  exit 2
fi

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
project_root="$(cd -- "$script_dir/.." && pwd)"
timestamp="$(date +%Y%m%d_%H%M%S)"
output_root="${CAMERA_DIAGNOSTIC_ROOT:-$HOME/kookmin_camera_diagnostics}"
session_dir="$output_root/${timestamp}_${label}"
bag_dir="$session_dir/diagnostic_bag"
calibration="$project_root/xycar_ws/src/xycar_perception/config/wide_camera_fisheye_1280x1024_20260708.yaml"
model="$project_root/xycar_ws/src/xycar_perception/models/kookmin_lane_yolo11n_512.pt"

if ! command -v ros2 >/dev/null 2>&1; then
  echo "ERROR: ros2 is unavailable. Source ROS 2 Humble and the workspace." >&2
  exit 1
fi
if ! ros2 node list | grep -qx '/xycar_camera_perception'; then
  echo "ERROR: /xycar_camera_perception is not running." >&2
  echo "Start the normal rectified or raw-camera recovery launch first." >&2
  exit 1
fi

mkdir -p "$session_dir"

{
  echo "captured_at=$(date --iso-8601=seconds)"
  echo "hostname=$(hostname)"
  echo "label=$label"
  echo "duration_sec=$duration_sec"
  echo "ros_domain_id=${ROS_DOMAIN_ID:-unset}"
  echo "git_commit=$(git -C "$project_root" rev-parse HEAD 2>/dev/null || echo unknown)"
  echo "git_status=$(git -C "$project_root" status --short 2>/dev/null | wc -l) changed entries"
  if [[ -f "$calibration" ]]; then
    echo "calibration_sha256=$(sha256sum "$calibration" | awk '{print $1}')"
  fi
  if [[ -f "$model" ]]; then
    echo "lane_model_sha256=$(sha256sum "$model" | awk '{print $1}')"
  fi
} > "$session_dir/runtime.txt"

ros2 node list > "$session_dir/nodes.txt"
ros2 topic list -t > "$session_dir/topics.txt"
ros2 param dump /xycar_camera_perception > "$session_dir/perception_params.yaml"

for topic in \
  /wide_camera_mjpeg/image_raw/compressed \
  /wide_camera/rect/image_raw \
  /wide_camera/rect/camera_info \
  /perception/rectified_camera_image \
  /perception/yolo_debug_image \
  /perception/debug_image \
  /perception/canonical_road_image \
  /perception/canonical_white_mask \
  /perception/canonical_yellow_mask \
  /perception/canonical_pregeometry_white_mask \
  /perception/canonical_pregeometry_yellow_mask \
  /perception/canonical_pretrack_white_mask \
  /perception/canonical_pretrack_yellow_mask \
  /perception/canonical_valid_mask \
  /perception/canonical_metric_debug \
  /perception/canonical_tracking_debug; do
  {
    echo "===== $topic ====="
    ros2 topic info "$topic" -v || true
  } >> "$session_dir/topic_info.txt" 2>&1
done

echo "Recording $duration_sec seconds to $bag_dir"
set +e
timeout --signal=INT --kill-after=5s "${duration_sec}s" \
  ros2 bag record -o "$bag_dir" \
    /wide_camera_mjpeg/image_raw/compressed \
    /wide_camera/rect/image_raw \
    /wide_camera/rect/camera_info \
    /perception/rectified_camera_image \
    /perception/yolo_debug_image \
    /perception/debug_image \
    /perception/canonical_road_image \
    /perception/canonical_white_mask \
    /perception/canonical_yellow_mask \
    /perception/canonical_pregeometry_white_mask \
    /perception/canonical_pregeometry_yellow_mask \
    /perception/canonical_pretrack_white_mask \
    /perception/canonical_pretrack_yellow_mask \
    /perception/canonical_valid_mask \
    /perception/canonical_metric_debug \
    /perception/canonical_tracking_debug
record_status=$?
set -e

if [[ "$record_status" -ne 0 && "$record_status" -ne 124 ]]; then
  echo "ERROR: ros2 bag record failed with status $record_status" >&2
  exit "$record_status"
fi

ros2 bag info "$bag_dir" > "$session_dir/bag_info.txt"
echo "Diagnostic capture complete: $session_dir"
echo "Move this entire directory to the simulation PC for comparison."
