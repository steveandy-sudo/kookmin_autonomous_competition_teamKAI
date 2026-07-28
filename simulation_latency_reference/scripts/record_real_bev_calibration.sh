#!/usr/bin/env bash
set -eo pipefail

usage() {
  cat <<'EOF'
Usage:
  record_real_bev_calibration.sh LABEL [DURATION_SEC] [IMAGE_TOPIC]

Examples:
  ./scripts/record_real_bev_calibration.sh center_on_yellow 15
  ./scripts/record_real_bev_calibration.sh lane_center 15
  ./scripts/record_real_bev_calibration.sh reflection 20 \
    /wide_camera_mjpeg/image_raw/compressed

Environment:
  ROS_DOMAIN_ID        Defaults to 7.
  XYCAR_WS             Workspace containing install/setup.bash.
                       Defaults to $HOME/xycar_ws.
  BEV_CALIBRATION_ROOT Output parent directory.
                       Defaults to $HOME/kookmin_bev_calibration_2m.
EOF
}

if [[ $# -lt 1 || $# -gt 3 ]]; then
  usage >&2
  exit 2
fi

label="$1"
duration_sec="${2:-15}"
requested_image_topic="${3:-}"

if [[ ! "$label" =~ ^[A-Za-z0-9_-]+$ ]]; then
  echo "LABEL may contain only letters, numbers, underscores, and hyphens." >&2
  exit 2
fi
if [[ ! "$duration_sec" =~ ^[1-9][0-9]*$ ]]; then
  echo "DURATION_SEC must be a positive integer." >&2
  exit 2
fi

source /opt/ros/humble/setup.bash
workspace="${XYCAR_WS:-$HOME/xycar_ws}"
if [[ -f "$workspace/install/setup.bash" ]]; then
  source "$workspace/install/setup.bash"
fi
set -u
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-7}"

topic_listing="$(ros2 topic list -t)"
has_topic() {
  grep -Fq "$1 [" <<<"$topic_listing"
}

if [[ -n "$requested_image_topic" ]]; then
  if ! has_topic "$requested_image_topic"; then
    echo "Requested image topic is not active: $requested_image_topic" >&2
    echo "$topic_listing" >&2
    exit 1
  fi
  image_topic="$requested_image_topic"
else
  image_topic=""
  for candidate in \
    /wide_camera_mjpeg/image_raw/compressed \
    /wide_camera/rect/image_raw/compressed \
    /wide_camera/rect/image_raw \
    /image_raw/compressed \
    /image_raw; do
    if has_topic "$candidate"; then
      image_topic="$candidate"
      break
    fi
  done
  if [[ -z "$image_topic" ]]; then
    echo "No supported camera image topic is active." >&2
    echo "$topic_listing" >&2
    exit 1
  fi
fi

topics=("$image_topic")
for candidate in \
  /wide_camera/rect/camera_info \
  /wide_camera/camera_info \
  /camera_info \
  /tf_static; do
  if has_topic "$candidate"; then
    topics+=("$candidate")
  fi
done

output_root="${BEV_CALIBRATION_ROOT:-$HOME/kookmin_bev_calibration_2m}"
distance_origin="${BEV_DISTANCE_ORIGIN:-camera_lens}"
mark_interval_m="${BEV_MARK_INTERVAL_M:-0.50}"
mkdir -p "$output_root"
timestamp="$(date +%Y%m%d_%H%M%S)"
bag_path="$output_root/${timestamp}_${label}"

echo "ROS_DOMAIN_ID=$ROS_DOMAIN_ID"
echo "image_topic=$image_topic"
echo "duration_sec=$duration_sec"
echo "distance_origin=$distance_origin"
echo "mark_interval_m=$mark_interval_m"
echo "bag_path=$bag_path"
printf 'recording topics:'
printf ' %s' "${topics[@]}"
printf '\n'
echo "Keep the vehicle and camera in the requested pose until recording stops."

set +e
timeout --signal=INT --kill-after=10s "${duration_sec}s" \
  ros2 bag record -o "$bag_path" "${topics[@]}"
record_status=$?
set -e
if [[ $record_status -ne 0 && $record_status -ne 124 && $record_status -ne 130 ]]; then
  echo "ros2 bag record failed with status $record_status" >&2
  exit "$record_status"
fi
if [[ ! -f "$bag_path/metadata.yaml" ]]; then
  echo "Recording did not create metadata.yaml: $bag_path" >&2
  exit 1
fi

{
  echo "label=$label"
  echo "captured_at=$timestamp"
  echo "duration_sec=$duration_sec"
  echo "distance_origin=$distance_origin"
  echo "mark_interval_m=$mark_interval_m"
  echo "ros_domain_id=$ROS_DOMAIN_ID"
  echo "image_topic=$image_topic"
  printf 'topics='
  printf '%s ' "${topics[@]}"
  printf '\n'
  echo
  echo "Active ROS topics at capture start:"
  echo "$topic_listing"
} >"$bag_path/capture_manifest.txt"

ros2 bag info "$bag_path"
echo
echo "Recorded calibration bag: $bag_path"
echo "Inspect it with:"
echo "  python3 scripts/inspect_bev_calibration_bag.py '$bag_path'"
