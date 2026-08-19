#!/usr/bin/env bash

set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "$(readlink -f "${BASH_SOURCE[0]}")")" && pwd)"
WORKSPACE="$(cd -- "$SCRIPT_DIR/../../.." && pwd)"
BAG_INPUT="${SHORTCUT_BAG:-}"
HARDWARE=false
RATE="0.50"
START_OFFSET="287.5"
CACHE_DIR="/tmp/xycar_shortcut_bag_cache"
OUTPUT_DIR="/tmp/shortcut_bag_event_frames"
DISPLAY_ENABLED="${SHORTCUT_DISPLAY_ENABLED:-true}"

usage() {
  echo "usage: $0 [--hardware] [--bag PATH] [--rate N] [--start-offset SEC]"
}

while (($#)); do
  case "$1" in
    --hardware)
      HARDWARE=true
      shift
      ;;
    --bag)
      BAG_INPUT="${2:?--bag needs a path}"
      shift 2
      ;;
    --rate)
      RATE="${2:?--rate needs a number}"
      shift 2
      ;;
    --start-offset)
      START_OFFSET="${2:?--start-offset needs seconds}"
      shift 2
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      echo "unknown option: $1" >&2
      usage >&2
      exit 2
      ;;
  esac
done

if [[ -z "$BAG_INPUT" ]]; then
  echo "bag path required: use --bag PATH or SHORTCUT_BAG=PATH" >&2
  usage >&2
  exit 2
fi
if [[ ! -e "$BAG_INPUT" ]]; then
  echo "bag not found: $BAG_INPUT" >&2
  exit 2
fi

set +u
source /opt/ros/humble/setup.bash

cd "$WORKSPACE"
source "$WORKSPACE/install/setup.bash"
set -u
export PYTHONPATH="$WORKSPACE/src/shortcut_entry_review:$WORKSPACE/src/xycar_map_nav${PYTHONPATH:+:$PYTHONPATH}"

prepare_bag() {
  local input="$1"
  if [[ -d "$input" ]]; then
    printf '%s\n' "$input"
    return
  fi
  if [[ "$input" == *.db3.zstd ]]; then
    mkdir -p "$CACHE_DIR"
    local target="$CACHE_DIR/shortcut_0.db3"
    if [[ ! -s "$target" || "$input" -nt "$target" ]]; then
      echo "[BAG] decompressing once into $target" >&2
      zstd -d -f "$input" -o "$target" >&2
      rm -f "$CACHE_DIR/metadata.yaml"
    fi
    if [[ ! -f "$CACHE_DIR/metadata.yaml" ]]; then
      echo "[BAG] rebuilding metadata" >&2
      ros2 bag reindex "$CACHE_DIR" >&2
    fi
    printf '%s\n' "$CACHE_DIR"
    return
  fi
  if [[ "$input" == *.db3 ]]; then
    local parent
    parent="$(dirname -- "$input")"
    if [[ -f "$parent/metadata.yaml" ]]; then
      printf '%s\n' "$parent"
      return
    fi
    CACHE_DIR="/tmp/xycar_shortcut_bag_db3"
    mkdir -p "$CACHE_DIR"
    ln -sf "$input" "$CACHE_DIR/shortcut_0.db3"
    rm -f "$CACHE_DIR/metadata.yaml"
    ros2 bag reindex "$CACHE_DIR" >&2
    printf '%s\n' "$CACHE_DIR"
    return
  fi
  echo "unsupported bag input: $input" >&2
  exit 2
}

BAG_URI="$(prepare_bag "$BAG_INPUT")"
mkdir -p "$OUTPUT_DIR"
rm -f "$OUTPUT_DIR"/event_*.png "$OUTPUT_DIR/four_event_frames.png"

declare -a CHILD_PIDS=()
cleanup() {
  local pid
  trap - EXIT INT TERM
  for pid in "${CHILD_PIDS[@]:-}"; do
    if [[ -n "$pid" ]] && kill -0 "$pid" 2>/dev/null; then
      kill -INT -- "-$pid" 2>/dev/null || kill -INT "$pid" 2>/dev/null || true
    fi
  done
  sleep 0.3
  if [[ "$HARDWARE" == true ]]; then
    timeout 1s ros2 topic pub --once /xycar_motor std_msgs/msg/Float32MultiArray \
      '{data: [0.0, 0.0]}' >/dev/null 2>&1 || true
  fi
}
trap cleanup EXIT INT TERM

if [[ "$HARDWARE" == true ]]; then
  if [[ ! -e /dev/ttyMOTOR ]]; then
    echo "--hardware requested but /dev/ttyMOTOR does not exist" >&2
    exit 3
  fi
  echo "[HARDWARE] VESC enabled for steering; every published speed is forced to 0"
  setsid ros2 launch xycar_vesc_driver xycar_vesc_driver.launch.py \
    drive_enabled:=true >"/tmp/shortcut_bag_vesc.log" 2>&1 &
  CHILD_PIDS+=("$!")
fi

setsid ros2 launch shortcut_entry_review shortcut_entry_semantic_control.launch.py \
  source_topic:=/wide_camera_mjpeg/image_raw/compressed \
  processing_enabled_topic:=/hybrid/shortcut_processing_enabled \
  candidate_topic:=/hybrid/shortcut_candidate \
  rule_command_topic:=/hybrid/rule_candidate \
  default_enabled:=false \
  handoff_to_rule:=true \
  w1_start_on_intersection:=true \
  w1_steering_start_delay_frames:=0 \
  minimum_entry_progress_m:=0.30 \
  maximum_entry_steering_sec:=1.3 \
  w1_steering_hold_sec:=1.3 \
  entry_direction_hold_enabled:=false \
  entry_steering_rate_limit_cmd_per_sec:=90.0 \
  entry_speed_command:=9.0 \
  show_opencv_windows:=false &
CHILD_PIDS+=("$!")

setsid ros2 run xycar_map_nav shortcut_bag_wheel_gate --ros-args \
  -p hardware_enabled:="$HARDWARE" \
  -p display_enabled:="$DISPLAY_ENABLED" \
  -p save_directory:="$OUTPUT_DIR" &
UI_PID="$!"
CHILD_PIDS+=("$UI_PID")

sleep 3
setsid ros2 bag play "$BAG_URI" \
  --rate "$RATE" \
  --start-offset "$START_OFFSET" \
  --disable-keyboard-controls \
  --topics \
    /wide_camera_mjpeg/image_raw/compressed \
    /my_rule/object_detections &
CHILD_PIDS+=("$!")

echo
echo "============================================================"
echo " W1/W2 MAIN LOGIC BAG REVIEW"
echo " SPACE: pause/play | N or Right: next recorded message | Q: quit"
echo " yellow left_4 events | pink steering start | green RULE resumed"
echo " saved four frames: $OUTPUT_DIR/four_event_frames.png"
echo "============================================================"

wait "$UI_PID"
