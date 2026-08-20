#!/usr/bin/env bash

set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "$(readlink -f "${BASH_SOURCE[0]}")")" && pwd)"
WORKSPACE="$(cd -- "$SCRIPT_DIR/../../.." && pwd)"
BAG_INPUT="${1:-${YELLOW_COUNT_BAG:-}}"
RATE="${YELLOW_COUNT_BAG_RATE:-0.35}"
START_OFFSET="${YELLOW_COUNT_BAG_START_OFFSET:-287.0}"
CACHE_DIR="/tmp/xycar_yellow_count_bag_cache"
DISPLAY_ENABLED="${YELLOW_COUNT_DISPLAY_ENABLED:-true}"

if [[ -z "$BAG_INPUT" ]]; then
  echo "usage: $0 BAG_PATH" >&2
  echo "or set YELLOW_COUNT_BAG=PATH" >&2
  exit 2
fi
if [[ ! -e "$BAG_INPUT" ]]; then
  echo "bag not found: $BAG_INPUT" >&2
  exit 2
fi

set +u
source /opt/ros/humble/setup.bash
source "$WORKSPACE/install/setup.bash"
set -u
export PYTHONPATH="$WORKSPACE/src/xycar_map_nav${PYTHONPATH:+:$PYTHONPATH}"

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
    CACHE_DIR="/tmp/xycar_yellow_count_bag_db3"
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
declare -a CHILD_PIDS=()

cleanup() {
  local pid
  trap - EXIT INT TERM
  for pid in "${CHILD_PIDS[@]:-}"; do
    if [[ -n "$pid" ]] && kill -0 "$pid" 2>/dev/null; then
      kill -INT -- "-$pid" 2>/dev/null || kill -INT "$pid" 2>/dev/null || true
    fi
  done
}
trap cleanup EXIT INT TERM

setsid python3 \
  "$WORKSPACE/src/xycar_map_nav/xycar_map_nav/yellow_count_bag_viewer.py" \
  --ros-args \
  -p display_enabled:="$DISPLAY_ENABLED" &
VIEWER_PID="$!"
CHILD_PIDS+=("$VIEWER_PID")

sleep 1
setsid ros2 launch \
  "$WORKSPACE/src/xycar_map_nav/launch/yellow_count_semantic_review.launch.py" &
CHILD_PIDS+=("$!")

sleep 2
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
echo " YELLOW_COUNT W1-MODEL BEV YELLOW REVIEW (no steering)"
echo " SPACE: pause/play | M: manual arm/reset | N: next message | Q: quit"
echo " W1-model yellow BEV inference starts after LEFT_4 ABSENT 2/2."
echo "============================================================"

wait "$VIEWER_PID"
