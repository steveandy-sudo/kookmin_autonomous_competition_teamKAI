#!/usr/bin/env bash

set -eo pipefail

usage() {
  cat <<'EOF'
Usage: record_integrated_drive_bag.sh [SESSION] [options]

Record all data needed to reproduce integrated real-drive behavior. This
script never publishes motor commands. Recording continues until Ctrl+C.

Options:
  --output-root PATH   Output root (default: ~/rosbags/integrated_drive)
  --image-rate HZ     Diagnostic image compression rate (default: 5)
  --no-debug-images   Omit rule/lane/object debug images
  --dry-run           Write no bag; print configuration and exit
  -h, --help          Show this help
EOF
}

SESSION_NAME="integrated_drive"
OUTPUT_ROOT="${INTEGRATED_DRIVE_OUTPUT_ROOT:-$HOME/rosbags/integrated_drive}"
IMAGE_RATE_HZ="5.0"
INCLUDE_DEBUG_IMAGES="true"
DRY_RUN="false"

if (($# > 0)) && [[ "$1" != -* ]]; then
  SESSION_NAME="$1"
  shift
fi
while (($# > 0)); do
  case "$1" in
    --output-root)
      OUTPUT_ROOT="$2"
      shift 2
      ;;
    --image-rate)
      IMAGE_RATE_HZ="$2"
      shift 2
      ;;
    --no-debug-images)
      INCLUDE_DEBUG_IMAGES="false"
      shift
      ;;
    --dry-run)
      DRY_RUN="true"
      shift
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      printf 'ERROR: unknown argument: %s\n' "$1" >&2
      usage >&2
      exit 2
      ;;
  esac
done

if [[ ! "$SESSION_NAME" =~ ^[A-Za-z0-9._-]+$ ]]; then
  printf 'ERROR: SESSION may contain only letters, numbers, dot, dash, underscore.\n' >&2
  exit 2
fi
if ! [[ "$IMAGE_RATE_HZ" =~ ^[0-9]+([.][0-9]+)?$ ]]; then
  printf 'ERROR: --image-rate must be a positive number.\n' >&2
  exit 2
fi
if [[ "$IMAGE_RATE_HZ" != *.* ]]; then
  IMAGE_RATE_HZ="${IMAGE_RATE_HZ}.0"
fi

WORKSPACE="${XYCAR_SLAM_WS:-/home/xytron/kookmin_ty/slam_gazebo_controller/xycar_ws}"
REPOSITORY="${XYCAR_SLAM_REPO:-/home/xytron/kookmin_ty/slam_gazebo_controller}"
STAMP="$(date +%Y%m%d_%H%M%S)"
RUN_DIR="$OUTPUT_ROOT/${SESSION_NAME}_${STAMP}"
BAG_DIR="$RUN_DIR/bag"
MANIFEST="$RUN_DIR/run_manifest.yaml"
PARAM_DIR="$RUN_DIR/parameters"
RUN_CONFIG_SOURCE="${XYCAR_HYBRID_RUN_CONFIG_FILE:-/tmp/xycar_hybrid_run_config.yaml}"
RUN_CONFIG_DEST="$RUN_DIR/launch_inputs.yaml"
RECORDER_STARTED_EPOCH="$(date +%s)"

# New matching topics are discovered while recording, so the recorder may be
# started before the driving launch. Raw image topics are deliberately absent;
# only the original MJPEG and low-rate compressed diagnostics are included.
TOPIC_REGEX='^(/wide_camera_mjpeg/image_raw/compressed|/wide_camera_mjpeg/camera_info|/wide_camera/rect/camera_info|/scan|/slam/scan_filtered|/imu|/imu/raw_data|/vehicle/.*|/odom|/slam/odom|/tf|/tf_static|/map|/map_metadata|/map_updates|/pose|/initialpose|/clicked_point|/map_nav/.*|/my_rule/object_detections|/my_rule/start_signal_green|/my_rule/cone_.*|/rule_drive/diagnostics|/rule_drive/connected_yellow_path|/rl/action_applied|/rl/policy_motor_shadow|/rl/policy_debug|/rl/policy_status|/lane_seg/diagnostics|/recording/.*|/xycar_motor|/xycar_motor_shadow|/xycar_motor_bridge/debug|/hybrid/rule_candidate|/hybrid/avoidance_lateral_offset|/hybrid/avoidance_debug|/hybrid_gate/drive_armed|/hybrid_gate/mode|/hybrid_gate/status|/hybrid_gate/diagnostics|/hybrid_gate/xycar_motor_shadow|/yolo_obstacle/stable_counts|/track_calibration/event|/diagnostics|/parameter_events|/rosout)$'

PARAMETER_NODES=(
  /wide_camera
  /lane_seg_lraspp_inference
  /canonical_stanley_pursuit_driver
  /rl_policy_inference
  /my_rule_object_detection_node
  /my_rule_cone_node
  /sequential_hybrid_driver
  /space_drive_gate
  /xycar_waypoint_nav
  /route_scan_localizer
  /global_lane_guard
  /slam_scan_filter
  /slam_toolbox
  /vesc_imu_odom
  /xycar_vesc_driver
  /imu_node
  /xycar_lidar_node
)

mkdir -p "$PARAM_DIR"
AVAILABLE_TOPICS="$(ros2 topic list -t 2>/dev/null || true)"
AVAILABLE_NODES="$(ros2 node list 2>/dev/null || true)"
printf '%s\n' "$AVAILABLE_TOPICS" >"$RUN_DIR/topic_list_at_start.txt"
printf '%s\n' "$AVAILABLE_NODES" >"$RUN_DIR/node_list_at_start.txt"

git_branch="unknown"
git_commit="unknown"
if git -C "$REPOSITORY" rev-parse --is-inside-work-tree >/dev/null 2>&1; then
  git_branch="$(git -C "$REPOSITORY" branch --show-current 2>/dev/null || true)"
  git_commit="$(git -C "$REPOSITORY" rev-parse HEAD 2>/dev/null || true)"
  git -C "$REPOSITORY" status --short >"$RUN_DIR/git_status_at_start.txt"
  git -C "$REPOSITORY" diff --binary >"$RUN_DIR/git_diff_at_start.patch"
fi

cat >"$MANIFEST" <<EOF
session_name: "$SESSION_NAME"
started_at: "$(date --iso-8601=seconds)"
host: "$(hostname)"
workspace: "$WORKSPACE"
repository: "$REPOSITORY"
git_branch: "$git_branch"
git_commit: "$git_commit"
ros_domain_id: "${ROS_DOMAIN_ID:-unset}"
ros_namespace: "${ROS_NAMESPACE:-unset}"
map_yaml: "${MAP_YAML:-unknown}"
waypoints_yaml: "${WAYPOINTS_YAML:-unknown}"
object_model: "${OBJECT_MODEL:-unknown}"
drive_model: "${DRIVE_MODEL:-none-rule-and-global}"
diagnostic_image_rate_hz: $IMAGE_RATE_HZ
include_debug_images: $INCLUDE_DEBUG_IMAGES
launch_inputs_file: "launch_inputs.yaml"
motor_commands_published_by_recorder: false
bag_topic_regex: "$TOPIC_REGEX"
EOF

printf '\n===== INTEGRATED DRIVE RECORDING =====\n'
printf 'session: %s\n' "$SESSION_NAME"
printf 'output:  %s\n' "$RUN_DIR"
printf 'stop:    Ctrl+C when the run is finished\n'
printf 'camera:  original compressed MJPEG\n'
printf 'images:  masks/canonical/debug compressed at %s Hz\n' "$IMAGE_RATE_HZ"
printf 'logic:   lane/global/mission/traffic/cone/object/final command\n'
printf 'motor commands published by recorder: NO\n'

required_now=(
  /wide_camera_mjpeg/image_raw/compressed
  /scan
  /vehicle/vesc_state
)
missing_now=()
for topic in "${required_now[@]}"; do
  if ! grep -Eq "^${topic//\//\\/}[[:space:]]" <<<"$AVAILABLE_TOPICS"; then
    missing_now+=("$topic")
  fi
done
if ((${#missing_now[@]})); then
  printf 'WARNING: not active yet (recording will discover them if started later):\n'
  printf '  %s\n' "${missing_now[@]}"
fi

if [[ "$DRY_RUN" == "true" ]]; then
  printf '\nregex: %s\n' "$TOPIC_REGEX"
  printf 'Dry run complete. No bag was written. Snapshot: %s\n' "$RUN_DIR"
  exit 0
fi

compressor_pid=""
parameter_pid=""
cleanup() {
  status=$?
  trap - EXIT INT TERM
  if [[ -n "$compressor_pid" ]] && kill -0 "$compressor_pid" 2>/dev/null; then
    kill -INT "$compressor_pid" 2>/dev/null || true
    wait "$compressor_pid" 2>/dev/null || true
  fi
  if [[ -n "$parameter_pid" ]] && kill -0 "$parameter_pid" 2>/dev/null; then
    kill "$parameter_pid" 2>/dev/null || true
    wait "$parameter_pid" 2>/dev/null || true
  fi
  ros2 topic list -t >"$RUN_DIR/topic_list_at_end.txt" 2>/dev/null || true
  ros2 node list >"$RUN_DIR/node_list_at_end.txt" 2>/dev/null || true
  cat >>"$MANIFEST" <<EOF
finished_at: "$(date --iso-8601=seconds)"
record_exit_status: $status
EOF
  if [[ -f "$BAG_DIR/metadata.yaml" ]]; then
    printf '\nSaved: %s\n' "$RUN_DIR"
    printf 'Inspect: ros2 bag info %q\n' "$BAG_DIR"
    printf 'Analyze: ros2 run xycar_map_nav analyze_integrated_drive_bag %q\n' "$RUN_DIR"
  else
    printf '\nWARNING: bag metadata was not created: %s\n' "$BAG_DIR" >&2
  fi
  if ((status != 0 && status != 130)); then
    exit "$status"
  fi
  exit 0
}
trap cleanup EXIT INT TERM

# The recorder is often started before the driving launch. Capture each
# relevant node's initial parameters as soon as that node appears.
(
  tar --zstd -cf "$RUN_DIR/source_snapshot.tar.zst" \
    --exclude='*.pt' --exclude='*.pth' --exclude='*.onnx' \
    --exclude='__pycache__' --exclude='.pytest_cache' \
    -C "$WORKSPACE" \
    src/xycar_map_nav src/xycar_rule_drive src/lane_seg_control \
    src/study/my_rule 2>"$RUN_DIR/source_snapshot.err" || true
  default_object_model="$WORKSPACE/src/study/my_rule/models/kookmin_objects_best_20260804.pt"
  if [[ -f "$default_object_model" ]]; then
    sha256sum "$default_object_model" >"$RUN_DIR/model_checksums.txt"
  fi
  while true; do
    current_nodes="$(ros2 node list 2>/dev/null || true)"
    if [[ ! -s "$RUN_CONFIG_DEST" && -s "$RUN_CONFIG_SOURCE" ]]; then
      config_mtime="$(stat -c %Y "$RUN_CONFIG_SOURCE" 2>/dev/null || echo 0)"
      if ((config_mtime >= RECORDER_STARTED_EPOCH)) || \
        grep -Fxq /canonical_stanley_pursuit_driver <<<"$current_nodes"; then
        cp "$RUN_CONFIG_SOURCE" "$RUN_CONFIG_DEST"
      fi
    fi
    for node in "${PARAMETER_NODES[@]}"; do
      safe_name="${node#/}"
      output="$PARAM_DIR/${safe_name//\//_}.yaml"
      if [[ ! -s "$output" ]] && grep -Fxq "$node" <<<"$current_nodes"; then
        timeout 5 ros2 param dump "$node" >"$output" \
          2>"$PARAM_DIR/${safe_name//\//_}.err" || true
      fi
    done
    sleep 2
  done
) >"$RUN_DIR/parameter_capture.log" 2>&1 &
parameter_pid=$!

ros2 run xycar_map_nav diagnostic_image_compressor --ros-args \
  -p max_rate_hz:="$IMAGE_RATE_HZ" \
  -p include_debug_images:="$INCLUDE_DEBUG_IMAGES" \
  >"$RUN_DIR/image_compressor.log" 2>&1 &
compressor_pid=$!

sleep 1
if ! kill -0 "$compressor_pid" 2>/dev/null; then
  printf 'ERROR: diagnostic image compressor failed to start:\n' >&2
  cat "$RUN_DIR/image_compressor.log" >&2
  exit 3
fi
ros2 bag record \
  --storage sqlite3 \
  --compression-mode file \
  --compression-format zstd \
  --compression-threads 2 \
  --compression-queue-size 4 \
  --polling-interval 100 \
  --output "$BAG_DIR" \
  --regex "$TOPIC_REGEX"
