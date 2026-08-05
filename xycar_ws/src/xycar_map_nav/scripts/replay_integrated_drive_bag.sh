#!/usr/bin/env bash
set -euo pipefail

RUN_DIR="${1:-/home/xytron/rosbags/integrated_drive/rule_cone_avoid_run01_20260805_124808}"
START_OFFSET="${2:-0.0}"
RATE="${3:-0.5}"
VIEWER="${4:-image}"
WORKSPACE="/home/xytron/kookmin_ty/slam_gazebo_controller/xycar_ws"
START_OFFSET_FLOAT="$(awk -v value="$START_OFFSET" 'BEGIN { printf "%.6f", value }')"

if [[ "$VIEWER" != "image" && "$VIEWER" != "rviz" ]]; then
  echo "[ERROR] viewer는 image 또는 rviz여야 합니다: $VIEWER" >&2
  exit 2
fi

if [[ -f "$RUN_DIR/metadata.yaml" ]]; then
  BAG_DIR="$RUN_DIR"
elif [[ -f "$RUN_DIR/bag/metadata.yaml" ]]; then
  BAG_DIR="$RUN_DIR/bag"
else
  echo "[ERROR] metadata.yaml을 찾지 못했습니다: $RUN_DIR" >&2
  exit 2
fi

BAG_START_TIME_NS="$(python3 - "$BAG_DIR/metadata.yaml" <<'PY'
import sys
import yaml

with open(sys.argv[1], encoding="utf-8") as stream:
    metadata = yaml.safe_load(stream)
print(
    metadata["rosbag2_bagfile_information"]["starting_time"]
    ["nanoseconds_since_epoch"]
)
PY
)"

set +u
source /opt/ros/humble/setup.bash
source "$WORKSPACE/install/setup.bash"
set -u
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-7}"
unset ROS_NAMESPACE || true

if ros2 node list 2>/dev/null | grep -Eq '/xycar_vesc_driver|/sequential_hybrid_driver|/space_drive_gate'; then
  echo "[ERROR] 실차 주행 노드가 실행 중입니다. 해당 노드를 종료한 뒤 bag을 재생하세요." >&2
  exit 3
fi

child_pids=()
cleanup() {
  for pid in "${child_pids[@]:-}"; do
    kill "$pid" 2>/dev/null || true
  done
  wait 2>/dev/null || true
}
trap cleanup EXIT
trap 'exit 130' INT TERM

republish() {
  local input_topic="$1"
  local output_topic="$2"
  ros2 run image_transport republish compressed raw --ros-args \
    -r in/compressed:="$input_topic" \
    -r out:="$output_topic" >/tmp/xycar_replay_"${3}".log 2>&1 &
  child_pids+=("$!")
}

republish /wide_camera_mjpeg/image_raw/compressed /replay/camera camera
republish /recording/object_detection_debug/compressed /replay/object_detection objects
republish /recording/canonical_road_image/compressed /replay/canonical canonical
republish /recording/canonical_white_mask/compressed /replay/canonical_white white
republish /recording/canonical_yellow_mask/compressed /replay/canonical_yellow yellow
republish /recording/rule_canonical_debug/compressed /replay/rule_debug rule

ros2 run xycar_map_nav replay_diagnostic_mosaic --ros-args \
  -p start_offset_sec:="$START_OFFSET_FLOAT" \
  -p bag_start_time_ns:="$BAG_START_TIME_NS" \
  >/tmp/xycar_replay_mosaic.log 2>&1 &
mosaic_pid="$!"
child_pids+=("$mosaic_pid")

# The recording contains dynamic TF but not the fixed rear-axle-to-LiDAR TF.
ros2 run tf2_ros static_transform_publisher \
  --x 0.065 --y 0.0 --z 0.0 \
  --roll 0.0 --pitch 0.0 --yaw 0.0 \
  --frame-id rear_axle --child-frame-id laser_frame \
  >/tmp/xycar_replay_static_tf.log 2>&1 &
child_pids+=("$!")

stdbuf -oL ros2 topic echo /hybrid_gate/status --field data 2>/dev/null | \
  awk 'BEGIN { last="" } $0 !~ /^WARNING:/ { if ($0 != last && $0 != "---") { print "[주행상태] " $0; fflush(); last=$0 } }' &
child_pids+=("$!")

QOS_CONFIG="$WORKSPACE/src/xycar_map_nav/config/integrated_drive_replay_qos.yaml"
if [[ "$VIEWER" == "rviz" ]]; then
  RVIZ_CONFIG="$WORKSPACE/src/xycar_map_nav/rviz/integrated_drive_replay.rviz"
  LIBGL_ALWAYS_SOFTWARE=1 QT_XCB_GL_INTEGRATION=none \
    rviz2 -d "$RVIZ_CONFIG" \
    >/tmp/xycar_integrated_replay_rviz.log 2>&1 &
else
  ros2 run image_view image_view --ros-args \
    -r image:=/replay/diagnostic_mosaic \
    >/tmp/xycar_replay_image_view.log 2>&1 &
fi
child_pids+=("$!")

replay_topics=(
  /wide_camera_mjpeg/image_raw/compressed
  /recording/object_detection_debug/compressed
  /recording/canonical_road_image/compressed
  /recording/canonical_white_mask/compressed
  /recording/canonical_yellow_mask/compressed
  /recording/rule_canonical_debug/compressed
  /scan
  /tf
  /my_rule/object_detections
  /my_rule/cone_clusters
  /my_rule/cone_path
  /my_rule/cone_cmd
  /hybrid/avoidance_debug
  /hybrid/avoidance_lateral_offset
  /hybrid_gate/drive_armed
  /hybrid_gate/mode
  /hybrid_gate/status
  /rule_drive/diagnostics
  /hybrid/rule_candidate
)

sleep 2
if ! kill -0 "$mosaic_pid" 2>/dev/null; then
  echo "[ERROR] 2x2 진단 화면 노드가 시작되지 않았습니다." >&2
  tail -n 20 /tmp/xycar_replay_mosaic.log >&2 || true
  exit 4
fi
cat <<EOF
통합 주행 bag 안전 재생
  bag:    $BAG_DIR
  offset: ${START_OFFSET}s
  rate:   ${RATE}x
  viewer: ${VIEWER}

Space : 재생/일시정지
Right : 한 메시지 진행
Up/Down : 재생 속도 증가/감소

/xycar_motor는 재생 목록에서 제외했습니다.
EOF

ros2 bag play "$BAG_DIR" \
  --clock 30 \
  --start-paused \
  --start-offset "$START_OFFSET" \
  --rate "$RATE" \
  --qos-profile-overrides-path "$QOS_CONFIG" \
  --topics "${replay_topics[@]}"
