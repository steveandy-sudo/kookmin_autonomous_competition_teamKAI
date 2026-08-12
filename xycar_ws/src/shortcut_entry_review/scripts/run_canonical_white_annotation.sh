#!/usr/bin/env bash
set -eo pipefail

repo=/home/kai/kookmin_autonomous_competition_teamKAI
bag=/home/kai/Downloads/drive-download-20260811T090224Z-1-001

source /opt/ros/humble/setup.bash
source "${repo}/xycar_ws/install/setup.bash"

export ROS_DOMAIN_ID=48
export ROS_LOCALHOST_ONLY=1
export ROS_LOG_DIR=/tmp/kai_canonical_annotation_logs
export KAI_CANONICAL_ANNOTATION_DIR="${repo}/analysis/canonical_white_line_annotations"
export KAI_ANNOTATION_INTERVAL_SEC=0.2
mkdir -p "${ROS_LOG_DIR}"

ros2 launch lane_seg_control lane_seg_lraspp_canonical_only.launch.py \
  model_path:="${repo}/xycar_ws/src/xycar_perception/models/kookmin_lane_lraspp_mbv3s_256x144.pt" \
  image_topic:=/wide_camera_mjpeg/image_raw/compressed \
  use_compressed_image:=true \
  enable_rectify:=true \
  direct_model_rectify_enabled:=true \
  direct_model_rectify_oversample:=3 \
  camera_yaml:="${repo}/xycar_ws/src/xycar_perception/config/wide_camera_fisheye_1280x1024_20260708.yaml" \
  rect_balance:=0.3 \
  max_input_age_sec:=0.0 \
  direct_canonical_enabled:=true \
  canonical_white_fit_enabled:=false \
  publish_intermediate_topics:=true \
  pipeline_qos_depth:=1 \
  cpu_threads:=4 \
  max_output_rate_hz:=15.0 \
  debug_rate_hz:=5.0 \
  > /tmp/kai_canonical_annotation_inference.log 2>&1 &
inference_pid=$!

ros2 bag play "${bag}" \
  --topics /wide_camera_mjpeg/image_raw/compressed \
  --rate 0.5 \
  --clock \
  --disable-keyboard-controls \
  > /tmp/kai_canonical_annotation_bag.log 2>&1 &
bag_pid=$!

cleanup() {
  kill -INT "${bag_pid}" "${inference_pid}" 2>/dev/null || true
  wait "${bag_pid}" "${inference_pid}" 2>/dev/null || true
}
trap cleanup EXIT INT TERM

python3 -u \
  "${repo}/xycar_ws/src/shortcut_entry_review/shortcut_entry_review/canonical_white_annotation_gui.py"
