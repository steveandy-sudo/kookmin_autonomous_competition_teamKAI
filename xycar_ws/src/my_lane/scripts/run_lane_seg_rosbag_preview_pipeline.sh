#!/usr/bin/env bash
set -eo pipefail

WORKSPACE="${KOOKMIN_TY_WORKSPACE:-/home/xytron/kookmin_ty/simulation_latest}"

source /opt/ros/humble/setup.bash
source "${WORKSPACE}/install/setup.bash"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-7}"
unset ROS_NAMESPACE

# Kept as a compatibility entry point. The live and rosbag pipelines now use
# the packaged ONNX model and do not activate an external Python environment.
exec ros2 launch my_lane lane_seg_canonical_perception.launch.py "$@"
