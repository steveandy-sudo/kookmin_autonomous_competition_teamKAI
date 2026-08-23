#!/usr/bin/env bash

set -eo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "$(readlink -f "${BASH_SOURCE[0]}")")" && pwd)"
WORKSPACE="$(cd -- "$SCRIPT_DIR/../../.." && pwd)"

# Build from ROS Humble only.  This prevents a setup sourced in the caller's
# terminal from silently turning ~/xycar_ws into an underlay.
unset AMENT_PREFIX_PATH COLCON_PREFIX_PATH CMAKE_PREFIX_PATH
unset PYTHONPATH LD_LIBRARY_PATH PKG_CONFIG_PATH ROS_PACKAGE_PATH
unset _colcon_cd_root
clean_workspace_path=""
IFS=: read -r -a path_entries <<< "${PATH:-}"
for path_entry in "${path_entries[@]}"; do
  case "$path_entry" in
    /home/xytron/*/install/*|/home/xytron/*/install_*/*) continue ;;
  esac
  clean_workspace_path="${clean_workspace_path:+$clean_workspace_path:}$path_entry"
done
export PATH="$clean_workspace_path"
unset clean_workspace_path path_entries path_entry
set +u
source /opt/ros/humble/setup.bash
set -u

cd "$WORKSPACE"
colcon --log-base "$WORKSPACE/log_xycar_only" build \
  --build-base "$WORKSPACE/build_xycar_only" \
  --install-base "$WORKSPACE/install_xycar_only" \
  --symlink-install \
  "$@"

echo
echo "[완료] xycar_ws 단독 빌드: $WORKSPACE/install_xycar_only"
