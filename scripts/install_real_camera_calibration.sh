#!/usr/bin/env bash
set -euo pipefail

usage() {
  cat <<'EOF'
Install the measured Kookmin Xycar camera calibration into the real workspace.

Usage:
  ./scripts/install_real_camera_calibration.sh [REAL_XYCAR_WS]

Default REAL_XYCAR_WS: $HOME/xycar_ws

The existing calibration is timestamp-backed up before replacement. After the
copy, rebuild app_wide_camera_calib in the real workspace.
EOF
}

if [[ "${1:-}" == "-h" || "${1:-}" == "--help" ]]; then
  usage
  exit 0
fi

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
project_root="$(cd -- "$script_dir/.." && pwd)"
real_ws="${1:-$HOME/xycar_ws}"
source_yaml="$project_root/src/xycar_perception/config/wide_camera_fisheye_1280x1024_20260708.yaml"
target_dir="$real_ws/src/xycar_application/app_wide_camera_calib/config"
target_yaml="$target_dir/wide_camera_fisheye_1280x1024.yaml"
expected_sha="0bc9b224e8105b7c9da2f280097d5f6ea75395482f0c504d288745cbd5a25dfe"

if [[ ! -f "$source_yaml" ]]; then
  echo "ERROR: packaged calibration is missing: $source_yaml" >&2
  exit 1
fi

actual_source_sha="$(sha256sum "$source_yaml" | awk '{print $1}')"
if [[ "$actual_source_sha" != "$expected_sha" ]]; then
  echo "ERROR: packaged calibration checksum mismatch" >&2
  echo "  expected: $expected_sha" >&2
  echo "  actual:   $actual_source_sha" >&2
  exit 1
fi

if [[ ! -d "$target_dir" ]]; then
  echo "ERROR: real camera package was not found: $target_dir" >&2
  echo "Pass the real xycar_ws path as the first argument." >&2
  exit 1
fi

if [[ -f "$target_yaml" ]]; then
  if cmp -s "$source_yaml" "$target_yaml"; then
    echo "Calibration is already current: $target_yaml"
    exit 0
  fi

  backup_yaml="${target_yaml}.backup-$(date +%Y%m%d-%H%M%S)"
  cp --preserve=mode,timestamps "$target_yaml" "$backup_yaml"
  echo "Backed up previous calibration: $backup_yaml"
fi

install -m 0644 "$source_yaml" "$target_yaml"
actual_target_sha="$(sha256sum "$target_yaml" | awk '{print $1}')"
if [[ "$actual_target_sha" != "$expected_sha" ]]; then
  echo "ERROR: installed calibration checksum mismatch: $target_yaml" >&2
  exit 1
fi

echo "Installed measured calibration: $target_yaml"
echo "SHA-256: $actual_target_sha"
echo
echo "Rebuild and source it with:"
echo "  cd '$real_ws'"
echo "  source /opt/ros/humble/setup.bash"
echo "  colcon build --packages-select app_wide_camera_calib --symlink-install"
echo "  source install/setup.bash"
