#!/usr/bin/env bash
set -eo pipefail

if [[ "${EUID}" -eq 0 ]]; then
  as_root=()
else
  as_root=(sudo)
fi

"${as_root[@]}" apt-get update
"${as_root[@]}" apt-get install -y curl lsb-release gnupg
"${as_root[@]}" curl -fsSL \
  https://packages.osrfoundation.org/gazebo.gpg \
  --output /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg

architecture="$(dpkg --print-architecture)"
codename="$(lsb_release -cs)"
repository="deb [arch=${architecture} signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] https://packages.osrfoundation.org/gazebo/ubuntu-stable ${codename} main"
echo "${repository}" | "${as_root[@]}" tee \
  /etc/apt/sources.list.d/gazebo-stable.list >/dev/null

"${as_root[@]}" apt-get update
"${as_root[@]}" apt-get install -y \
  gz-harmonic \
  ros-humble-ros-gzharmonic \
  python3-tk \
  python3-pil.imagetk

gz sim --versions
source /opt/ros/humble/setup.bash
set -u
ros2 pkg prefix ros_gz_bridge
