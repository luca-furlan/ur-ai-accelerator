#!/usr/bin/env bash

set -euo pipefail

if ! command -v sudo >/dev/null 2>&1; then
  echo "sudo is required to run this script." >&2
  exit 1
fi

sudo apt update
sudo apt install -y \
  curl \
  gnupg2 \
  lsb-release \
  ca-certificates \
  software-properties-common

sudo add-apt-repository -y universe

curl -fsSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /tmp/ros.key
sudo install -m 0644 /tmp/ros.key /usr/share/keyrings/ros-archive-keyring.gpg
rm /tmp/ros.key

ARCH="$(dpkg --print-architecture)"
CODENAME="$( . /etc/os-release && echo "$UBUNTU_CODENAME" )"
echo "deb [arch=${ARCH} signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu ${CODENAME} main" | sudo tee /etc/apt/sources.list.d/ros2.list >/dev/null

sudo apt update

sudo apt install -y \
  ros-humble-ros-base \
  python3-colcon-common-extensions \
  ros-humble-cv-bridge \
  ros-humble-image-transport \
  ros-humble-camera-info-manager \
  ros-humble-diagnostic-updater \
  ros-humble-image-publisher \
  ros-humble-image-tools \
  ros-humble-backward-ros \
  nlohmann-json3-dev

mkdir -p "${HOME}/ros2_ws/src"
cd "${HOME}/ros2_ws/src"

if [ ! -d OrbbecSDK_ROS2 ]; then
  git clone https://github.com/orbbec/OrbbecSDK_ROS2.git
fi

cd OrbbecSDK_ROS2
git fetch origin
git checkout v2-main

sudo bash orbbec_camera/scripts/install_udev_rules.sh

cd "${HOME}/ros2_ws"

if [ -f /opt/ros/humble/setup.bash ]; then
  set +u
  # shellcheck disable=SC1091
  source /opt/ros/humble/setup.bash
  set -u
fi

colcon build
