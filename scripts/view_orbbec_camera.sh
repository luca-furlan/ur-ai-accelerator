#!/usr/bin/env bash

set -euo pipefail

# Usage: ./view_orbbec_camera.sh [image_topic] [viewer]
# image_topic default: /camera/color/image_raw
# viewer options: rqt (default), showimage

IMAGE_TOPIC="${1:-/camera/color/image_raw}"
VIEWER="${2:-rqt}"
DISPLAY_VALUE="${DISPLAY:-:1}"

if [ ! -f /opt/ros/humble/setup.bash ]; then
  echo "ROS 2 Humble non trovato in /opt/ros/humble" >&2
  exit 1
fi

# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash

if [ -f "${HOME}/ros2_ws/install/setup.bash" ]; then
  # shellcheck disable=SC1091
  source "${HOME}/ros2_ws/install/setup.bash"
else
  echo "Attenzione: ~/ros2_ws/install/setup.bash non trovato. Assicurarsi di aver compilato il workspace." >&2
fi

case "${VIEWER}" in
  rqt)
    exec env DISPLAY="${DISPLAY_VALUE}" ros2 run rqt_image_view rqt_image_view
    ;;
  showimage)
    exec env DISPLAY="${DISPLAY_VALUE}" ros2 run image_tools showimage --ros-args -r image:="${IMAGE_TOPIC}"
    ;;
  *)
    echo "Viewer non supportato: ${VIEWER}. Usa 'rqt' o 'showimage'." >&2
    exit 1
    ;;
 esac
