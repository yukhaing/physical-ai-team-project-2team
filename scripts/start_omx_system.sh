#!/usr/bin/env bash
set -Eeuo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
CONTAINER="omx_box_project"

if docker ps --format '{{.Names}}' | grep -Fxq "${CONTAINER}"; then
  RELATED_PROCESSES="$(docker exec "${CONTAINER}" ps -eo comm=,args= | awk '
    $1 == "rmw_zenohd" { print; next }
    $1 == "python3" && $0 ~ /\/opt\/ros\/jazzy\/bin\/ros2/ &&
      $0 ~ /(omx_f\.launch\.py|omx_controller\.launch\.py|camera_usb_cam\.launch\.py|yolo_target_bridge\.launch\.py|pick_coordinator\.launch\.py)/ { print }
  ')"
  if [[ -n "${RELATED_PROCESSES}" ]]; then
    echo "ERROR: robot-related processes are already running in ${CONTAINER}." >&2
    echo "Stop or inspect them before rebuilding/restarting the integrated system." >&2
    echo "${RELATED_PROCESSES}"
    exit 1
  fi
fi

"${ROOT_DIR}/docker/container.sh" start

if ! docker exec "${CONTAINER}" bash -lc 'command -v tmux' >/dev/null 2>&1; then
  echo "ERROR: the container image does not contain tmux." >&2
  echo "Rebuild with: ${ROOT_DIR}/docker/container.sh build" >&2
  exit 1
fi

docker exec "${CONTAINER}" bash -lc \
  'source /opt/ros/jazzy/setup.bash && source /root/ros2_ws/install/setup.bash && cd /root/omx_box_project_ws && colcon build --packages-select omx_box_control --symlink-install'

docker exec -it \
  -e OMX_PORT_NAME="${OMX_PORT_NAME:-auto}" \
  -e OMX_VIDEO_DEVICE="${OMX_VIDEO_DEVICE:-/dev/video0}" \
  "${CONTAINER}" \
  /root/omx_box_project_ws/scripts/omx_system_container.sh start
