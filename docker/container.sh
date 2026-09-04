#!/bin/bash
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
COMPOSE_FILE="${SCRIPT_DIR}/docker-compose.yml"
CONTAINER_NAME="omx_box_project"

show_help() {
  echo "Usage: $0 {build|start|enter|stop|logs|status|gui-up|gui-down|gui-status|gui-attach}"
}

gui_build() {
  docker exec "${CONTAINER_NAME}" bash -lc '
    set -e
    source /opt/ros/jazzy/setup.bash
    source /root/ros2_ws/install/setup.bash
    cd /root/omx_box_project_ws/integration/yeongjin_gui/omx
    colcon build --base-paths src --symlink-install --packages-select omx_box_control
  '
}

gui_validate() {
  docker exec "${CONTAINER_NAME}" bash -lc '
    set -e
    source /opt/ros/jazzy/setup.bash
    source /root/ros2_ws/install/setup.bash
    export PATH=/opt/ultralytics-venv/bin:$PATH
    export YOLO_CONFIG_DIR=/tmp/Ultralytics
    mkdir -p /tmp/Ultralytics
    python -c "import cv_bridge, numpy, PyQt5, rclpy, ultralytics; assert int(numpy.__version__.split(chr(46))[0]) < 2; print(\"GUI Python dependencies: OK\")"
    test -f /root/omx_box_project_ws/integration/omx_box_system/models/box_defect_best.pt
    test -f /root/omx_box_project_ws/integration/omx_box_system/calibration/omx_camera_homography_7point.yaml
    echo "GUI model and calibration: OK"
  '
}

case "${1:-}" in
  build)
    docker compose -f "${COMPOSE_FILE}" build
    ;;
  start)
    if [ -n "${DISPLAY:-}" ] && command -v xhost >/dev/null 2>&1; then
      xhost +si:localuser:root >/dev/null || true
    fi
    docker compose -f "${COMPOSE_FILE}" up -d --build
    ;;
  enter)
    docker exec -it "${CONTAINER_NAME}" bash
    ;;
  stop)
    docker compose -f "${COMPOSE_FILE}" down
    if [ -n "${DISPLAY:-}" ] && command -v xhost >/dev/null 2>&1; then
      xhost -si:localuser:root >/dev/null || true
    fi
    ;;
  logs)
    docker compose -f "${COMPOSE_FILE}" logs -f
    ;;
  status)
    docker compose -f "${COMPOSE_FILE}" ps
    ;;
  gui-up)
    if [ -n "${DISPLAY:-}" ] && command -v xhost >/dev/null 2>&1; then
      xhost +si:localuser:root >/dev/null || true
    fi
    docker compose -f "${COMPOSE_FILE}" up -d --build
    gui_build
    gui_validate
    docker exec \
      -e OMX_PORT_NAME="${OMX_PORT_NAME:-auto}" \
      -e UNLOAD_OMX_PORT_NAME="${UNLOAD_OMX_PORT_NAME:-}" \
      -e AUTOMATIC_UNLOAD_OMX="${AUTOMATIC_UNLOAD_OMX:-true}" \
      -e ENABLE_UNLOAD_OMX="${ENABLE_UNLOAD_OMX:-true}" \
      -e ROS_DOMAIN_ID="${GUI_ROS_DOMAIN_ID:-31}" \
      -e OMX_VIDEO_DEVICE="${OMX_VIDEO_DEVICE:-/dev/video0}" \
      -e UNLOAD_VIDEO_DEVICE="${UNLOAD_VIDEO_DEVICE:-/dev/video2}" \
      -e BEAGLE_MODE="${BEAGLE_MODE:-auto}" \
      -e BEAGLE_TRIGGER_HOST="${BEAGLE_TRIGGER_HOST:-}" \
      -e BEAGLE_TRIGGER_PORT="${BEAGLE_TRIGGER_PORT:-8765}" \
      -e BEAGLE_STATUS_PORT="${BEAGLE_STATUS_PORT:-9000}" \
      -e BEAGLE_LOCAL_MISSION_LAUNCH="${BEAGLE_LOCAL_MISSION_LAUNCH:-auto}" \
      "${CONTAINER_NAME}" \
      bash "/root/omx_box_project_ws/scripts/omx_gui_system_container.sh" start
    ;;
  gui-down)
    docker exec "${CONTAINER_NAME}" bash "/root/omx_box_project_ws/scripts/omx_gui_system_container.sh" stop
    ;;
  gui-status)
    docker exec "${CONTAINER_NAME}" bash "/root/omx_box_project_ws/scripts/omx_gui_system_container.sh" status
    ;;
  gui-attach)
    docker exec -it "${CONTAINER_NAME}" bash "/root/omx_box_project_ws/scripts/omx_gui_system_container.sh" attach
    ;;
  *)
    show_help
    exit 1
    ;;
esac
