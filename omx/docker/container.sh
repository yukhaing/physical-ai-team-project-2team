#!/bin/bash
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
COMPOSE_FILE="${SCRIPT_DIR}/docker-compose.yml"
CONTAINER_NAME="omx_box_project"
STACK_DIR="/tmp/omx_gui_stack"
STACK_LOG_DIR="/root/omx_box_project_ws/logs/gui_stack"
ZENOHD_PID_FILE="/tmp/omx_zenohd.pid"

show_help() {
  echo "Usage: $0 {build|start|enter|stop|logs|status|gui-up|gui-down|gui-status}"
}

start_container() {
  if [ -n "${DISPLAY:-}" ] && command -v xhost >/dev/null 2>&1; then
    xhost +si:localuser:root >/dev/null || true
  fi
  docker compose -f "${COMPOSE_FILE}" up -d --build
}

run_in_container() {
  docker exec "${CONTAINER_NAME}" bash -lc "$1"
}

ensure_ultralytics_venv() {
  run_in_container "
    /opt/ultralytics-venv/bin/python - <<'PY'
import importlib
import sys

checks = {
    'numpy': lambda m: int(m.__version__.split('.', 1)[0]) < 2,
    'scipy': lambda m: True,
    'PyQt5': lambda m: True,
}

for name, validator in checks.items():
    try:
        module = importlib.import_module(name)
    except Exception:
        sys.exit(1)
    if not validator(module):
        sys.exit(1)
PY
  " && return

  echo "repairing /opt/ultralytics-venv for ROS compatibility"
  run_in_container "
    /opt/ultralytics-venv/bin/python -m pip install --no-cache-dir 'numpy<2' scipy PyQt5
  "
}

start_component() {
  local name="$1"
  local command="$2"
  local pid_file="${STACK_DIR}/${name}.pid"
  local log_file="${STACK_LOG_DIR}/${name}.log"

  run_in_container "mkdir -p '${STACK_DIR}' '${STACK_LOG_DIR}'"
  run_in_container "
    if [ -f '${pid_file}' ] && kill -0 \$(cat '${pid_file}') 2>/dev/null; then
      echo '${name} already running'
      exit 0
    fi
    rm -f '${pid_file}'
    source /opt/ros/jazzy/setup.bash
    source /root/ros2_ws/install/setup.bash
    if [ -f /root/omx_box_project_ws/install/setup.bash ]; then
      source /root/omx_box_project_ws/install/setup.bash
    fi
    export PATH=/opt/ultralytics-venv/bin:\$PATH
    nohup bash -lc \"${command}\" >'${log_file}' 2>&1 &
    echo \$! > '${pid_file}'
    echo started '${name}' pid=\$(cat '${pid_file}')
  "
}

stop_component() {
  local name="$1"
  local pid_file="${STACK_DIR}/${name}.pid"

  run_in_container "
    if [ ! -f '${pid_file}' ]; then
      echo '${name} not running'
      exit 0
    fi
    pid=\$(cat '${pid_file}')
    if kill -0 \"\$pid\" 2>/dev/null; then
      kill \"\$pid\" || true
      sleep 1
      if kill -0 \"\$pid\" 2>/dev/null; then
        kill -9 \"\$pid\" || true
      fi
      echo stopped '${name}' pid=\$pid
    else
      echo '${name} stale pid=\$pid'
    fi
    rm -f '${pid_file}'
  "
}

status_component() {
  local name="$1"
  local pid_file="${STACK_DIR}/${name}.pid"

  run_in_container "
    if [ -f '${pid_file}' ] && kill -0 \$(cat '${pid_file}') 2>/dev/null; then
      echo '${name}: running pid='\"\$(cat '${pid_file}')\"
    else
      echo '${name}: stopped'
    fi
  "
}

gui_up() {
  start_container
  ensure_ultralytics_venv
  start_component \
    "zenohd" \
    "exec ros2 run rmw_zenoh_cpp rmw_zenohd"
  sleep 2
  start_component \
    "omx_bringup" \
    "exec ros2 launch open_manipulator_bringup omx_f.launch.py start_rviz:=false port_name:=/dev/ttyACM0"
  sleep 2
  start_component \
    "movej_controller" \
    "exec ros2 launch cyclo_motion_controller_ros omx_controller.launch.py controller_type:=movej start_interactive_marker:=false config_file:=/root/omx_box_project_ws/docker/config/omx_config_physical.yaml"
  sleep 2
  start_component \
    "camera" \
    "exec ros2 launch open_manipulator_bringup camera_usb_cam.launch.py name:=camera1 video_device:=/dev/video0"
  sleep 2
  start_component \
    "integrated_console" \
    "exec ros2 launch omx_box_control integrated_console.launch.py"
  echo "GUI stack started"
  echo "Use '$0 gui-status' to inspect each component"
}

gui_down() {
  stop_component "integrated_console"
  stop_component "camera"
  stop_component "movej_controller"
  stop_component "omx_bringup"
  stop_component "zenohd"
}

gui_status() {
  status_component "zenohd"
  status_component "omx_bringup"
  status_component "movej_controller"
  status_component "camera"
  status_component "integrated_console"
}

case "${1:-}" in
  build)
    docker compose -f "${COMPOSE_FILE}" build
    ;;
  start)
    start_container
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
    gui_up
    ;;
  gui-down)
    gui_down
    ;;
  gui-status)
    gui_status
    ;;
  *)
    show_help
    exit 1
    ;;
esac
