#!/usr/bin/env bash
set -Eeuo pipefail

SESSION="omx_system"
WORKSPACE="/root/omx_box_project_ws"
LOG_DIR="${WORKSPACE}/log/system"
PORT_NAME="${OMX_PORT_NAME:-/dev/ttyACM0}"
VIDEO_DEVICE="${OMX_VIDEO_DEVICE:-/dev/video0}"

source_ros() {
  # ROS/ament setup scripts reference optional variables that may be unset.
  # Keep strict nounset checking for this script, but not while sourcing them.
  set +u
  source /opt/ros/jazzy/setup.bash
  source /root/ros2_ws/install/setup.bash
  source "${WORKSPACE}/install/setup.bash"
  set -u
}

ros_command() {
  local command="$1"
  printf 'source /opt/ros/jazzy/setup.bash; source /root/ros2_ws/install/setup.bash; source %q; cd %q; exec %s' \
    "${WORKSPACE}/install/setup.bash" "${WORKSPACE}" "${command}"
}

wait_for_topic() {
  local topic="$1"
  local seconds="$2"
  local description="$3"
  local deadline=$((SECONDS + seconds))

  while (( SECONDS < deadline )); do
    if ros2 topic list 2>/dev/null | grep -Fxq "${topic}"; then
      echo "Ready: ${description} (${topic})"
      return 0
    fi
    sleep 1
  done
  echo "ERROR: ${description} did not become ready: ${topic}" >&2
  return 1
}

wait_for_message() {
  local topic="$1"
  local seconds="$2"
  local description="$3"
  local deadline=$((SECONDS + seconds))

  while (( SECONDS < deadline )); do
    if timeout 2 ros2 topic echo "${topic}" --once >/dev/null 2>&1; then
      echo "Ready: ${description} (${topic})"
      return 0
    fi
  done
  echo "ERROR: no message received from ${description}: ${topic}" >&2
  return 1
}

wait_for_node() {
  local node="$1"
  local seconds="$2"
  local description="$3"
  local deadline=$((SECONDS + seconds))

  while (( SECONDS < deadline )); do
    if ros2 node list 2>/dev/null | grep -Fxq "${node}"; then
      echo "Ready: ${description} (${node})"
      return 0
    fi
    sleep 1
  done
  echo "ERROR: ${description} did not become ready: ${node}" >&2
  return 1
}

new_window() {
  local name="$1"
  local command="$2"
  tmux new-window -d -t "${SESSION}" -n "${name}" "$(ros_command "${command}")"
}

related_processes() {
  ps -eo comm=,args= | awk '
    $1 == "rmw_zenohd" { print; next }
    $1 == "python3" && $0 ~ /\/opt\/ros\/jazzy\/bin\/ros2/ &&
      $0 ~ /(omx_f\.launch\.py|omx_controller\.launch\.py|camera_usb_cam\.launch\.py|yolo_target_bridge\.launch\.py|pick_coordinator\.launch\.py)/ { print }
  '
}

start_system() {
  command -v tmux >/dev/null || {
    echo "ERROR: tmux is not installed in the container; rebuild the Docker image." >&2
    exit 1
  }
  source_ros

  if tmux has-session -t "${SESSION}" 2>/dev/null; then
    echo "ERROR: tmux session '${SESSION}' is already running." >&2
    echo "Use scripts/status_omx_system.sh or stop it first." >&2
    exit 1
  fi

  if [[ -n "$(related_processes)" ]]; then
    echo "ERROR: related ROS processes already exist outside '${SESSION}'." >&2
    related_processes
    echo "Stop the existing processes before starting the integrated system." >&2
    exit 1
  fi

  [[ -e "${PORT_NAME}" ]] || {
    echo "ERROR: robot serial port does not exist: ${PORT_NAME}" >&2
    exit 1
  }
  [[ -e "${VIDEO_DEVICE}" ]] || {
    echo "ERROR: camera device does not exist: ${VIDEO_DEVICE}" >&2
    exit 1
  }
  mkdir -p "${LOG_DIR}"
  tmux new-session -d -s "${SESSION}" -n zenoh \
    "$(ros_command 'ros2 run rmw_zenoh_cpp rmw_zenohd')"
  sleep 2
  tmux has-session -t "${SESSION}" 2>/dev/null || {
    echo "ERROR: zenoh window exited during startup." >&2
    exit 1
  }

  new_window bringup "ros2 launch open_manipulator_bringup omx_f.launch.py start_rviz:=false port_name:=${PORT_NAME}"
  wait_for_message /joint_states 30 "OpenManipulator bringup"

  new_window movej "ros2 launch cyclo_motion_controller_ros omx_controller.launch.py controller_type:=movej start_interactive_marker:=false config_file:=${WORKSPACE}/docker/config/omx_config_physical.yaml"
  wait_for_node /omx_movej_controller 20 "MoveJ controller"

  new_window camera "ros2 launch open_manipulator_bringup camera_usb_cam.launch.py name:=camera1 video_device:=${VIDEO_DEVICE}"
  wait_for_message /camera1/image_raw 20 "USB camera"

  new_window coordinator "ros2 launch omx_box_control pick_coordinator.launch.py"
  wait_for_node /pick_coordinator 15 "pick coordinator"

  new_window target "ros2 launch omx_box_control yolo_target_bridge.launch.py"
  wait_for_node /yolo_target_bridge 15 "YOLO target bridge"

  new_window rviz "rviz2"
  new_window monitor "ros2 topic echo /pick_coordinator/status"
  tmux select-window -t "${SESSION}:monitor"

  echo
  echo "OMX system is ready with the YOLO target bridge. No robot motion has been requested."
  echo "View logs: docker exec -it omx_box_project tmux attach -t ${SESSION}"
  echo "Begin staging only after inspection:"
  echo "  docker exec -it omx_box_project bash -lc 'source /opt/ros/jazzy/setup.bash; source /root/ros2_ws/install/setup.bash; source ${WORKSPACE}/install/setup.bash; ros2 service call /pick_coordinator/start std_srvs/srv/Trigger \"{}\"'"
}

status_system() {
  if ! tmux has-session -t "${SESSION}" 2>/dev/null; then
    echo "OMX tmux session is not running."
    exit 1
  fi
  tmux list-windows -t "${SESSION}" -F '#{window_index}:#{window_name} #{?window_active,(active),}'
  echo
  source_ros
  for node in /omx_movej_controller /pick_coordinator; do
    if ros2 node list 2>/dev/null | grep -Fxq "${node}"; then
      echo "OK      ${node}"
    else
      echo "MISSING ${node}"
    fi
  done
  if ros2 node list 2>/dev/null | grep -Fxq /yolo_target_bridge; then
    echo "OK      target source"
  else
    echo "MISSING target source"
  fi
  if ros2 topic list 2>/dev/null | grep -Fxq /joint_states; then
    echo "OK      /joint_states"
  else
    echo "MISSING /joint_states"
  fi
  if ros2 topic list 2>/dev/null | grep -Fxq /camera1/image_raw; then
    echo "OK      /camera1/image_raw"
  else
    echo "MISSING /camera1/image_raw"
  fi
}

stop_system() {
  if tmux has-session -t "${SESSION}" 2>/dev/null; then
    tmux kill-session -t "${SESSION}"
    echo "Stopped OMX tmux session '${SESSION}'."
  else
    echo "OMX tmux session was not running."
  fi
}

case "${1:-}" in
  start) start_system ;;
  status) status_system ;;
  stop) stop_system ;;
  attach) exec tmux attach -t "${SESSION}" ;;
  *)
    echo "Usage: $0 {start|status|stop|attach}" >&2
    exit 2
    ;;
esac
