#!/usr/bin/env bash
set -Eeuo pipefail

SESSION="omx_gui_system"
LEGACY_SESSION="omx_system"
ROOT_WORKSPACE="/root/omx_box_project_ws"
GUI_WORKSPACE="${ROOT_WORKSPACE}/integration/yeongjin_gui/omx"
BEAGLE_WORKSPACE="${ROOT_WORKSPACE}/integration/yeongjin_gui/Beagle_Lidar_and_AStar"
# Reuse the established Roboid environment; the A* folder contains source and
# calibrated map data, not a separate virtual environment.
BEAGLE_PYTHON="${ROOT_WORKSPACE}/integration/yeongjin_gui/Beagle_mobile_robot/.venv/bin/python"
BEAGLE_MISSION="${BEAGLE_WORKSPACE}/scripts/10_shuttle_mission.py"
LOG_DIR="${ROOT_WORKSPACE}/integration/yeongjin_gui/runtime/logs"
PORT_NAME="${OMX_PORT_NAME:-auto}"
UNLOAD_PORT_NAME="${UNLOAD_OMX_PORT_NAME:-}"
AUTOMATIC_UNLOAD_OMX="${AUTOMATIC_UNLOAD_OMX:-false}"
ENABLE_UNLOAD_OMX="${ENABLE_UNLOAD_OMX:-false}"
DEFAULT_LOADING_OMX_PORT="/dev/serial/by-id/usb-ROBOTIS_OpenRB-150_CAD761565157375037202020FF0D022B-if00"
DEFAULT_UNLOAD_OMX_PORT="/dev/serial/by-id/usb-ROBOTIS_OpenRB-150_CDC2589C5157375037202020FF0F0C0D-if00"
VIDEO_DEVICE="${OMX_VIDEO_DEVICE:-/dev/video0}"
BEAGLE_MODE="${BEAGLE_MODE:-auto}"
BEAGLE_TRIGGER_HOST="${BEAGLE_TRIGGER_HOST:-}"
BEAGLE_TRIGGER_PORT="${BEAGLE_TRIGGER_PORT:-8765}"
BEAGLE_STATUS_PORT="${BEAGLE_STATUS_PORT:-9000}"
# Auto-start only when a dedicated Robomation receiver is present, preventing
# roboid from probing OMX's serial port when Beagle is disconnected.
BEAGLE_LOCAL_MISSION_LAUNCH="${BEAGLE_LOCAL_MISSION_LAUNCH:-auto}"
BEAGLE_LOCAL_MISSION_ENABLED=false
BEAGLE_LOCAL_PORT_NAME=""

resolve_omx_ports() {
  local requested="${PORT_NAME}"
  local candidates=()
  local remaining=()
  local candidate

  shopt -s nullglob
  candidates=(/dev/serial/by-id/usb-ROBOTIS_OpenRB-150_*-if00)
  shopt -u nullglob

  case "${ENABLE_UNLOAD_OMX,,}" in
    true) ;;
    false) UNLOAD_PORT_NAME="" ;;
    *)
      echo "ERROR: ENABLE_UNLOAD_OMX must be true or false." >&2
      exit 1
      ;;
  esac

  if [[ -z "${requested}" || "${requested}" == "auto" ]]; then
    if (( ${#candidates[@]} == 1 )); then
      PORT_NAME="${candidates[0]}"
    elif [[ -n "${UNLOAD_PORT_NAME}" ]]; then
      for candidate in "${candidates[@]}"; do
        if [[ "$(readlink -f "${candidate}")" != "$(readlink -f "${UNLOAD_PORT_NAME}")" ]]; then
          remaining+=("${candidate}")
        fi
      done
      if (( ${#remaining[@]} == 1 )); then
        PORT_NAME="${remaining[0]}"
      fi
    fi

    if [[ "${PORT_NAME}" == "auto" && -e "${DEFAULT_LOADING_OMX_PORT}" ]]; then
      PORT_NAME="${DEFAULT_LOADING_OMX_PORT}"
    fi
    if [[ "${PORT_NAME}" == "auto" ]]; then
      echo "ERROR: cannot identify the loading OMX among ${#candidates[@]} OpenRB-150 controllers." >&2
      printf '  %s\n' "${candidates[@]}" >&2
      echo "Set OMX_PORT_NAME and UNLOAD_OMX_PORT_NAME explicitly." >&2
      exit 1
    fi
  fi

  if [[ "${ENABLE_UNLOAD_OMX,,}" == "true" && -z "${UNLOAD_PORT_NAME}" &&
        -e "${DEFAULT_UNLOAD_OMX_PORT}" &&
        "$(readlink -f "${PORT_NAME}")" != "$(readlink -f "${DEFAULT_UNLOAD_OMX_PORT}")" ]]; then
    UNLOAD_PORT_NAME="${DEFAULT_UNLOAD_OMX_PORT}"
  fi

  echo "Loading OMX: ${PORT_NAME} -> $(readlink -f "${PORT_NAME}")"
  if [[ -n "${UNLOAD_PORT_NAME}" ]]; then
    echo "Unloading OMX: ${UNLOAD_PORT_NAME} -> $(readlink -f "${UNLOAD_PORT_NAME}")"
  elif [[ -e "${DEFAULT_UNLOAD_OMX_PORT}" ]]; then
    echo "Unloading OMX is isolated from this GUI (ENABLE_UNLOAD_OMX=false)."
  fi
}

resolve_beagle_local_launch() {
  BEAGLE_LOCAL_MISSION_ENABLED=false
  [[ "${BEAGLE_MODE}" == "local" ]] || return 0

  local requested="${BEAGLE_LOCAL_MISSION_LAUNCH,,}"
  local receivers=()
  shopt -s nullglob
  receivers=(/dev/serial/by-id/usb-Robomation_EXPRESS_RECEIVER_*-if00)
  shopt -u nullglob

  case "${requested}" in
    false)
      echo "Beagle local mission disabled by configuration."
      ;;
    auto|true)
      if (( ${#receivers[@]} == 1 )); then
        BEAGLE_LOCAL_MISSION_ENABLED=true
        BEAGLE_LOCAL_PORT_NAME="${receivers[0]}"
        echo "Detected Beagle receiver: ${receivers[0]} -> $(readlink -f "${receivers[0]}")"
      elif [[ "${requested}" == "true" ]]; then
        echo "ERROR: expected exactly one Robomation EXPRESS RECEIVER, found ${#receivers[@]}." >&2
        exit 1
      else
        echo "Beagle receiver not detected; local mission remains disabled."
      fi
      ;;
    *)
      echo "ERROR: BEAGLE_LOCAL_MISSION_LAUNCH must be auto, true, or false." >&2
      exit 1
      ;;
  esac
}

source_ros() {
  set +u
  source /opt/ros/jazzy/setup.bash
  source /root/ros2_ws/install/setup.bash
  source "${GUI_WORKSPACE}/install/setup.bash"
  set -u
  export PATH="/opt/ultralytics-venv/bin:${PATH}"
  export YOLO_CONFIG_DIR="/tmp/Ultralytics"
  export BEAGLE_MODE BEAGLE_TRIGGER_HOST BEAGLE_TRIGGER_PORT BEAGLE_STATUS_PORT
  export BEAGLE_LOCAL_MISSION_LAUNCH
  export AUTOMATIC_UNLOAD_OMX
}

ros_command() {
  local command="$1"
  printf 'source /opt/ros/jazzy/setup.bash; source /root/ros2_ws/install/setup.bash; source %q; export PATH=/opt/ultralytics-venv/bin:$PATH; export YOLO_CONFIG_DIR=/tmp/Ultralytics; export BEAGLE_MODE=%q BEAGLE_TRIGGER_HOST=%q BEAGLE_TRIGGER_PORT=%q BEAGLE_STATUS_PORT=%q BEAGLE_LOCAL_MISSION_LAUNCH=%q BEAGLE_LOCAL_PORT_NAME=%q AUTOMATIC_UNLOAD_OMX=%q; cd %q; exec %s' \
    "${GUI_WORKSPACE}/install/setup.bash" "${BEAGLE_MODE}" \
    "${BEAGLE_TRIGGER_HOST}" "${BEAGLE_TRIGGER_PORT}" "${BEAGLE_STATUS_PORT}" "${BEAGLE_LOCAL_MISSION_ENABLED}" "${BEAGLE_LOCAL_PORT_NAME}" \
    "${AUTOMATIC_UNLOAD_OMX}" "${GUI_WORKSPACE}" "${command}"
}

wait_for_message() {
  local topic="$1" seconds="$2" description="$3"
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
  local node="$1" seconds="$2" description="$3"
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

wait_for_beagle_ready() {
  local deadline=$((SECONDS + 30)) message
  while (( SECONDS < deadline )); do
    message="$(timeout 3 ros2 topic echo /beagle/status std_msgs/msg/String --once 2>/dev/null || true)"
    if [[ "${message}" == *'"state":"idle"'* ]]; then
      echo "Ready: local Beagle mission is waiting at the receiving zone"
      return 0
    fi
  done
  echo "ERROR: local Beagle mission did not publish idle status." >&2
  return 1
}

new_window() {
  local name="$1" command="$2"
  tmux new-window -d -t "${SESSION}" -n "${name}" "$(ros_command "${command}")"
}

related_processes() {
  local gui_domain="${ROS_DOMAIN_ID:-0}"
  local pid process_domain command

  while IFS= read -r pid; do
    process_domain="$(
      tr '\0' '\n' <"/proc/${pid}/environ" 2>/dev/null |
        awk -F= '$1 == "ROS_DOMAIN_ID" { print $2; exit }' || true
    )"
    process_domain="${process_domain:-0}"
    [[ "${process_domain}" == "${gui_domain}" ]] || continue
    command="$(ps -p "${pid}" -o comm=,args= 2>/dev/null || true)"
    [[ -n "${command}" ]] && echo "${command}"
  done < <(
    ps -eo pid=,comm=,args= | awk '
      $2 == "rmw_zenohd" { print $1; next }
      $2 == "python3" && $0 ~ /\/opt\/ros\/jazzy\/bin\/ros2/ &&
        $0 ~ /(omx_f\.launch\.py|omx_controller\.launch\.py|unload_omx_system\.launch\.py|camera_usb_cam\.launch\.py|integrated_console\.launch\.py|pick_coordinator\.launch\.py|yolo_target_bridge\.launch\.py)/ { print $1 }
    '
  )
}

start_system() {
  command -v tmux >/dev/null || { echo "ERROR: tmux is missing; rebuild the image." >&2; exit 1; }
  [[ -x /opt/ultralytics-venv/bin/python ]] || { echo "ERROR: Ultralytics environment is missing; rebuild the image." >&2; exit 1; }
  [[ -f "${GUI_WORKSPACE}/install/setup.bash" ]] || { echo "ERROR: GUI workspace is not built: ${GUI_WORKSPACE}" >&2; exit 1; }
  source_ros
  resolve_omx_ports
  resolve_beagle_local_launch

  if tmux has-session -t "${SESSION}" 2>/dev/null; then
    echo "ERROR: GUI session '${SESSION}' is already running." >&2
    exit 1
  fi
  if tmux has-session -t "${LEGACY_SESSION}" 2>/dev/null; then
    echo "ERROR: existing '${LEGACY_SESSION}' session is running. Stop it before GUI startup." >&2
    exit 1
  fi
  if [[ -n "$(related_processes)" ]]; then
    echo "ERROR: robot-related ROS processes already exist." >&2
    related_processes
    exit 1
  fi
  [[ -e "${PORT_NAME}" ]] || { echo "ERROR: robot serial port not found: ${PORT_NAME}" >&2; exit 1; }
  if [[ -n "${UNLOAD_PORT_NAME}" ]]; then
    [[ -e "${UNLOAD_PORT_NAME}" ]] || {
      echo "ERROR: unloading OMX serial port not found: ${UNLOAD_PORT_NAME}" >&2
      exit 1
    }
    if [[ "$(readlink -f "${PORT_NAME}")" == "$(readlink -f "${UNLOAD_PORT_NAME}")" ]]; then
      echo "ERROR: loading and unloading OMX ports resolve to the same controller." >&2
      exit 1
    fi
  elif [[ "${AUTOMATIC_UNLOAD_OMX,,}" == "true" ]]; then
    echo "ERROR: AUTOMATIC_UNLOAD_OMX=true requires UNLOAD_OMX_PORT_NAME." >&2
    exit 1
  fi
  [[ -e "${VIDEO_DEVICE}" ]] || { echo "ERROR: camera device not found: ${VIDEO_DEVICE}" >&2; exit 1; }
  if [[ "${BEAGLE_MODE}" == "local" && "${BEAGLE_LOCAL_MISSION_ENABLED}" == "true" ]]; then
    [[ -x "${BEAGLE_PYTHON}" ]] || {
      echo "ERROR: local Beagle virtual environment is missing: ${BEAGLE_PYTHON}" >&2
      exit 1
    }
    "${BEAGLE_PYTHON}" -c 'import roboid' || {
      echo "ERROR: roboid is not installed in the local Beagle environment." >&2
      exit 1
    }
  fi

  mkdir -p "${LOG_DIR}" /tmp/Ultralytics
  tmux new-session -d -s "${SESSION}" -n zenoh "$(ros_command 'ros2 run rmw_zenoh_cpp rmw_zenohd')"
  trap 'tmux kill-session -t "${SESSION}" 2>/dev/null || true' ERR
  sleep 2
  tmux has-session -t "${SESSION}" 2>/dev/null || { echo "ERROR: zenoh exited during startup." >&2; exit 1; }

  new_window bringup "ros2 launch open_manipulator_bringup omx_f.launch.py start_rviz:=false port_name:=${PORT_NAME}"
  wait_for_message /joint_states 30 "OpenManipulator bringup"
  new_window movej "ros2 launch cyclo_motion_controller_ros omx_controller.launch.py controller_type:=movej start_interactive_marker:=false config_file:=${ROOT_WORKSPACE}/docker/config/omx_config_physical.yaml"
  wait_for_node /omx_movej_controller 20 "MoveJ controller"
  if [[ -n "${UNLOAD_PORT_NAME}" ]]; then
    new_window unload_omx "ros2 launch omx_box_control unload_omx_system.launch.py port_name:=${UNLOAD_PORT_NAME}"
    wait_for_node /unload_omx/unload_coordinator 35 "unloading OMX coordinator"
  fi
  new_window camera "ros2 launch open_manipulator_bringup camera_usb_cam.launch.py name:=camera1 video_device:=${VIDEO_DEVICE}"
  wait_for_message /camera1/image_raw 20 "USB camera"
  new_window console "ros2 launch omx_box_control integrated_console.launch.py"
  wait_for_node /pick_coordinator 20 "pick coordinator"
  wait_for_node /yolo_detection 30 "YOLO detector"
  wait_for_node /beagle_adapter 20 "Beagle adapter"
  wait_for_node /omx_console 20 "operator GUI"
  if [[ "${BEAGLE_MODE}" == "local" &&
        "${BEAGLE_LOCAL_MISSION_ENABLED}" == "true" ]]; then
    new_window beagle "${BEAGLE_PYTHON} ${BEAGLE_MISSION// /\\ } --port-name ${BEAGLE_LOCAL_PORT_NAME} --trigger-port ${BEAGLE_TRIGGER_PORT} --status-host 127.0.0.1 --status-port ${BEAGLE_STATUS_PORT} --output ${LOG_DIR}/beagle_shuttle.csv"
    if ! wait_for_beagle_ready; then
      echo "WARNING: GUI remains available in WAIT_BEAGLE; inspect the beagle tmux window."
    fi
  elif [[ "${BEAGLE_MODE}" == "local" ]]; then
    echo "Beagle local mission launch is disabled; GUI will show 연결 끊김 until a remote status mission connects."
  fi
  new_window monitor "ros2 topic echo /console/status"
  tmux select-window -t "${SESSION}:console"
  trap - ERR

  echo
  echo "OMX GUI system is ready. Robot motion has not been requested."
  echo "Use the GUI buttons to enable and start the pick cycle."
  echo "Logs: docker exec -it omx_box_project tmux attach -t ${SESSION}"
}

status_system() {
  if ! tmux has-session -t "${SESSION}" 2>/dev/null; then
    echo "OMX GUI tmux session is not running."
    exit 1
  fi
  tmux list-windows -t "${SESSION}" -F '#{window_index}:#{window_name} #{?window_active,(active),}'
  echo
  source_ros
  for node in /omx_movej_controller /pick_coordinator /camera_homography_target /yolo_detection /beagle_adapter /sorting_orchestrator /omx_console; do
    if ros2 node list 2>/dev/null | grep -Fxq "${node}"; then echo "OK      ${node}"; else echo "MISSING ${node}"; fi
  done
  if tmux list-windows -t "${SESSION}" -F '#{window_name}' | grep -Fxq unload_omx; then
    for node in /unload_omx/omx_movej_controller /unload_omx/unload_coordinator; do
      if ros2 node list 2>/dev/null | grep -Fxq "${node}"; then echo "OK      ${node}"; else echo "MISSING ${node}"; fi
    done
  fi
  for topic in /joint_states /camera1/image_raw /console/annotated_image /console/status; do
    if ros2 topic list 2>/dev/null | grep -Fxq "${topic}"; then echo "OK      ${topic}"; else echo "MISSING ${topic}"; fi
  done
}

stop_system() {
  if tmux has-session -t "${SESSION}" 2>/dev/null; then
    if tmux list-windows -t "${SESSION}" -F '#{window_name}' | grep -Fxq beagle; then
      tmux send-keys -t "${SESSION}:beagle" C-c
      sleep 2
    fi
    tmux kill-session -t "${SESSION}"
    echo "Stopped OMX GUI tmux session '${SESSION}'."
  else
    echo "OMX GUI tmux session was not running."
  fi
}

case "${1:-}" in
  start) start_system ;;
  detect-ports) resolve_omx_ports; resolve_beagle_local_launch ;;
  status) status_system ;;
  stop) stop_system ;;
  attach) exec tmux attach -t "${SESSION}" ;;
  *) echo "Usage: $0 {start|detect-ports|status|stop|attach}" >&2; exit 2 ;;
esac
