#!/usr/bin/env bash
set -Eeuo pipefail

CONTAINER="${OMX_CONTAINER_NAME:-omx_box_project}"
SESSION="unload_omx_process"
CALIBRATION_SESSION="unload_camera_calibration"
TEACH_SESSION="unload_omx_teach"
ROOT_WORKSPACE="/root/omx_box_project_ws"
UNLOAD_WORKSPACE="${ROOT_WORKSPACE}/integration/yeongjin_gui/omx"
DEFAULT_PORT="/dev/serial/by-id/usb-ROBOTIS_OpenRB-150_CDC2589C5157375037202020FF0F0C0D-if00"
PORT_NAME="${UNLOAD_OMX_PORT_NAME:-${DEFAULT_PORT}}"
VIDEO_DEVICE="${UNLOAD_VIDEO_DEVICE:-/dev/video2}"
DRY_RUN="${UNLOAD_DRY_RUN:-true}"
WEB_PORT="${UNLOAD_CALIBRATION_PORT:-8088}"
CALIBRATION_CONFIG="${UNLOAD_WORKSPACE}/src/omx_box_control/config/unload_homography_7point_calibration.yaml"

inside() {
  docker exec "${CONTAINER}" bash -lc "$1"
}

inside_interactive() {
  docker exec -it "${CONTAINER}" bash -lc "$1"
}

source_command() {
  printf 'source /opt/ros/jazzy/setup.bash; source /root/ros2_ws/install/setup.bash; source %q/install/setup.bash; export PATH=/opt/ultralytics-venv/bin:/opt/ros/jazzy/bin:/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin' "${UNLOAD_WORKSPACE}"
}

require_container() {
  docker inspect -f '{{.State.Running}}' "${CONTAINER}" 2>/dev/null | grep -Fxq true || {
    echo "ERROR: container is not running: ${CONTAINER}" >&2
    exit 1
  }
}

require_value() {
  local name="$1" value="$2"
  [[ -n "${value}" ]] || { echo "ERROR: ${name} is empty." >&2; exit 1; }
}

build_package() {
  require_container
  inside "source /opt/ros/jazzy/setup.bash; source /root/ros2_ws/install/setup.bash; cd '${UNLOAD_WORKSPACE}'; colcon build --symlink-install --packages-select omx_box_control"
}

start_process() {
  require_container
  require_value UNLOAD_OMX_PORT_NAME "${PORT_NAME}"
  case "${DRY_RUN,,}" in true|false) ;; *) echo 'ERROR: UNLOAD_DRY_RUN must be true or false.' >&2; exit 1;; esac
  inside "test -e '${PORT_NAME}'" || { echo "ERROR: unloading OMX port not found: ${PORT_NAME}" >&2; exit 1; }
  inside "test -e '${VIDEO_DEVICE}'" || { echo "ERROR: unloading camera not found: ${VIDEO_DEVICE}" >&2; exit 1; }
  inside "test -f '${UNLOAD_WORKSPACE}/install/setup.bash'" || {
    echo "ERROR: unloading workspace is not built; run '$0 build'." >&2
    exit 1
  }
  if inside "tmux has-session -t '${SESSION}' 2>/dev/null"; then
    echo "ERROR: ${SESSION} is already running." >&2
    exit 1
  fi
  if inside "tmux has-session -t '${CALIBRATION_SESSION}' 2>/dev/null"; then
    echo "ERROR: stop ${CALIBRATION_SESSION} before starting the robot process." >&2
    exit 1
  fi
  local command
  command="$(source_command); exec ros2 launch omx_box_control unload_process.launch.py port_name:='${PORT_NAME}' video_device:='${VIDEO_DEVICE}' dry_run:='${DRY_RUN}'"
  inside "tmux new-session -d -s '${SESSION}' -n process \"${command}\""
  sleep 2
  inside "tmux has-session -t '${SESSION}' 2>/dev/null" || {
    echo "ERROR: ${SESSION} exited during startup." >&2
    exit 1
  }
  echo "Waiting for unloading service, arm feedback, and camera frames..."
  inside "$(source_command); timeout 45 bash -c 'until ros2 service type /unload_omx/unload_coordinator/start 2>/dev/null | grep -q std_srvs/srv/Trigger; do sleep 1; done'" || {
    echo "ERROR: unloading coordinator service did not become ready." >&2
    exit 1
  }
  inside "$(source_command); timeout 45 ros2 topic echo --once /unload_omx/joint_states --field name >/dev/null 2>&1" || {
    echo "ERROR: unloading arm feedback did not become ready." >&2
    exit 1
  }
  inside "$(source_command); timeout 45 ros2 topic echo --once /unload_camera/image_raw --field header >/dev/null 2>&1" || {
    echo "ERROR: unloading camera frames did not become ready." >&2
    exit 1
  }
  echo "Standalone unloading process started (dry_run=${DRY_RUN})."
  echo "Arm feedback and unloading camera are ready."
  echo "No robot motion occurs until: $0 cycle"
}

stop_session() {
  local session="$1"
  if inside "tmux has-session -t '${session}' 2>/dev/null"; then
    inside "tmux send-keys -t '${session}' C-c; sleep 2; tmux kill-session -t '${session}' 2>/dev/null || true"
    echo "Stopped ${session}."
  else
    echo "${session} is not running."
  fi
}

status_process() {
  require_container
  if ! inside "tmux has-session -t '${SESSION}' 2>/dev/null"; then
    echo "${SESSION} is not running."
    exit 1
  fi
  inside "$(source_command); printf '%s\n' '[nodes]'; ros2 node list 2>/dev/null | grep -E '^/(usb_cam|unload_(omx|vision|marker))' || true; printf '%s\n' '[vision]'; timeout 4 ros2 topic echo --once /unload_vision/status 2>/dev/null || true; printf '%s\n' '[coordinator]'; ros2 param get /unload_omx/unload_coordinator dry_run 2>/dev/null || true; ros2 service list 2>/dev/null | grep '^/unload_omx/unload_coordinator/' || true; printf '%s\n' '[joints]'; timeout 4 ros2 topic echo --once /unload_omx/joint_states --field name 2>/dev/null || true"
}

cycle() {
  require_container
  inside "$(source_command); ros2 service call /unload_omx/unload_coordinator/start std_srvs/srv/Trigger '{}'"
}

cancel_cycle() {
  require_container
  inside "$(source_command); ros2 service call /unload_omx/unload_coordinator/cancel std_srvs/srv/Trigger '{}'"
}

start_teaching() {
  require_container
  require_value UNLOAD_OMX_PORT_NAME "${PORT_NAME}"
  [[ -t 0 && -t 1 ]] || {
    echo 'ERROR: teach requires an interactive terminal.' >&2
    exit 1
  }
  inside "test -e '${PORT_NAME}'" || {
    echo "ERROR: unloading OMX port not found: ${PORT_NAME}" >&2
    exit 1
  }
  inside "test -e '${VIDEO_DEVICE}'" || {
    echo "ERROR: unloading camera not found: ${VIDEO_DEVICE}" >&2
    exit 1
  }
  inside "test -f '${UNLOAD_WORKSPACE}/install/setup.bash'" || {
    echo "ERROR: unloading workspace is not built; run '$0 build'." >&2
    exit 1
  }
  for session in "${SESSION}" "${CALIBRATION_SESSION}" "${TEACH_SESSION}"; do
    if inside "tmux has-session -t '${session}' 2>/dev/null"; then
      echo "ERROR: stop the existing ${session} session first." >&2
      exit 1
    fi
  done
  local command
  command="$(source_command); exec ros2 launch omx_box_control unload_process.launch.py port_name:='${PORT_NAME}' video_device:='${VIDEO_DEVICE}' dry_run:=true start_coordinator:=false teaching_mode:=true"
  inside "tmux new-session -d -s '${TEACH_SESSION}' -n infrastructure; tmux set-option -t '${TEACH_SESSION}' remain-on-exit on; tmux send-keys -t '${TEACH_SESSION}:infrastructure' \"${command}\" Enter"
  echo 'Starting unloading arm, MoveJ controller, camera, and teaching vision...'
  local ready=false
  for _attempt in $(seq 1 30); do
    if inside "$(source_command); ros2 topic list 2>/dev/null | grep -Fxq /unload_omx/joint_states && ros2 topic list 2>/dev/null | grep -Fxq /unload_omx/vision_raw_target"; then
      ready=true
      break
    fi
    sleep 1
  done
  if [[ "${ready}" != true ]]; then
    inside "tmux capture-pane -p -S -120 -t '${TEACH_SESSION}:infrastructure'" || true
    stop_session "${TEACH_SESSION}"
    echo 'ERROR: teaching topics did not become ready.' >&2
    exit 1
  fi
  cleanup_teaching() {
    trap - EXIT INT TERM
    stop_session "${TEACH_SESSION}"
  }
  trap cleanup_teaching EXIT INT TERM
  echo 'Teaching ready. Use W/S/A/D, select 1/5/0 mm, P to inspect, V to save, X to exit.'
  inside_interactive "$(source_command); exec ros2 run omx_box_control unload_keyboard_teach_node.py"
}

start_calibration() {
  require_container
  inside "test -e '${VIDEO_DEVICE}'" || { echo "ERROR: unloading camera not found: ${VIDEO_DEVICE}" >&2; exit 1; }
  if inside "tmux has-session -t '${CALIBRATION_SESSION}' 2>/dev/null"; then
    echo "ERROR: ${CALIBRATION_SESSION} is already running." >&2
    exit 1
  fi
  if inside "tmux has-session -t '${SESSION}' 2>/dev/null"; then
    echo "ERROR: stop ${SESSION} before opening calibration." >&2
    exit 1
  fi
  local ros_source camera_command web_command
  ros_source="$(source_command)"
  camera_command="${ros_source}; sleep 2; exec ros2 launch open_manipulator_bringup camera_usb_cam.launch.py name:=unload_camera video_device:='${VIDEO_DEVICE}'"
  web_command="${ros_source}; sleep 4; exec python3 '${ROOT_WORKSPACE}/scripts/camera_mjpeg_server.py' --ros-topic /unload_camera/image_raw --port '${WEB_PORT}' --calibration-config '${CALIBRATION_CONFIG}'"
  inside "tmux new-session -d -s '${CALIBRATION_SESSION}' -n zenoh \"${ros_source}; exec ros2 run rmw_zenoh_cpp rmw_zenohd\"; tmux new-window -t '${CALIBRATION_SESSION}' -n camera \"${camera_command}\"; tmux new-window -t '${CALIBRATION_SESSION}' -n web \"${web_command}\""
  echo "Unload calibration screen: http://127.0.0.1:${WEB_PORT}/"
  echo 'Click START, then P1 through P7 in the configured order.'
}

case "${1:-}" in
  build) build_package ;;
  up|start) start_process ;;
  down|stop) require_container; stop_session "${SESSION}" ;;
  status) status_process ;;
  cycle) cycle ;;
  cancel) cancel_cycle ;;
  logs) require_container; inside "tmux capture-pane -p -S -300 -t '${SESSION}:process'" ;;
  calibration-up) start_calibration ;;
  calibration-down) require_container; stop_session "${CALIBRATION_SESSION}" ;;
  calibration-status) require_container; inside "tmux list-windows -t '${CALIBRATION_SESSION}' -F '#{window_name} #{?pane_dead,DEAD,RUNNING}'" ;;
  teach) start_teaching ;;
  *)
    echo "Usage: $0 {build|up|down|status|cycle|cancel|logs|teach|calibration-up|calibration-down|calibration-status}" >&2
    exit 2
    ;;
esac

