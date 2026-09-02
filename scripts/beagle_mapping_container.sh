#!/usr/bin/env bash
set -Eeuo pipefail

CONTAINER="${BEAGLE_CONTAINER_NAME:-omx_box_project}"
SESSION="beagle_mapping"
ROOT_WORKSPACE="/root/omx_box_project_ws"
BEAGLE_ROOT="${ROOT_WORKSPACE}/integration/yeongjin_gui/Beagle_mobile_robot"
ROS_ROOT="${BEAGLE_ROOT}/ros2"
PORT_NAME="${BEAGLE_PORT_NAME:-/dev/ttyACM0}"

inside() {
  docker exec "${CONTAINER}" bash -lc "$1"
}

source_command() {
  printf 'source /opt/ros/jazzy/setup.bash; source /root/ros2_ws/install/setup.bash; source %q; export RMW_IMPLEMENTATION=rmw_zenoh_cpp' \
    "${ROS_ROOT}/install/setup.bash"
}

case "${1:-}" in
  install-deps)
    docker exec "${CONTAINER}" bash -lc \
      'apt-get update && apt-get install -y --no-install-recommends ros-jazzy-nav2-map-server ros-jazzy-slam-toolbox ros-jazzy-teleop-twist-keyboard'
    ;;
  build)
    inside "source /opt/ros/jazzy/setup.bash; source /root/ros2_ws/install/setup.bash; cd '${ROS_ROOT}'; colcon build --base-paths beagle_slam --symlink-install --packages-select beagle_slam"
    ;;
  start)
    inside "test -e '${PORT_NAME}'"
    inside "test -f '${ROS_ROOT}/install/setup.bash'"
    if inside "tmux has-session -t '${SESSION}' 2>/dev/null"; then
      echo "ERROR: tmux session '${SESSION}' is already running." >&2
      exit 1
    fi
    command="$(source_command); exec ros2 launch beagle_slam mapping.launch.py port_name:='${PORT_NAME}' use_rviz:=true"
    docker exec "${CONTAINER}" tmux new-session -d -s "${SESSION}" -n mapping "${command}"
    echo "Mapping started. Inspect with: $0 status"
    echo "Drive with:            $0 teleop"
    ;;
  teleop)
    docker exec -it "${CONTAINER}" bash -lc \
      "$(source_command); ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=/cmd_vel"
    ;;
  save)
    name="${2:-beagle_map}"
    [[ "${name}" =~ ^[A-Za-z0-9_-]+$ ]] || {
      echo "ERROR: map name may contain only letters, digits, underscores and hyphens." >&2
      exit 2
    }
    inside "$(source_command); ros2 run beagle_slam save_map.sh '${BEAGLE_ROOT}/ros2/beagle_slam/maps/${name}'"
    ;;
  status)
    inside "tmux list-windows -t '${SESSION}' -F '#{window_name} #{?pane_dead,DEAD,RUNNING}' 2>/dev/null || true; $(source_command); ros2 topic list | grep -E '^/(map|odom|scan|tf|tf_static)$' | sort"
    ;;
  attach)
    docker exec -it "${CONTAINER}" tmux attach-session -t "${SESSION}"
    ;;
  stop)
    inside "tmux send-keys -t '${SESSION}:mapping' C-c 2>/dev/null || true; sleep 2; tmux kill-session -t '${SESSION}' 2>/dev/null || true"
    ;;
  *)
    echo "Usage: $0 {install-deps|build|start|teleop|save [map_name]|status|attach|stop}" >&2
    exit 2
    ;;
esac
