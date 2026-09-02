#!/usr/bin/env bash
set -Eeuo pipefail

if [[ $# -ne 1 ]]; then
  echo "Usage: $0 /absolute/or/relative/map_prefix" >&2
  exit 2
fi

MAP_PREFIX="$1"
mkdir -p "$(dirname "${MAP_PREFIX}")"

ros2 run nav2_map_server map_saver_cli -f "${MAP_PREFIX}"
ros2 service call /slam_toolbox/serialize_map \
  slam_toolbox/srv/SerializePoseGraph "{filename: '${MAP_PREFIX}'}"

echo "Saved occupancy map: ${MAP_PREFIX}.yaml and ${MAP_PREFIX}.pgm"
echo "Saved SLAM pose graph: ${MAP_PREFIX}.posegraph and ${MAP_PREFIX}.data"
