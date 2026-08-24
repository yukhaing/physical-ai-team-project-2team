#!/usr/bin/env bash
set -Eeuo pipefail

CONTAINER="physical_ai_server"

if ! docker ps --format '{{.Names}}' | grep -Fxq "${CONTAINER}"; then
  echo "MISSING ${CONTAINER}"
  exit 1
fi

docker exec "${CONTAINER}" sh -lc '
if ps -eo args= | awk '\''/[y]olo_calibrated_preview.py/ { found=1 } END { exit !found }'\''; then
  echo "OK      YOLO detector process"
else
  echo "MISSING YOLO detector process"
fi
. /opt/ros/jazzy/setup.sh
export RMW_IMPLEMENTATION=rmw_zenoh_cpp ROS_DOMAIN_ID=30
ros2 topic info /yolo/selected_box 2>/dev/null || true
'
