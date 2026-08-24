#!/usr/bin/env bash
set -Eeuo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
CONTAINER="physical_ai_server"

if ! docker ps --format '{{.Names}}' | grep -Fxq "${CONTAINER}"; then
  echo "ERROR: ${CONTAINER} is not running." >&2
  exit 1
fi

if docker exec "${CONTAINER}" sh -lc \
  'ps -eo args= | awk '\''/[y]olo_calibrated_preview.py/ { found=1 } END { exit !found }'\'''; then
  echo "ERROR: YOLO detector is already running in ${CONTAINER}." >&2
  echo "Stop its existing terminal with Ctrl+C before starting another instance." >&2
  exit 1
fi

"${ROOT_DIR}/scripts/setup_yolo_runtime.sh"

exec docker exec -it \
  -e RMW_IMPLEMENTATION=rmw_zenoh_cpp \
  -e ROS_DOMAIN_ID=30 \
  -e YOLO_CONFIG_DIR=/tmp/Ultralytics \
  -e OMX_YOLO_MODEL=/opt/omx_yolo/box_defect_best.pt \
  -e OMX_YOLO_CALIBRATION=/opt/omx_yolo/omx_camera_homography_7point.yaml \
  -e OMX_YOLO_DEVICE="${OMX_YOLO_DEVICE:-cpu}" \
  "${CONTAINER}" sh -lc \
  '. /opt/ros/jazzy/setup.sh && exec /opt/omx_yolo/venv/bin/python /opt/omx_yolo/yolo_calibrated_preview.py'
