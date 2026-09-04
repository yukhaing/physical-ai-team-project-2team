#!/usr/bin/env bash
set -Eeuo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
CONTAINER="physical_ai_server"
RUNTIME_DIR="/opt/omx_yolo"
CALIBRATION_SOURCE="${ROOT_DIR}/integration/omx_box_system/calibration/omx_camera_homography_7point.yaml"

if ! docker ps --format '{{.Names}}' | grep -Fxq "${CONTAINER}"; then
  echo "ERROR: ${CONTAINER} is not running." >&2
  exit 1
fi

if [[ ! -f "${CALIBRATION_SOURCE}" ]]; then
  echo "ERROR: current seven-point calibration does not exist:" >&2
  echo "  ${CALIBRATION_SOURCE}" >&2
  echo "Run camera_homography_7point_calibration.launch.py first." >&2
  exit 1
fi

docker exec "${CONTAINER}" mkdir -p "${RUNTIME_DIR}"
docker cp \
  "${ROOT_DIR}/integration/omx_box_system/experiments/yolo_calibrated_preview.py" \
  "${CONTAINER}:${RUNTIME_DIR}/yolo_calibrated_preview.py"
docker cp \
  "${CALIBRATION_SOURCE}" \
  "${CONTAINER}:${RUNTIME_DIR}/omx_camera_homography_7point.yaml"
docker cp \
  "${ROOT_DIR}/integration/omx_box_system/models/box_defect_best.pt" \
  "${CONTAINER}:${RUNTIME_DIR}/box_defect_best.pt"
docker cp \
  "${ROOT_DIR}/integration/omx_box_system/requirements-yolo-runtime.txt" \
  "${CONTAINER}:${RUNTIME_DIR}/requirements.txt"

if ! docker exec "${CONTAINER}" test -x "${RUNTIME_DIR}/venv/bin/python"; then
  docker exec "${CONTAINER}" sh -lc \
    'apt-get update && apt-get install -y --no-install-recommends python3.12-venv && rm -rf /var/lib/apt/lists/*'
  docker exec "${CONTAINER}" python3 -m venv --system-site-packages \
    "${RUNTIME_DIR}/venv"
fi

docker exec "${CONTAINER}" "${RUNTIME_DIR}/venv/bin/python" -m pip install \
  --disable-pip-version-check -r "${RUNTIME_DIR}/requirements.txt"

docker exec "${CONTAINER}" sh -lc \
  '. /opt/ros/jazzy/setup.sh && /opt/omx_yolo/venv/bin/python - <<"PY"
import cv2
import numpy
import rclpy
from cv_bridge import CvBridge
from ultralytics import YOLO

model = YOLO("/opt/omx_yolo/box_defect_best.pt")
assert model.task == "detect"
assert set(model.names.values()) == {"normal", "defect"}
print(f"YOLO runtime ready: numpy={numpy.__version__}, opencv={cv2.__version__}, classes={model.names}")
PY'
