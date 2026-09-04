#!/usr/bin/env bash
set -Eeuo pipefail

CONTAINER="omx_box_project"

if docker ps --format '{{.Names}}' | grep -Fxq "${CONTAINER}"; then
  docker exec "${CONTAINER}" \
    /root/omx_box_project_ws/scripts/omx_system_container.sh stop
else
  echo "OMX container is not running."
fi
