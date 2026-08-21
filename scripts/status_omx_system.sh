#!/usr/bin/env bash
set -Eeuo pipefail

CONTAINER="omx_box_project"
docker exec -it "${CONTAINER}" \
  /root/omx_box_project_ws/scripts/omx_system_container.sh status
