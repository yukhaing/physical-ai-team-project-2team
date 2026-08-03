#!/bin/bash
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
COMPOSE_FILE="${SCRIPT_DIR}/docker-compose.yml"
CONTAINER_NAME="omx_box_project"

show_help() {
  echo "Usage: $0 {build|start|enter|stop|logs|status}"
}

case "${1:-}" in
  build)
    docker compose -f "${COMPOSE_FILE}" build
    ;;
  start)
    if [ -n "${DISPLAY:-}" ] && command -v xhost >/dev/null 2>&1; then
      xhost +local:docker >/dev/null || true
    fi
    docker compose -f "${COMPOSE_FILE}" up -d --build
    ;;
  enter)
    docker exec -it "${CONTAINER_NAME}" bash
    ;;
  stop)
    docker compose -f "${COMPOSE_FILE}" down
    ;;
  logs)
    docker compose -f "${COMPOSE_FILE}" logs -f
    ;;
  status)
    docker compose -f "${COMPOSE_FILE}" ps
    ;;
  *)
    show_help
    exit 1
    ;;
esac
