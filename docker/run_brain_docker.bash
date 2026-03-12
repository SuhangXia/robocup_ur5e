#!/usr/bin/env bash
# =============================================================================
# 启动/进入 RoboCup Brain 容器（代码通过 compose 挂载）
# 默认：不重建镜像，仅确保容器运行并进入 shell
# 可选：--build 先重建镜像再启动
# 在宿主机仓库目录执行: ./docker/run_brain_docker.bash [--build] [--force-recreate]
# =============================================================================
set -e
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
SERVICE_NAME="robocup_brain"
CONTAINER_NAME="robocup_brain"

BUILD_IMAGE=0
FORCE_RECREATE=0
for arg in "$@"; do
  case "$arg" in
    --build)
      BUILD_IMAGE=1
      ;;
    --force-recreate)
      FORCE_RECREATE=1
      ;;
    -h|--help)
      echo "Usage: ./docker/run_brain_docker.bash [--build] [--force-recreate]"
      exit 0
      ;;
    *)
      echo "[run_brain] Unknown option: $arg" >&2
      echo "Usage: ./docker/run_brain_docker.bash [--build] [--force-recreate]" >&2
      exit 1
      ;;
  esac
done

# 兼容 docker-compose（旧）与 docker compose（新）
DOCKER_COMPOSE="docker compose"
command -v docker-compose &>/dev/null && DOCKER_COMPOSE="docker-compose"

cd "$REPO_ROOT"

is_running() {
  docker ps -q -f "name=^${CONTAINER_NAME}$" | grep -q .
}

compose_up() {
  local cmd=("$DOCKER_COMPOSE" up -d)
  [ "$BUILD_IMAGE" -eq 1 ] && cmd+=(--build)
  [ "$FORCE_RECREATE" -eq 1 ] && cmd+=(--force-recreate)
  cmd+=("$SERVICE_NAME")
  "${cmd[@]}"
}

if [ "$BUILD_IMAGE" -eq 1 ] || [ "$FORCE_RECREATE" -eq 1 ] || ! is_running; then
  if ! compose_up; then
    echo "[run_brain] compose up failed, cleaning stale containers and retrying..."
    docker rm -f "$CONTAINER_NAME" 2>/dev/null || true
    while IFS= read -r stale; do
      [ -n "$stale" ] && docker rm -f "$stale" >/dev/null 2>&1 || true
    done < <(docker ps -a --format '{{.Names}}' | grep -E "_${CONTAINER_NAME}$" || true)
    compose_up
  fi
fi
sleep 1
docker exec -it "$CONTAINER_NAME" bash -c '
  source /opt/ros/noetic/setup.bash 2>/dev/null || true
  [ -f /workspace/devel/setup.bash ] && source /workspace/devel/setup.bash
  export ROS_MASTER_URI=${ROS_MASTER_URI:-http://localhost:11311}
  export ROS_IP=${ROS_IP:-}
  cd /workspace 2>/dev/null || true
  exec bash
'
