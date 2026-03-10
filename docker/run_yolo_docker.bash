#!/usr/bin/env bash
# =============================================================================
# 仅启动 Perception YOLO 容器并进入 shell（代码通过 compose 挂载）
# 在仓库根目录执行: ./docker/run_yolo_docker.bash
# =============================================================================
set -e
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
SERVICE_NAME="perception_yolo"
CONTAINER_NAME="perception_yolo"
DOCKER_COMPOSE="docker compose"
command -v docker-compose &>/dev/null && DOCKER_COMPOSE="docker-compose"

cd "$REPO_ROOT"
$DOCKER_COMPOSE up -d "$SERVICE_NAME"
sleep 1
docker exec -it "$CONTAINER_NAME" bash -c '
  source /opt/ros/noetic/setup.bash 2>/dev/null || true
  [ -f /workspace/devel/setup.bash ] && source /workspace/devel/setup.bash
  export ROS_MASTER_URI=${ROS_MASTER_URI:-http://localhost:11311}
  export ROS_IP=${ROS_IP:-}
  cd /workspace 2>/dev/null || true
  exec bash
'
