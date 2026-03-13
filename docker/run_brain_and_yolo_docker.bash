#!/usr/bin/env bash
# =============================================================================
# 同时启动 Brain + YOLO 两个容器（代码通过 compose 挂载）
# 启动后：终端1 执行 ./docker/run_brain_docker.bash 进 brain 的 shell，
#        终端2 执行 ./docker/run_yolo_docker.bash 进 yolo 的 shell。
# 在仓库根目录执行: ./docker/run_brain_and_yolo_docker.bash
# =============================================================================
set -e
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
DOCKER_COMPOSE="docker compose"
command -v docker-compose &>/dev/null && DOCKER_COMPOSE="docker-compose"

cd "$REPO_ROOT"
$DOCKER_COMPOSE up -d robocup_brain perception_yolo
echo ""
echo "Brain + YOLO 已启动。要进容器 shell："
echo "  终端1: ./docker/run_brain_docker.bash"
echo "  终端2: ./docker/run_yolo_docker.bash"
echo ""
