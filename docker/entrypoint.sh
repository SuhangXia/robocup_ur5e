#!/bin/bash
# Docker 容器入口点脚本

set -e

# 设置 ROS 环境变量
source /opt/ros/noetic/setup.bash

# 如果工作区已编译，source 工作区
if [ -f /workspace/devel/setup.bash ]; then
    source /workspace/devel/setup.bash
fi

# 配置 ROS_MASTER_URI（从环境变量读取，默认指向 VirtualBox VM）
export ROS_MASTER_URI=${ROS_MASTER_URI:-http://192.168.56.101:11311}
export ROS_IP=${ROS_IP:-$(hostname -I | awk '{print $1}')}

echo "========================================"
echo "ROS Environment:"
echo "  ROS_MASTER_URI: ${ROS_MASTER_URI}"
echo "  ROS_IP: ${ROS_IP}"
echo "  ROS_HOSTNAME: ${ROS_HOSTNAME}"
echo "========================================"

# 执行传入的命令
exec "$@"
