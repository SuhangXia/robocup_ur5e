#!/bin/bash
# 快速启动指南 - 用于首次启动系统

cat << 'EOF'
╔══════════════════════════════════════════════════════════════╗
║          🎉 RoboCup UR5e 系统构建成功！                      ║
╚══════════════════════════════════════════════════════════════╝

✅ 所有 3 个 Docker 镜像已成功构建：

   • robocup_ur5e/brain            (4.7GB)
   • robocup_ur5e/perception_yolo  (18.2GB)
   • robocup_ur5e/perception_grasp (17GB)

══════════════════════════════════════════════════════════════

📋 启动前检查清单：

EOF

echo -n "  ⏳ 检查 Docker 镜像..."
image_count=$(docker images | grep "robocup_ur5e/" | wc -l)
if [ "$image_count" -ge 3 ]; then
    echo " ✅ (找到 $image_count 个镜像)"
else
    echo " ❌ (只找到 $image_count 个镜像，需要 3 个)"
    echo ""
    echo "错误：Docker 镜像不完整！"
    echo "请先运行: ./rebuild_all.sh"
    exit 1
fi

echo -n "  ⏳ 检查 .env 配置..."
if [ -f ".env" ]; then
    echo " ✅"
    set -a
    source .env 2>/dev/null || true
    set +a
else
    echo " ⚠️  创建默认配置..."
    cat > .env << 'ENVFILE'
# 本地 arm_gazebo Docker 模式
ROS_MASTER_URI=http://127.0.0.1:11311
ROS_IP=127.0.0.1

# NVIDIA GPU 配置
NVIDIA_VISIBLE_DEVICES=all
NVIDIA_DRIVER_CAPABILITIES=compute,utility
ENVFILE
    echo " ✅ 已创建"
    source .env 2>/dev/null || true
fi

# 若使用 localhost（本地 arm_gazebo Docker），检查本机 11311 端口
_ros_master_uri="${ROS_MASTER_URI:-}"
if echo "$_ros_master_uri" | grep -q "127.0.0.1\|localhost"; then
    echo -n "  ⏳ 检查本地 ROS Master (localhost:11311)..."
    _port_ok=0
    if (echo >/dev/tcp/127.0.0.1/11311) 2>/dev/null; then _port_ok=1; fi
    if [ $_port_ok -eq 0 ] && command -v nc >/dev/null 2>&1 && nc -z 127.0.0.1 11311 2>/dev/null; then _port_ok=1; fi
    if [ $_port_ok -eq 1 ]; then
        echo " ✅"
    else
        echo " ⚠️  未检测到 roscore"
        echo ""
        echo "  请先启动 arm_gazebo 环境："
        echo "    cd arm_gazebo/docker && sudo ./run.bash"
        echo "  等待 Gazebo 和 roscore 启动后再运行本脚本。"
        echo ""
    fi
else
    echo -n "  ⏳ 检查 VirtualBox VM 连接..."
    vm_ip=$(echo "${ROS_MASTER_URI}" | sed -n 's|http://\([^:]*\):.*|\1|p')
    if [ -n "$vm_ip" ] && ping -c 1 -W 2 "$vm_ip" > /dev/null 2>&1; then
        echo " ✅"
    else
        echo " ⚠️  警告：无法连接到 VM ($vm_ip)"
        echo ""
        echo "  请确保："
        echo "    1. VirtualBox VM 已启动"
        echo "    2. .env 中 ROS_MASTER_URI 指向正确的 VM IP"
        echo "    3. VM 中运行了 roscore"
        echo ""
    fi
fi

cat << 'EOF'

══════════════════════════════════════════════════════════════

🚀 启动选项：

EOF

PS3="请选择操作 (1-4): "
options=("启动所有服务" "查看容器状态" "查看日志" "退出")

select opt in "${options[@]}"
do
    case $opt in
        "启动所有服务")
            echo ""
            echo "正在启动所有服务..."
            docker-compose up -d
            echo ""
            echo "等待容器启动..."
            sleep 5
            echo ""
            echo "容器状态："
            docker-compose ps
            echo ""
            echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
            echo "✅ 系统已启动！"
            echo ""
            echo "查看日志："
            echo "  docker-compose logs -f"
            echo ""
            echo "测试 ROS 连接："
            echo "  docker-compose exec robocup_brain bash -c 'source /workspace/devel/setup.bash && rostopic list'"
            echo ""
            echo "停止系统："
            echo "  docker-compose down"
            echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
            break
            ;;
        "查看容器状态")
            echo ""
            docker-compose ps
            echo ""
            ;;
        "查看日志")
            echo ""
            echo "按 Ctrl+C 退出日志查看"
            sleep 2
            docker-compose logs -f
            break
            ;;
        "退出")
            echo ""
            echo "手动启动命令："
            echo "  docker-compose up -d"
            echo ""
            break
            ;;
        *)
            echo "无效选项 $REPLY"
            ;;
    esac
done
