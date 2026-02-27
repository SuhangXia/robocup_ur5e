#!/bin/bash
# 检查容器运行状态和 ROS 节点

echo "╔══════════════════════════════════════════════════════════════╗"
echo "║          🔍 RoboCup UR5e 系统运行状态检查                   ║"
echo "╚══════════════════════════════════════════════════════════════╝"
echo ""

echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo "1️⃣  容器状态："
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
docker-compose ps 2>/dev/null | tail -n +3 | while read line; do
    if echo "$line" | grep -q "Up"; then
        echo "  ✅ $line"
    else
        echo "  ❌ $line"
    fi
done
echo ""

echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo "2️⃣  容器日志（最后 10 行）："
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo ""
echo "📦 Brain 容器："
docker-compose logs --tail=10 brain 2>/dev/null | sed 's/^/  /'
echo ""

echo "📦 Perception YOLO 容器："
docker-compose logs --tail=10 perception_yolo 2>/dev/null | sed 's/^/  /'
echo ""

echo "📦 Perception Grasp 容器："
docker-compose logs --tail=10 perception_grasp 2>/dev/null | sed 's/^/  /'
echo ""

echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo "3️⃣  ROS 节点状态（如果 roscore 运行中）："
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
ros_nodes=$(docker-compose exec -T robocup_brain bash -c "source /workspace/devel/setup.bash && timeout 3 rosnode list 2>/dev/null" 2>/dev/null)
if [ -n "$ros_nodes" ]; then
    echo "$ros_nodes" | while read node; do
        echo "  ✅ $node"
    done
else
    echo "  ⚠️  无法连接到 ROS Master"
    echo "     原因：VirtualBox VM 中的 roscore 未运行"
    echo "     这是正常的，容器会一直重试连接"
fi
echo ""

echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo "4️⃣  ROS 话题（如果 roscore 运行中）："
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
ros_topics=$(docker-compose exec -T robocup_brain bash -c "source /workspace/devel/setup.bash && timeout 3 rostopic list 2>/dev/null" 2>/dev/null)
if [ -n "$ros_topics" ]; then
    echo "$ros_topics" | while read topic; do
        echo "  ✅ $topic"
    done
else
    echo "  ⚠️  无法获取话题列表"
    echo "     原因：VirtualBox VM 中的 roscore 未运行"
fi
echo ""

echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo "📝 常用命令："
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo "  查看实时日志:     docker-compose logs -f"
echo "  查看单个服务:     docker-compose logs -f brain"
echo "  进入容器调试:     docker-compose exec robocup_brain bash"
echo "  重启服务:         docker-compose restart brain"
echo "  停止所有:         docker-compose down"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
