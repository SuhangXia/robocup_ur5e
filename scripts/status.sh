#!/bin/bash
# 一键检查系统状态

# 统一 docker compose 命令（优先 V2）
DCOMPOSE="docker-compose"
if docker compose version >/dev/null 2>&1; then
  DCOMPOSE="docker compose"
fi

# 加载 .env 以便区分本地/VM 模式
[ -f ".env" ] && set -a && source .env 2>/dev/null || true && set +a

echo "╔══════════════════════════════════════════════════════════════╗"
echo "║          RoboCup UR5e 系统状态检查                          ║"
echo "╚══════════════════════════════════════════════════════════════╝"
echo ""

# 1. 检查 Docker 镜像
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo "📦 Docker 镜像："
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
if docker images | grep robocup_ur5e > /dev/null 2>&1; then
    docker images | grep robocup_ur5e | while read line; do
        echo "  ✅ $line"
    done
else
    echo "  ❌ 未找到 robocup_ur5e 镜像"
    echo "     运行: ./rebuild_all.sh"
fi
echo ""

# 2. 检查容器状态
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo "🐳 容器状态："
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
if $DCOMPOSE ps 2>/dev/null | grep -q "Up"; then
    $DCOMPOSE ps 2>/dev/null | tail -n +3 | while read line; do
        if echo "$line" | grep -q "Up"; then
            echo "  ✅ $line"
        else
            echo "  ⚠️  $line"
        fi
    done
else
    echo "  ⚠️  容器未运行"
    echo "     启动: ./start.sh 或 $DCOMPOSE up -d"
fi
echo ""

# 3. 检查网络配置
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo "🌐 网络配置："
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
if [ -f ".env" ]; then
    echo "  ✅ .env 配置文件存在"
    grep -v '^#' .env | grep -E '^ROS_MASTER_URI=|^ROS_IP=' | sed 's/^/     /'
else
    echo "  ❌ .env 文件不存在"
fi

_ros_uri="${ROS_MASTER_URI:-}"
echo ""
if echo "$_ros_uri" | grep -q "127.0.0.1\|localhost"; then
    echo -n "  本地 ROS Master (127.0.0.1:11311): "
    # WSL2+Docker: 127.0.0.1 在宿主机与容器内不同，必须从 brain 容器内检查
    _port_ok=0
    if $DCOMPOSE ps 2>/dev/null | grep -q "robocup_brain.*Up"; then
        if $DCOMPOSE exec -T robocup_brain bash -c "timeout 2 bash -c 'echo >/dev/tcp/127.0.0.1/11311' 2>/dev/null || (command -v nc >/dev/null && nc -z 127.0.0.1 11311 2>/dev/null)" 2>/dev/null; then
            _port_ok=1
        fi
    fi
    if [ $_port_ok -eq 1 ]; then
        echo "✅ 可达 (roscore 已运行)"
    else
        echo "❌ 不可达"
        echo "     请先启动 arm_gazebo: cd arm_gazebo/docker && sudo ./run.bash"
    fi
else
    vm_ip=$(echo "$_ros_uri" | sed -n 's|http://\([^:]*\):.*|\1|p')
    echo -n "  VM 连接 (${vm_ip:-?}): "
    if [ -n "$vm_ip" ] && ping -c 1 -W 2 "$vm_ip" > /dev/null 2>&1; then
        echo "✅ 可达"
    else
        echo "❌ 不可达"
        echo "     请检查 VirtualBox VM 是否启动"
    fi
fi
echo ""

# 4. 检查 ROS 话题（如果容器在运行）
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo "📡 ROS 话题："
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
if $DCOMPOSE ps 2>/dev/null | grep -q "robocup_brain.*Up"; then
    topics=$($DCOMPOSE exec -T robocup_brain bash -c "source /workspace/devel/setup.bash && timeout 5 rostopic list 2>/dev/null" 2>/dev/null)
    if [ -n "$topics" ]; then
        echo "$topics" | while read topic; do
            [ -n "$topic" ] && echo "  ✅ $topic"
        done
    else
        echo "  ⚠️  无法连接到 ROS Master"
        if echo "$_ros_uri" | grep -q "127.0.0.1\|localhost"; then
            echo "     请先启动 arm_gazebo 环境后再启动 RoboCup："
            echo "     终端1: cd arm_gazebo/docker && sudo ./run.bash"
            echo "     终端2: cd robocup_ur5e && ./scripts/start.sh"
        else
            echo "     请确保 VM 中运行了 roscore"
        fi
    fi
else
    echo "  ⚠️  Brain 容器未运行，无法检查话题"
fi
echo ""

# 5. 检查 GPU
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo "🎮 GPU 状态："
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
if command -v nvidia-smi > /dev/null 2>&1; then
    echo "  ✅ NVIDIA 驱动已安装"
    nvidia-smi --query-gpu=index,name,driver_version,memory.total --format=csv,noheader 2>/dev/null | while read line; do
        echo "     GPU $line"
    done
else
    echo "  ⚠️  nvidia-smi 未找到"
fi
echo ""

# 总结
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo "📋 快速命令："
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo "  启动系统:         ./start.sh"
echo "  查看状态:         $DCOMPOSE ps"
echo "  查看日志:         $DCOMPOSE logs -f"
echo "  停止系统:         $DCOMPOSE down"
echo "  重建镜像:         ./rebuild_all.sh"
echo "  查看话题:         $DCOMPOSE exec robocup_brain bash"
echo "                    source /workspace/devel/setup.bash"
echo "                    rostopic list"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
