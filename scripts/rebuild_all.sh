#!/bin/bash
# 一键重新构建所有容器

echo "=========================================="
echo "重新构建所有 Docker 镜像"
echo "=========================================="
echo ""

# 清理
echo "清理旧容器..."
docker-compose down 2>/dev/null

echo ""
echo "开始构建（预计 30-40 分钟）..."
echo ""

# 重新构建所有
docker-compose build --no-cache 2>&1 | tee /tmp/full_build.log

echo ""
echo "=========================================="
echo "构建完成！"
echo "=========================================="
echo ""

# 显示结果
echo "已构建的镜像："
docker images | grep robocup_ur5e

echo ""
echo "检查构建日志中的错误："
# 排除误报：
# - "Failed: No packages failed" 实际表示成功
# - "ln: failed to create symbolic link '/etc/resolv.conf'" 是正常警告
# - "ERROR" 在 Python traceback 或实际错误中才是真错误
if grep -i "ERROR:" /tmp/full_build.log | grep -v "WARNING" | grep -v "libgpg" || \
   grep "failed to build" /tmp/full_build.log || \
   grep "Service.*failed to build" /tmp/full_build.log; then
    echo "⚠️  发现错误，请检查日志：/tmp/full_build.log"
    echo ""
else
    echo "✅ 所有镜像构建成功！"
    echo ""
    echo "验证并启动系统："
    echo "  ./verify_and_start.sh"
    echo ""
    echo "或手动启动："
    echo "  docker-compose up -d"
    echo ""
    echo "查看日志："
    echo "  docker-compose logs -f"
fi
