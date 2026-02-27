#!/bin/bash
# Fix Git configuration and complete the push

echo "╔══════════════════════════════════════════════════════════════╗"
echo "║              Fix Git Setup & Complete Push                   ║"
echo "╚══════════════════════════════════════════════════════════════╝"
echo ""

cd /home/suhang/robocup_ur5e_ws

echo "📝 Step 1: Configure Git identity..."
git config user.name "Suhang Xia"
git config user.email "suhang@robocup.org"
echo "✅ Git identity configured"
echo ""

echo "📝 Step 2: Check current Git status..."
git status
echo ""

echo "📝 Step 3: Create commit (retry)..."
git commit -m "feat: initial RoboCup UR5e system with complete architecture

- Complete ROS 1 Noetic monorepo structure
- All Docker configurations (Brain, YOLO, Grasp)
- Interface definitions in common_msgs
- Skeleton code with TODO markers for all team members
- Comprehensive documentation (TEAM_README, SETUP_GUIDE, etc.)
- Multi-platform support (Ubuntu/WSL2/Mac)
- GPU acceleration (CUDA 11.3 & 12.0) with CPU fallback

Team: Suhang Xia, Jiaxin Liang, Sarvin, Chang Gao, Fazhan, Ruiyi, Muye Yuan
Institution: King's College London (KCL)
Competition: RoboCup 2026 - YCB Object Sorting"

if [ $? -eq 0 ]; then
    echo "✅ Commit created successfully"
else
    echo "❌ Commit failed"
    exit 1
fi
echo ""

echo "📝 Step 4: Verify branch..."
git branch -a
echo ""

echo "📝 Step 5: Push to GitHub..."
echo "   Remote: https://github.com/SuhangXia/robocup_ur5e.git"
echo "   Branch: main"
echo ""

git push -u origin main

if [ $? -eq 0 ]; then
    echo ""
    echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
    echo "✅ Successfully pushed to GitHub!"
    echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
    echo ""
    echo "🎉 Your repository is now live at:"
    echo "   https://github.com/SuhangXia/robocup_ur5e"
    echo ""
    echo "📋 Next steps:"
    echo "  1. Visit: https://github.com/SuhangXia/robocup_ur5e"
    echo "  2. Verify all files are there"
    echo "  3. Share with team:"
    echo "     git clone https://github.com/SuhangXia/robocup_ur5e.git"
    echo ""
    echo "📧 Team message template:"
    echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
    echo "Hi Team,"
    echo ""
    echo "RoboCup UR5e system is now on GitHub!"
    echo ""
    echo "📦 Repository: https://github.com/SuhangXia/robocup_ur5e"
    echo ""
    echo "📖 Quick Start:"
    echo "  1. Read SETUP_GUIDE.md (platform-specific)"
    echo "  2. Read TEAM_README.md (your tasks)"
    echo "  3. Clone and start:"
    echo "     git clone https://github.com/SuhangXia/robocup_ur5e.git"
    echo "     cd robocup_ur5e"
    echo "     ./start.sh"
    echo ""
    echo "- Suhang"
    echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
else
    echo ""
    echo "❌ Push failed!"
    echo ""
    echo "Common causes:"
    echo "  1. Repository doesn't exist on GitHub"
    echo "     → Create it at: https://github.com/new"
    echo "     → Name: robocup_ur5e"
    echo "     → Visibility: Public"
    echo ""
    echo "  2. Authentication required"
    echo "     → Configure credentials:"
    echo "       git config --global credential.helper store"
    echo "       (then re-run this script)"
    echo ""
    echo "  3. No push access"
    echo "     → Check repository permissions"
    echo ""
    echo "To retry manually:"
    echo "  git push -u origin main"
fi
