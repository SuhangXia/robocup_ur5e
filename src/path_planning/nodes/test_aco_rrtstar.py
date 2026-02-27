#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
ACO+RRT* 混合规划测试脚本
启动规划节点，订阅点云、规划路径、RViz 可视化

Usage:
  docker exec -it path_planning bash -c "source /workspace/scripts/setup_container_env.sh && python3 /workspace/src/path_planning/nodes/test_aco_rrtstar.py"

或:
  docker exec -it robocup_brain bash -c "source /workspace/scripts/setup_container_env.sh && rosrun path_planning test_aco_rrtstar.py"
"""

import rospy
import sys
import os
import subprocess

# 添加 nodes 目录以便导入同目录模块
_NODES_DIR = os.path.dirname(os.path.abspath(__file__))
_PKG_DIR = os.path.dirname(os.path.dirname(_NODES_DIR))
if _NODES_DIR not in sys.path:
    sys.path.insert(0, _NODES_DIR)

def main():
    print("=" * 60)
    print("ACO+RRT* 混合规划测试")
    print("=" * 60)
    print("请确保：")
    print("  1. 已启动 roscore 或 arm_gazebo/demo")
    print("  2. use_virtual_grasp_only=true 时可不依赖点云，使用虚拟抓取点规划")
    print("  3. RViz 或 Matplotlib PNG（见 path_planning_viz/）")
    print("")
    cfg = os.path.join(_PKG_DIR, 'config', 'planning_config.yaml')
    if os.path.isfile(cfg):
        try:
            subprocess.call(['rosparam', 'load', cfg, 'aco_rrtstar_planner'], timeout=5)
        except Exception:
            pass
    print("运行规划节点...")

    import aco_rrtstar_planner_node
    node = aco_rrtstar_planner_node.ACO_RRTStarPlannerNode()
    node.run()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
