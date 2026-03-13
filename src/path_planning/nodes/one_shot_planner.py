#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
一次性规划模块：从当前起点规划到目标，避障，输出 JointTrajectory 供 motion_control 执行。
与 ur5e_teach_pendant_ui 一致：不直接调用 IK/FK，相信 motion_control；起点位姿由 TF 获取，
目标关节由配置/上层给出，仅通过 /motion/command 下发 MotionCommand。

用法:
  from one_shot_planner import plan_one_shot, joints_to_trajectory, get_default_virtual_grasp_point
  path_joints, traj = plan_one_shot(
      start_xyz=..., start_joints=...,   # 由 TF + /joint_states 获取
      goal_xyz=..., goal_joints=...,     # goal_joints 由配置提供，不在此做 IK
      ...
  )
"""

from __future__ import division

import sys
import os

# 保证可导入同目录下的 aco_rrtstar_planner_node
_SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
if _SCRIPT_DIR not in sys.path:
    sys.path.insert(0, _SCRIPT_DIR)

import rospy
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from common_msgs.msg import MotionCommand

# 从 aco_rrtstar_planner_node 复用（仅 ACO/RRT* 与障碍物，不依赖 KinematicsClient/IK/FK）
from aco_rrtstar_planner_node import (
    pointcloud_to_obstacles,
    ACOPhase,
    RRTStarPhase,
    GAZEBO_DEFAULT_OBSTACLES,
    DEFAULT_VIRTUAL_GRASP_POINT,
    UR5eFK,
)

# 默认虚拟抓取点：与 planning_config.yaml 一致，Gazebo 箱体后方
def get_default_virtual_grasp_point():
    """返回默认虚拟抓取点 [x, y, z]（base_link 系），与既有配置一致。"""
    return list(DEFAULT_VIRTUAL_GRASP_POINT)


def _to_bounds(workspace_x, workspace_y, workspace_z):
    """(wx, wy, wz) -> bounds 三元组"""
    def to_pair(val, default):
        if isinstance(val, (list, tuple)) and len(val) >= 2:
            return (float(val[0]), float(val[1]))
        return default
    wx = to_pair(workspace_x, (-0.5, 1.0))
    wy = to_pair(workspace_y, (-0.5, 0.5))
    wz = to_pair(workspace_z, (0.0, 0.8))
    return (wx, wy, wz)


def plan_one_shot(
    start_xyz,
    start_joints,
    goal_xyz,
    goal_joints,
    obstacles=None,
    bounds=None,
    frame_id="base_link",
    aco_grid_res=0.08,
    aco_n_ants=40,
    aco_n_iters=30,
    aco_greedy_prob=0.3,
    aco_elite_deposit_ratio=2.0,
    rrt_step_size=0.12,
    rrt_max_iter=4000,
    return_vis_data=False,
):
    """
    一次性规划：从 start 到 goal，避障，返回关节路径与轨迹消息。
    与示教器一致：不在此调用 IK/FK，相信 motion_control；start_xyz 由 TF 提供，goal_joints 由配置/上层提供。

    Args:
        start_xyz: [x, y, z] 起点笛卡尔位置（base_link），由 TF base_link->tcp 获取，同 ur5e_teach_pendant_ui。
        start_joints: 起点关节角 (6,)；由 /joint_states 获取。
        goal_xyz: [x, y, z] 目标笛卡尔位置（用于 ACO 路径），一般即 virtual_grasp_point。
        goal_joints: 目标关节角 (6,)；由配置或上层提供，不在此做 IK。
        obstacles: 障碍物列表；None 时使用 GAZEBO_DEFAULT_OBSTACLES。
        bounds: 工作空间 (wx, wy, wz)；None 时从 rospy 参数或默认值取。

    Returns:
        (path_joints, trajectory_msg): path_joints 为 list of 6-tuple 或 None；trajectory_msg 为 trajectory_msgs/JointTrajectory 或 None。
    """
    try:
        start_xyz = list(start_xyz)[:3]
        start_joints = list(start_joints)[:6]
        goal_xyz = list(goal_xyz)[:3]
        goal_joints = list(goal_joints)[:6]
        if len(start_joints) != 6 or len(goal_joints) != 6:
            rospy.logerr("[OneShot] start_joints / goal_joints 需各 6 个关节角")
            return None, None

        if obstacles is None:
            obstacles = list(GAZEBO_DEFAULT_OBSTACLES)
        else:
            obstacles = list(obstacles)
        ground = ((0.0, 0.0, -0.06), (1.5, 1.5, 0.01))
        if ground not in obstacles:
            obstacles.append(ground)

        if bounds is None:
            wx = rospy.get_param("~workspace_x", [-0.5, 1.0])
            wy = rospy.get_param("~workspace_y", [-0.5, 0.5])
            wz = rospy.get_param("~workspace_z", [0.0, 0.8])
            bounds = _to_bounds(wx, wy, wz)

        start = tuple(start_joints)
        goal = tuple(goal_joints)
        start_xyz = tuple(float(x) for x in start_xyz)
        goal_xyz = tuple(float(x) for x in goal_xyz)

        aco = ACOPhase(grid_res=aco_grid_res, bounds=bounds)
        aco.mark_obstacles(obstacles)
        aco.run(start_xyz, goal_xyz, n_ants=aco_n_ants, n_iters=aco_n_iters,
                greedy_prob=aco_greedy_prob, elite_deposit_ratio=aco_elite_deposit_ratio)

        pheromone_fn = lambda x, y, z: aco.get_pheromone(x, y, z)
        rrt = RRTStarPhase(obstacles, pheromone_fn, step_size=rrt_step_size, max_iter=rrt_max_iter)
        if return_vis_data:
            path, rrt_nodes = rrt.plan(start, goal, return_tree=True)
        else:
            path = rrt.plan(start, goal)

        if path is None or len(path) == 0:
            rospy.logwarn("[OneShot] RRT* 未找到路径")
            if return_vis_data:
                aco_path_xyz = aco.get_path_xyz()
                aco_all_paths_xyz = aco.get_all_paths_xyz()
                tree_edges = [(rrt_nodes[q]['parent'], q) for q in rrt_nodes if rrt_nodes[q].get('parent') is not None]
                rrt_tree_edges_xyz = [(tuple(UR5eFK.fk(p)), tuple(UR5eFK.fk(q))) for (p, q) in tree_edges]
                vis_data = {
                    "aco_path_xyz": aco_path_xyz,
                    "aco_all_paths_xyz": aco_all_paths_xyz,
                    "rrt_path_xyz": [],
                    "rrt_tree_edges_xyz": rrt_tree_edges_xyz,
                    "obstacles": obstacles,
                    "start_xyz": start_xyz,
                    "goal_xyz": goal_xyz,
                }
                return None, None, vis_data
            return None, None

        rospy.loginfo("[OneShot] 规划完成，路径点数: %d", len(path))
        joint_names = list(RRTStarPhase.JOINT_NAMES)
        traj = joints_to_trajectory(path, joint_names, time_step=0.5)

        if return_vis_data:
            aco_path_xyz = aco.get_path_xyz()
            aco_all_paths_xyz = aco.get_all_paths_xyz()
            rrt_path_xyz = [tuple(UR5eFK.fk(q)) for q in path]
            tree_edges = [(rrt_nodes[q]['parent'], q) for q in rrt_nodes if rrt_nodes[q].get('parent') is not None]
            rrt_tree_edges_xyz = [(tuple(UR5eFK.fk(p)), tuple(UR5eFK.fk(q))) for (p, q) in tree_edges]
            vis_data = {
                "aco_path_xyz": aco_path_xyz,
                "aco_all_paths_xyz": aco_all_paths_xyz,
                "rrt_path_xyz": rrt_path_xyz,
                "rrt_tree_edges_xyz": rrt_tree_edges_xyz,
                "obstacles": obstacles,
                "start_xyz": start_xyz,
                "goal_xyz": goal_xyz,
            }
            return path, traj, vis_data
        return path, traj

    except Exception as e:
        rospy.logerr("[OneShot] 规划异常: %s", str(e))
        import traceback
        traceback.print_exc()
        if return_vis_data:
            return None, None, None
        return None, None


def joints_to_trajectory(path_joints, joint_names, time_step=0.5):
    """
    将关节路径转为 trajectory_msgs/JointTrajectory，各点间隔 time_step 秒。

    Args:
        path_joints: list of 6-tuple 或 list of list。
        joint_names: 关节名列表，与 motion_control 一致。
        time_step: 相邻点时间间隔（秒）。

    Returns:
        trajectory_msgs/JointTrajectory
    """
    traj = JointTrajectory()
    traj.joint_names = list(joint_names)
    traj.points = []
    t = 0.0
    from rospy import Duration
    for q in path_joints:
        pt = JointTrajectoryPoint()
        pt.positions = list(q)[:6]
        pt.time_from_start = Duration.from_sec(t)
        traj.points.append(pt)
        t += time_step
    return traj


def build_motion_command_execute_trajectory(trajectory):
    """构造 MotionCommand(EXECUTE_TRAJECTORY) 消息。"""
    cmd = MotionCommand()
    cmd.command_type = MotionCommand.EXECUTE_TRAJECTORY
    cmd.trajectory = trajectory
    cmd.max_velocity = 1.0
    cmd.max_acceleration = 1.0
    return cmd


def plot_planning_result_3d(vis_data, output_path=None):
    """
    根据 ACO 路径、RRT* 路径、最终路径与点云障碍物绘制 3D 坐标图并保存。

    vis_data: dict，需包含 aco_path_xyz, rrt_path_xyz, final_path_xyz, obstacles, start_xyz, goal_xyz。
    output_path: 保存路径，默认 /tmp/planning_result_3d.png
    """
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        from mpl_toolkits.mplot3d import Axes3D
        from mpl_toolkits.mplot3d.art3d import Poly3DCollection
        # 中文显示：使用支持 GBK 的字体，避免图例/标题乱码
        plt.rcParams["font.sans-serif"] = ["SimHei", "Microsoft YaHei", "SimSun", "KaiTi", "sans-serif"]
        plt.rcParams["axes.unicode_minus"] = False
        try:
            plt.rcParams["font.family"] = "sans-serif"
        except Exception:
            pass
    except ImportError:
        rospy.logwarn("[OneShot] 3D 绘图需要 matplotlib，跳过")
        return False

    if output_path is None:
        output_path = "/tmp/planning_result_3d.png"

    obstacles = vis_data.get("obstacles", [])
    aco_path_xyz = vis_data.get("aco_path_xyz", [])
    aco_all_paths_xyz = vis_data.get("aco_all_paths_xyz", [])
    rrt_path_xyz = vis_data.get("rrt_path_xyz", [])
    rrt_tree_edges_xyz = vis_data.get("rrt_tree_edges_xyz", [])
    start_xyz = vis_data.get("start_xyz")
    goal_xyz = vis_data.get("goal_xyz")

    fig = plt.figure(figsize=(10, 8))
    ax = fig.add_subplot(111, projection="3d")

    # RRT* 树：所有节点/边转笛卡尔后画枝杈（细线、半透明）
    tree_drawn = False
    for (p1, p2) in rrt_tree_edges_xyz:
        ax.plot([p1[0], p2[0]], [p1[1], p2[1]], [p1[2], p2[2]], "c-", linewidth=0.4, alpha=0.35)
        tree_drawn = True
    if tree_drawn:
        ax.plot([], [], [], "c-", linewidth=0.8, alpha=0.5, label="RRT* 树")

    # ACO 多条候选路径（细绿线、半透明）
    for path_xyz in aco_all_paths_xyz:
        if path_xyz and len(path_xyz) >= 2:
            xs, ys, zs = zip(*path_xyz)
            ax.plot(xs, ys, zs, "g-", linewidth=0.5, alpha=0.25)
    if aco_all_paths_xyz:
        ax.plot([], [], [], "g-", linewidth=1, alpha=0.5, label="ACO 候选路径")

    # 障碍物：每个 (center, half_extents) 画一个半透明立方体框
    for (cx, cy, cz), (hx, hy, hz) in obstacles:
        x = [cx - hx, cx + hx]
        y = [cy - hy, cy + hy]
        z = [cz - hz, cz + hz]
        verts = [
            [x[0], y[0], z[0]], [x[1], y[0], z[0]], [x[1], y[1], z[0]], [x[0], y[1], z[0]],
            [x[0], y[0], z[1]], [x[1], y[0], z[1]], [x[1], y[1], z[1]], [x[0], y[1], z[1]],
        ]
        faces = [
            [verts[0], verts[1], verts[2], verts[3]],
            [verts[4], verts[5], verts[6], verts[7]],
            [verts[0], verts[1], verts[5], verts[4]],
            [verts[2], verts[3], verts[7], verts[6]],
            [verts[0], verts[3], verts[7], verts[4]],
            [verts[1], verts[2], verts[6], verts[5]],
        ]
        ax.add_collection3d(Poly3DCollection(faces, facecolor="red", alpha=0.15, edgecolor="darkred", linewidths=0.5))

    # ACO 最优路径（粗绿线）
    if aco_path_xyz and len(aco_path_xyz) >= 2:
        xs, ys, zs = zip(*aco_path_xyz)
        ax.plot(xs, ys, zs, "g-", linewidth=2, alpha=0.95, label="ACO 最优路径")

    # RRT* 路径（即最终执行的关节路径在笛卡尔空间的投影）
    if rrt_path_xyz and len(rrt_path_xyz) >= 2:
        xs, ys, zs = zip(*rrt_path_xyz)
        ax.plot(xs, ys, zs, "b-", linewidth=2.5, alpha=0.9, label="RRT* / 最终路径")

    # 起点、终点
    if start_xyz and len(start_xyz) >= 3:
        ax.scatter([start_xyz[0]], [start_xyz[1]], [start_xyz[2]], c="k", s=80, marker="^", label="起点")
    if goal_xyz and len(goal_xyz) >= 3:
        ax.scatter([goal_xyz[0]], [goal_xyz[1]], [goal_xyz[2]], c="orange", s=80, marker="o", label="目标点")

    ax.set_xlabel("X (m)")
    ax.set_ylabel("Y (m)")
    ax.set_zlabel("Z (m)")
    ax.legend(loc="upper left", fontsize=8)
    ax.set_title("规划结果 3D：障碍物 / ACO 候选与最优 / RRT* 树与最终路径")

    os.makedirs(os.path.dirname(output_path) or ".", exist_ok=True)
    fig.savefig(output_path, dpi=150, bbox_inches="tight")
    plt.close()
    rospy.loginfo("[OneShot] 3D 规划图已保存: %s", output_path)
    return True
