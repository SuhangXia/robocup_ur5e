#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
ACO+RRT* 混合规划节点
- 订阅 3D 点云，全部转化为障碍物
- 支持虚拟抓取点：在任意位置（默认：Gazebo 桌面上箱子后方）进行路径规划
- 在 RViz 中可视化障碍物、ACO 路径、RRT* 路径、虚拟抓取点
- 机械臂站位与 Gazebo 相同 (x=-0.10, y=0, z=0.615)
- IK/FK 优先订阅 motion_control 的 ROS 服务，不可用时回退到本地计算
"""

import rospy
import random
import ast
import numpy as np
from math import pi, sqrt, cos, sin

try:
    from scipy.optimize import minimize
    HAS_SCIPY = True
except ImportError:
    HAS_SCIPY = False

from sensor_msgs.msg import PointCloud2, JointState
import sensor_msgs.point_cloud2 as pc2
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point, PoseStamped
from std_msgs.msg import ColorRGBA, Header

# motion_control 运动学服务 (可选)
try:
    from path_planning.srv import ComputeIK, ComputeFK
    HAS_KINEMATICS_SRV = True
except ImportError:
    HAS_KINEMATICS_SRV = False


# =============================================================================
# UR5e 前向运动学 (DH 参数) - 本地备用实现
# =============================================================================
class UR5eFK:
    DH = [
        (0.163, 0, 0, pi/2),
        (0, -0.425, 0, 0),
        (0, -0.39225, 0, 0),
        (0.134, 0, 0, pi/2),
        (0.1, 0, 0, -pi/2),
        (0.1, 0, 0, 0),
    ]

    @staticmethod
    def transform_dh(d, a, alpha, theta):
        ct, st = cos(theta), sin(theta)
        ca, sa = cos(alpha), sin(alpha)
        return np.array([
            [ct, -st*ca, st*sa, a*ct],
            [st, ct*ca, -ct*sa, a*st],
            [0, sa, ca, d],
            [0, 0, 0, 1]
        ])

    @classmethod
    def fk(cls, joints):
        T = np.eye(4)
        for i, (d, a, alpha, offset) in enumerate(cls.DH):
            theta = joints[i] + offset
            T = T @ cls.transform_dh(d, a, alpha, theta)
        return T[:3, 3]

    @classmethod
    def fk_chain(cls, joints):
        poses = [np.array([0, 0, 0])]
        T = np.eye(4)
        for i, (d, a, alpha, offset) in enumerate(cls.DH):
            theta = joints[i] + offset
            T = T @ cls.transform_dh(d, a, alpha, theta)
            poses.append(T[:3, 3].copy())
        return poses


def ik_target_local(target_xyz, q_init=None, joint_limits=None):
    """
    本地数值 IK：求关节角使末端到达 target_xyz [x,y,z]
    使用 scipy.minimize 最小化 ||FK(q) - target||^2
    当 motion_control 的 IK 服务不可用时使用
    """
    if joint_limits is None:
        joint_limits = [(-2*pi, 2*pi)] * 6
    if q_init is None:
        q_init = np.array([0, -pi/2, 0, -pi/2, 0, 0])
    target = np.array(target_xyz, dtype=float)

    def cost(q):
        ee = UR5eFK.fk(q)
        return np.sum((ee - target) ** 2)

    if HAS_SCIPY:
        res = minimize(cost, q_init, method='L-BFGS-B',
                     bounds=joint_limits,
                     options={'maxiter': 200})
        if res.fun < 1e-4:
            return tuple(res.x)
    return None


# =============================================================================
# KinematicsClient: 优先使用 motion_control 服务，不可用时回退本地
# =============================================================================
class KinematicsClient:
    """封装 IK/FK 调用：优先 motion_control 服务，回退本地计算"""
    IK_SERVICE = '/motion/compute_ik'
    FK_SERVICE = '/motion/compute_fk'
    EE_POSE_TOPIC = '/motion/ee_pose'

    def __init__(self, use_motion_control=True):
        self.use_motion_control = use_motion_control
        self._ik_proxy = None
        self._fk_proxy = None
        self._ee_pose = None  # 订阅 motion_control 发布的当前末端位姿

        if use_motion_control and HAS_KINEMATICS_SRV:
            try:
                rospy.wait_for_service(self.IK_SERVICE, timeout=2.0)
                self._ik_proxy = rospy.ServiceProxy(self.IK_SERVICE, ComputeIK)
                rospy.loginfo("[ACO+RRT*] 已连接 motion_control IK 服务: %s", self.IK_SERVICE)
            except (rospy.ROSException, rospy.ROSInterruptException):
                rospy.logwarn("[ACO+RRT*] motion_control IK 服务不可用，使用本地 IK")
            try:
                rospy.wait_for_service(self.FK_SERVICE, timeout=2.0)
                self._fk_proxy = rospy.ServiceProxy(self.FK_SERVICE, ComputeFK)
                rospy.loginfo("[ACO+RRT*] 已连接 motion_control FK 服务: %s", self.FK_SERVICE)
            except (rospy.ROSException, rospy.ROSInterruptException):
                rospy.logwarn("[ACO+RRT*] motion_control FK 服务不可用，使用本地 FK")

    def ik(self, target_xyz):
        """目标笛卡尔坐标 [x,y,z] -> 关节角 tuple 或 None"""
        if self._ik_proxy is not None:
            try:
                req = ComputeIK.Request()
                req.target_pose = PoseStamped()
                req.target_pose.header.frame_id = 'base_link'
                req.target_pose.header.stamp = rospy.Time.now()
                req.target_pose.pose.position.x = float(target_xyz[0])
                req.target_pose.pose.position.y = float(target_xyz[1])
                req.target_pose.pose.position.z = float(target_xyz[2])
                req.target_pose.pose.orientation.w = 1.0
                resp = self._ik_proxy(req)
                if resp.success and resp.solution.position:
                    return tuple(resp.solution.position[:6])
            except Exception as e:
                rospy.logdebug_throttle(5, "[ACO+RRT*] IK 服务调用失败: %s", str(e))
        return ik_target_local(target_xyz)

    def fk(self, joints):
        """关节角 -> 末端位置 [x,y,z] (仅位置)"""
        if self._fk_proxy is not None:
            try:
                req = ComputeFK.Request()
                req.joint_state = JointState()
                req.joint_state.name = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
                                        'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
                req.joint_state.position = list(joints)[:6]
                resp = self._fk_proxy(req)
                if resp.success and resp.ee_pose.pose.position:
                    p = resp.ee_pose.pose.position
                    return np.array([p.x, p.y, p.z])
            except Exception as e:
                rospy.logdebug_throttle(5, "[ACO+RRT*] FK 服务调用失败: %s", str(e))
        return np.array(UR5eFK.fk(joints))

    def get_current_ee_pose(self):
        """获取 motion_control 发布的当前末端位姿 (若已订阅)"""
        return self._ee_pose

    def set_current_ee_pose(self, pose):
        """由订阅回调设置"""
        self._ee_pose = pose


def point_in_aabb(p, center, half_extents):
    cx, cy, cz = center
    hx, hy, hz = half_extents
    return (cx - hx <= p[0] <= cx + hx and
            cy - hy <= p[1] <= cy + hy and
            cz - hz <= p[2] <= cz + hz)


def check_collision(joints, obstacles):
    poses = UR5eFK.fk_chain(joints)
    for center, half in obstacles:
        for p in poses:
            if point_in_aabb(p, center, half):
                return True
    return False


# =============================================================================
# 点云 -> 障碍物体素化
# =============================================================================
def pointcloud_to_obstacles(cloud_msg, voxel_res=0.05, frame_id="base_link",
                           bounds=None):
    """
    将点云体素化，每个非空体素转为 (center, half_extents)
    """
    if bounds is None:
        bounds = ((-0.5, 1.0), (-0.5, 0.5), (0.0, 0.8))
    bounds = tuple(tuple(float(b) for b in r) for r in bounds)
    voxels = {}
    half = voxel_res / 2.0
    for p in pc2.read_points(cloud_msg, skip_nans=True,
                             field_names=("x", "y", "z")):
        x, y, z = float(p[0]), float(p[1]), float(p[2])
        if not (bounds[0][0] <= x <= bounds[0][1] and
                bounds[1][0] <= y <= bounds[1][1] and
                bounds[2][0] <= z <= bounds[2][1]):
            continue
        cell = (int(x / voxel_res), int(y / voxel_res), int(z / voxel_res))
        voxels[cell] = (x, y, z)
    obstacles = []
    for cell, (cx, cy, cz) in voxels.items():
        obstacles.append(((cx, cy, cz), (half, half, half)))
    return obstacles


# =============================================================================
# ACO 阶段
# =============================================================================
class ACOPhase:
    def __init__(self, grid_res=0.1, bounds=None):
        self.res = grid_res
        self.bounds = bounds or ((-0.5, 1.0), (-0.5, 0.5), (0.0, 0.8))
        self.pheromone = {}
        self.obstacle_cells = set()
        self.aco_path_cells = []

    def _to_cell(self, x, y, z):
        bx, by, bz = self.bounds
        return (int((x - bx[0]) / self.res), int((y - by[0]) / self.res),
                int((z - bz[0]) / self.res))

    def _from_cell(self, cell):
        ix, iy, iz = cell
        bx, by, bz = self.bounds
        return (bx[0] + (ix + 0.5) * self.res, by[0] + (iy + 0.5) * self.res,
                bz[0] + (iz + 0.5) * self.res)

    def mark_obstacles(self, obstacles):
        for center, half in obstacles:
            cx, cy, cz = center
            hx, hy, hz = half
            for ix in range(int((cx - hx - self.bounds[0][0]) / self.res),
                           int((cx + hx - self.bounds[0][0]) / self.res) + 1):
                for iy in range(int((cy - hy - self.bounds[1][0]) / self.res),
                               int((cy + hy - self.bounds[1][0]) / self.res) + 1):
                    for iz in range(int((cz - hz - self.bounds[2][0]) / self.res),
                                   int((cz + hz - self.bounds[2][0]) / self.res) + 1):
                        self.obstacle_cells.add((ix, iy, iz))

    def _neighbors(self, cell):
        ix, iy, iz = cell
        for d in [(-1,0,0),(1,0,0),(0,-1,0),(0,1,0),(0,0,-1),(0,0,1)]:
            n = (ix+d[0], iy+d[1], iz+d[2])
            if n not in self.obstacle_cells:
                bx, by, bz = self.bounds
                gx, gy, gz = self._from_cell(n)
                if bx[0] <= gx <= bx[1] and by[0] <= gy <= by[1] and bz[0] <= gz <= bz[1]:
                    yield n

    def run(self, start_xyz, goal_xyz, n_ants=30, n_iters=50, rho=0.75, Q=1.0,
            greedy_prob=0.3, elite_deposit_ratio=2.0):
        """
        ACO 主循环。
        greedy_prob: 贪心步概率，约此概率下选择到 goal 最近的邻居
        elite_deposit_ratio: 精英路径额外沉积倍数（相对普通蚂蚁）
        """
        start_cell = self._to_cell(*start_xyz)
        goal_cell = self._to_cell(*goal_xyz)
        for c in [start_cell, goal_cell]:
            self.obstacle_cells.discard(c)
        tau0 = 1.0
        for c in self.obstacle_cells:
            self.pheromone[c] = 0.0
        self.pheromone[start_cell] = tau0
        self.pheromone[goal_cell] = tau0
        self.aco_path_cells = []
        goal_xyz_tuple = self._from_cell(goal_cell)

        for _ in range(n_iters):
            for _ in range(n_ants):
                path = [start_cell]
                current = start_cell
                visited = {start_cell}
                while current != goal_cell:
                    ns = list(self._neighbors(current))
                    if not ns:
                        break
                    ns_unvisited = [n for n in ns if n not in visited]
                    if not ns_unvisited:
                        break
                    # 贪心步：约 greedy_prob 概率选择到 goal 最近的未访问邻居
                    if random.random() < greedy_prob:
                        current = min(ns_unvisited, key=lambda n: sum(
                            (a - b) ** 2 for a, b in zip(
                                self._from_cell(n), goal_xyz_tuple)))
                    else:
                        probs = []
                        for n in ns_unvisited:
                            tau = self.pheromone.get(n, tau0)
                            eta = 1.0 / (1.0 + sqrt(sum((a - b) ** 2 for a, b in zip(
                                self._from_cell(n), goal_xyz_tuple))))
                            probs.append(tau ** 1.0 * eta ** 2.0)
                        s = sum(probs)
                        probs = [p / s for p in probs]
                        r = random.random()
                        for i, p in enumerate(probs):
                            r -= p
                            if r <= 0:
                                current = ns[i]
                                break
                    if current in visited:
                        break
                    visited.add(current)
                    path.append(current)
                if current == goal_cell and len(path) > 1:
                    deposit = Q / len(path)
                    for c in path:
                        self.pheromone[c] = self.pheromone.get(c, tau0) + deposit
                    if not self.aco_path_cells or len(path) < len(self.aco_path_cells):
                        self.aco_path_cells = path[:]
            # Elite Ant 逻辑：对当前最优路径额外沉积信息素
            if self.aco_path_cells:
                elite_deposit = (Q / len(self.aco_path_cells)) * (elite_deposit_ratio - 1.0)
                for c in self.aco_path_cells:
                    self.pheromone[c] = self.pheromone.get(c, tau0) + elite_deposit
            for c in list(self.pheromone.keys()):
                if c not in self.obstacle_cells:
                    self.pheromone[c] = max(tau0 * 0.1, self.pheromone[c] * rho)
        rospy.loginfo("[ACO] 信息素场构建完成，路径长度 %d", len(self.aco_path_cells))

    def get_pheromone(self, x, y, z):
        c = self._to_cell(x, y, z)
        return self.pheromone.get(c, 0.001)

    def get_path_xyz(self):
        return [self._from_cell(c) for c in self.aco_path_cells]


# =============================================================================
# RRT* 阶段
# =============================================================================
class RRTStarPhase:
    JOINT_NAMES = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
                   'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
    JOINT_LIMITS = [(-2*pi, 2*pi)] * 6

    def __init__(self, obstacles, pheromone_fn, step_size=0.15, max_iter=3000):
        self.obstacles = obstacles
        self.pheromone = pheromone_fn
        self.step = step_size
        self.max_iter = max_iter

    def _sample(self, goal_bias=0.1):
        if random.random() < goal_bias:
            return None
        if random.random() < 0.7:
            q = tuple(random.uniform(a, b) for a, b in self.JOINT_LIMITS)
            pos = UR5eFK.fk(q)
            tau = self.pheromone(pos[0], pos[1], pos[2])
            if random.random() < min(1.0, tau * 2.0):
                return q
        return tuple(random.uniform(a, b) for a, b in self.JOINT_LIMITS)

    def _dist(self, a, b):
        return sqrt(sum((x - y) ** 2 for x, y in zip(a, b)))

    def _steer(self, from_q, to_q):
        d = self._dist(from_q, to_q)
        if d <= self.step:
            return to_q
        t = self.step / d
        return tuple(f + t * (t_ - f) for f, t_ in zip(from_q, to_q))

    def _collision_free(self, qa, qb, n_checks=10):
        for i in range(1, n_checks):
            t = i / n_checks
            q = tuple(a + t * (b - a) for a, b in zip(qa, qb))
            if check_collision(q, self.obstacles):
                return False
        return not check_collision(qb, self.obstacles)

    def plan(self, start, goal):
        nodes = {start: {'cost': 0, 'parent': None}}
        for _ in range(self.max_iter):
            q_rand = self._sample(goal_bias=0.08)
            if q_rand is None:
                q_rand = goal
            nearest = min(nodes.keys(), key=lambda q: self._dist(q, q_rand))
            q_new = self._steer(nearest, q_rand)
            if not self._collision_free(nearest, q_new):
                continue
            cost_new = nodes[nearest]['cost'] + self._dist(nearest, q_new)
            near_radius = min(0.5, (3.0 * len(nodes) ** (-1.0/6)))
            near_nodes = [q for q in nodes if self._dist(q, q_new) < near_radius
                         and self._collision_free(q, q_new)]
            best_parent = nearest
            best_cost = cost_new
            for q_near in near_nodes:
                c = nodes[q_near]['cost'] + self._dist(q_near, q_new)
                if c < best_cost:
                    best_cost = c
                    best_parent = q_near
            nodes[q_new] = {'cost': best_cost, 'parent': best_parent}
            for q_near in near_nodes:
                if q_near == best_parent:
                    continue
                c = best_cost + self._dist(q_new, q_near)
                if c < nodes[q_near]['cost'] and self._collision_free(q_new, q_near):
                    nodes[q_near]['parent'] = q_new
                    nodes[q_near]['cost'] = c
            if self._dist(q_new, goal) < self.step and self._collision_free(q_new, goal):
                nodes[goal] = {'cost': nodes[q_new]['cost'] + self._dist(q_new, goal), 'parent': q_new}
                break
        if goal not in nodes:
            return None
        path = []
        p = goal
        while p is not None:
            path.append(p)
            p = nodes[p]['parent']
        path.reverse()
        rospy.loginfo("[RRT*] 找到路径，共 %d 个构型", len(path))
        return path


# =============================================================================
# RViz 可视化
# =============================================================================
def create_obstacle_markers(obstacles, frame_id, ns="obstacles"):
    arr = MarkerArray()
    for i, ((cx, cy, cz), (hx, hy, hz)) in enumerate(obstacles):
        m = Marker()
        m.header.frame_id = frame_id
        m.header.stamp = rospy.Time(0)  # 使用最新 TF，避免时序导致不显示
        m.ns = ns
        m.id = i
        m.type = Marker.CUBE
        m.action = Marker.ADD
        m.pose.position.x = cx
        m.pose.position.y = cy
        m.pose.position.z = cz
        m.pose.orientation.w = 1.0
        m.scale.x = 2 * hx
        m.scale.y = 2 * hy
        m.scale.z = 2 * hz
        m.color = ColorRGBA(0.8, 0.2, 0.2, 0.6)
        arr.markers.append(m)
    return arr


def create_path_marker(points_xyz, frame_id, ns, color, marker_id=0):
    """points_xyz: list of (x,y,z)"""
    m = Marker()
    m.header.frame_id = frame_id
    m.header.stamp = rospy.Time(0)  # 使用最新 TF，避免时序导致不显示
    m.ns = ns
    m.id = marker_id
    m.type = Marker.LINE_STRIP
    m.action = Marker.ADD
    m.scale.x = 0.02
    m.points = [Point(x=p[0], y=p[1], z=p[2]) for p in points_xyz]
    m.colors = [color] * len(m.points)
    return m


def create_joint_path_marker(path_joints, frame_id, ns, color, marker_id=0, kinematics_client=None):
    """path_joints: list of 6-tuples, 转为 workspace 路径点，优先使用 kinematics_client"""
    if kinematics_client is not None:
        xyz = [tuple(kinematics_client.fk(q)) for q in path_joints]
    else:
        xyz = [tuple(UR5eFK.fk(q)) for q in path_joints]
    return create_path_marker(xyz, frame_id, ns, color, marker_id)


def create_grasp_point_marker(x, y, z, frame_id, marker_id=0):
    """虚拟抓取点：球体 Marker"""
    m = Marker()
    m.header.frame_id = frame_id
    m.header.stamp = rospy.Time(0)  # 使用最新 TF，避免时序导致不显示
    m.ns = "virtual_grasp_point"
    m.id = marker_id
    m.type = Marker.SPHERE
    m.action = Marker.ADD
    m.pose.position.x = x
    m.pose.position.y = y
    m.pose.position.z = z
    m.pose.orientation.w = 1.0
    m.scale.x = 0.06
    m.scale.y = 0.06
    m.scale.z = 0.06
    m.color = ColorRGBA(1.0, 0.8, 0.0, 0.9)
    return m


# =============================================================================
# Gazebo 默认障碍物（桌面上箱子，与 arm_gazebo 的 obstacle_box 一致）
# =============================================================================
GAZEBO_DEFAULT_OBSTACLES = [
    ((0.5, 0.0, 0.4), (0.05, 0.2, 0.2)),
    ((0.0, 0.0, -0.06), (1.5, 1.5, 0.01)),
]

# 默认虚拟抓取点：Gazebo 箱子的后方 (x>0.55)，桌面上方
DEFAULT_VIRTUAL_GRASP_POINT = (0.68, 0.0, 0.55)


# =============================================================================
# 主节点
# =============================================================================
class ACO_RRTStarPlannerNode:
    def __init__(self):
        rospy.init_node('aco_rrtstar_planner', anonymous=False)
        self.pointcloud_topic = rospy.get_param('~pointcloud_topic', '/camera/depth/points')
        def _to_float_list(val, default):
            if isinstance(val, (list, tuple)):
                return [float(v) for v in val]
            if isinstance(val, dict) and 'x' in val:
                return [float(val.get('x', 0)), float(val.get('y', 0)), float(val.get('z', 0))]
            if isinstance(val, str):
                try:
                    parsed = ast.literal_eval(val)
                    return [float(v) for v in parsed]
                except (ValueError, TypeError):
                    pass
            return default

        def _to_float_list_6(val, default):
            if isinstance(val, (list, tuple)) and len(val) >= 6:
                return [float(v) for v in val[:6]]
            if isinstance(val, str):
                try:
                    parsed = ast.literal_eval(val)
                    return [float(v) for v in parsed[:6]] if len(parsed) >= 6 else default
                except (ValueError, TypeError):
                    pass
            return default
        voxel_res = float(rospy.get_param('~voxel_resolution', 0.05))
        wx = _to_float_list(rospy.get_param('~workspace_x', None), [-0.5, 1.0])
        wy = _to_float_list(rospy.get_param('~workspace_y', None), [-0.5, 0.5])
        wz = _to_float_list(rospy.get_param('~workspace_z', None), [0.0, 0.8])
        bounds = (tuple(wx), tuple(wy), tuple(wz))
        self.voxel_res = voxel_res
        self.bounds = bounds
        self.frame_id = rospy.get_param('~frame_id', 'base_link')

        self.obstacles = []
        self.obstacles_from_cloud = []
        self.last_plan_time = None

        self.obstacle_pub = rospy.Publisher('/path_planning/obstacles', MarkerArray, queue_size=1)
        self.aco_path_pub = rospy.Publisher('/path_planning/aco_path', Marker, queue_size=1)
        self.rrt_path_pub = rospy.Publisher('/path_planning/rrt_path', Marker, queue_size=1)
        self.grasp_point_pub = rospy.Publisher('/path_planning/virtual_grasp_point', Marker, queue_size=1)
        self.joint_state_pub = rospy.Publisher('/joint_states', JointState, queue_size=1)

        self.home_joints = _to_float_list_6(rospy.get_param('~home_joints', None), [0, -1.5708, 0, -1.5708, 0, 0])
        gp = rospy.get_param('~virtual_grasp_point', None)
        if gp is not None:
            lst = _to_float_list(gp, list(DEFAULT_VIRTUAL_GRASP_POINT))
            self.virtual_grasp_point = lst[:3] if len(lst) >= 3 else list(DEFAULT_VIRTUAL_GRASP_POINT)
        else:
            self.virtual_grasp_point = list(DEFAULT_VIRTUAL_GRASP_POINT)
        goal_from_param = rospy.get_param('~goal_joints', None)
        self.goal_joints = _to_float_list_6(goal_from_param, None)
        if self.goal_joints is None:
            self.goal_joints = self._compute_goal_from_grasp_point()
        self.joint_names = RRTStarPhase.JOINT_NAMES

        self.use_virtual_grasp_only = rospy.get_param('~use_virtual_grasp_only', True)
        self.publish_joint_state_enabled = rospy.get_param('~publish_joint_state', False)
        self.save_matplotlib_viz = rospy.get_param('~save_matplotlib_viz', True)
        use_motion_control = rospy.get_param('~use_motion_control_kinematics', True)
        self.kinematics_client = KinematicsClient(use_motion_control=use_motion_control)
        # 可选：订阅 motion_control 发布的当前末端位姿
        try:
            rospy.Subscriber(self.kinematics_client.EE_POSE_TOPIC, PoseStamped,
                             lambda msg: self.kinematics_client.set_current_ee_pose(msg), queue_size=1)
            rospy.loginfo_throttle(60, "[ACO+RRT*] 已订阅 motion_control EE 位姿: %s",
                                  self.kinematics_client.EE_POSE_TOPIC)
        except Exception:
            pass
        rospy.Subscriber(self.pointcloud_topic, PointCloud2, self.pointcloud_cb, queue_size=1)
        if self.publish_joint_state_enabled:
            rospy.Timer(rospy.Duration(0.1), self.publish_joint_state)
        rospy.Timer(rospy.Duration(0.5), self.visualization_loop)

        rospy.loginfo("[ACO+RRT*] 节点已启动，虚拟抓取点 (x,y,z): %s", self.virtual_grasp_point)
        rospy.loginfo("[ACO+RRT*] 目标关节角: %s", self.goal_joints)
        if self.use_virtual_grasp_only:
            rospy.Timer(rospy.Duration(1.0), self._once_run_planning, oneshot=True)

    def _once_run_planning(self, event):
        """仅调用一次：虚拟抓取点模式下启动时执行规划"""
        self.run_planning()

    def _compute_goal_from_grasp_point(self):
        """从虚拟抓取点 (x,y,z) 通过 IK 计算目标关节角，优先使用 motion_control"""
        q = self.kinematics_client.ik(self.virtual_grasp_point)
        if q is not None:
            rospy.loginfo("[ACO+RRT*] IK 成功，目标关节: %s", list(q))
            return list(q)
        rospy.logwarn("[ACO+RRT*] IK 失败，使用默认目标 (箱子后方姿态)")
        return [-1.5708, -1.5708, -1.5708, -1.5708, 1.5708, 0]

    def pointcloud_cb(self, msg):
        try:
            obs = pointcloud_to_obstacles(msg, self.voxel_res, self.frame_id, self.bounds)
            if obs:
                self.obstacles_from_cloud = obs
                rospy.loginfo_throttle(5, "[ACO+RRT*] 点云障碍物数量: %d", len(obs))
                #  throttle: 至少间隔 3 秒再规划
                now = rospy.Time.now()
                if self.last_plan_time is None or (now - self.last_plan_time).to_sec() > 3.0:
                    self.run_planning()
        except Exception as e:
            rospy.logerr("[ACO+RRT*] 点云处理错误: %s", str(e))

    def run_planning(self):
        if self.obstacles_from_cloud:
            ground = ((0.0, 0.0, -0.06), (1.5, 1.5, 0.01))
            self.obstacles = list(self.obstacles_from_cloud) + [ground]
        elif getattr(self, 'use_virtual_grasp_only', False):
            self.obstacles = list(GAZEBO_DEFAULT_OBSTACLES)
        else:
            return

        start = tuple(self.home_joints)
        goal = tuple(self.goal_joints)
        start_xyz = self.kinematics_client.fk(start)
        goal_xyz = self.kinematics_client.fk(goal)

        aco = ACOPhase(grid_res=0.08, bounds=self.bounds)
        aco.mark_obstacles(self.obstacles)
        aco.run(start_xyz, goal_xyz, n_ants=40, n_iters=30, greedy_prob=0.3, elite_deposit_ratio=2.0)

        pheromone_fn = lambda x, y, z: aco.get_pheromone(x, y, z)
        rrt = RRTStarPhase(self.obstacles, pheromone_fn, step_size=0.12, max_iter=4000)
        path = rrt.plan(start, goal)

        self.aco_path_xyz = aco.get_path_xyz()
        self.rrt_path_joints = path
        self.last_plan_time = rospy.Time.now()
        rospy.loginfo("[ACO+RRT*] 规划完成")
        if getattr(self, 'save_matplotlib_viz', False):
            self._save_matplotlib_viz()

    def _save_matplotlib_viz(self):
        """规划完成后保存 Matplotlib PNG（无需 RViz/X11）"""
        try:
            import sys
            import os
            _dir = os.path.dirname(os.path.abspath(__file__))
            if _dir not in sys.path:
                sys.path.insert(0, _dir)
            from plot_path_planning import plot_planning_results
            kc = self.kinematics_client
            rrt_xyz = [tuple(kc.fk(q)) for q in self.rrt_path_joints] if self.rrt_path_joints else []
            start_xyz = tuple(kc.fk(self.home_joints))
            out = rospy.get_param('~matplotlib_viz_output', '/workspace/viz_output/path_planning.png')
            plot_planning_results(
                self.obstacles, self.aco_path_xyz or [], rrt_xyz,
                self.virtual_grasp_point, start_xyz, out
            )
        except Exception as e:
            rospy.logwarn_throttle(60, "[ACO+RRT*] Matplotlib 保存失败: %s", str(e))

    def publish_joint_state(self, event):
        if not getattr(self, 'publish_joint_state_enabled', False):
            return
        js = JointState()
        js.header.stamp = rospy.Time.now()
        js.name = self.joint_names
        js.position = list(self.home_joints)
        self.joint_state_pub.publish(js)

    def visualization_loop(self, event):
        arr = create_obstacle_markers(self.obstacles, self.frame_id)
        self.obstacle_pub.publish(arr)
        gx, gy, gz = self.virtual_grasp_point[0], self.virtual_grasp_point[1], self.virtual_grasp_point[2]
        self.grasp_point_pub.publish(create_grasp_point_marker(gx, gy, gz, self.frame_id))
        if hasattr(self, 'aco_path_xyz') and self.aco_path_xyz:
            m = create_path_marker(self.aco_path_xyz, self.frame_id, 'aco',
                                  ColorRGBA(0, 1, 0, 1))
            self.aco_path_pub.publish(m)
        if hasattr(self, 'rrt_path_joints') and self.rrt_path_joints:
            m = create_joint_path_marker(self.rrt_path_joints, self.frame_id, 'rrtstar',
                                        ColorRGBA(0, 0, 1, 1), kinematics_client=self.kinematics_client)
            self.rrt_path_pub.publish(m)

    def run(self):
        rospy.spin()


if __name__ == '__main__':
    try:
        node = ACO_RRTStarPlannerNode()
        node.run()
    except rospy.ROSInterruptException:
        pass
