## one_shot_planner 说明（一次性规划方案）

本模块实现 **一次性完整轨迹规划** 的封装，基于 `path_planning` 包中的 ACO + RRT* 规划器，将「从起点到目标」的路径转为 `JointTrajectory`，通过 `/motion/command` 交给 `motion_control` 执行。

**与 `ur5e_teach_pendant_ui` 一致**：不在此处直接调用 IK/FK，相信 `motion_control_node`；当前末端位姿由 **TF** 获取，当前关节由 **`/joint_states`** 获取，目标关节由 **配置/上层** 提供，仅通过 **`MotionCommand`** 与 motion_control 交互。

---

### 1. 相关文件

- **`nodes/one_shot_planner.py`**  
  一次性规划核心函数，供其他节点/业务逻辑调用；内部不调用 IK/FK。
- **`nodes/demo_one_shot_planning.py`**  
  演示节点：用 TF 取当前位姿、用 `/joint_states` 取当前关节、用参数取目标关节，规划后向 `/motion/command` 发布 `EXECUTE_TRAJECTORY`。

---

### 2. one_shot_planner 提供的核心接口

#### 2.1 `get_default_virtual_grasp_point()`

- 返回默认虚拟抓取点位置（base_link 系）`[x, y, z]`，与 `config/planning_config.yaml` 中的 `virtual_grasp_point` 一致（默认 `[0.35, 0.0, 0.2]`，桌面中心、桌面上表面高度），一般用于 ACO 的目标笛卡尔点。

#### 2.2 `plan_one_shot(...)`

**当前函数签名：**

```python
path_joints, trajectory = plan_one_shot(
    start_xyz,      # [x, y, z] 起点笛卡尔位置，由 TF 获取（同示教器）
    start_joints,   # 起点关节角 (6,)，由 /joint_states 获取
    goal_xyz,       # [x, y, z] 目标笛卡尔位置，用于 ACO，一般即 virtual_grasp_point
    goal_joints,    # 目标关节角 (6,)，由配置/上层提供，不在此做 IK
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
)
```

- **`start_xyz`**（必选）  
  - 起点笛卡尔位置 `[x, y, z]`（base_link）。  
  - 与示教器一致：由 **TF** `base_link` → `tcp_link`（如 `gripper_tip_link`）查当前末端位置得到。

- **`start_joints`**（必选）  
  - 起点关节角（6 维），与 `motion_control` 的关节顺序一致。  
  - 由 **`/joint_states`** 订阅得到。

- **`goal_xyz`**（必选）  
  - 目标笛卡尔位置 `[x, y, z]`，供 ACO 路径使用，一般用 `virtual_grasp_point` 或 `get_default_virtual_grasp_point()`。

- **`goal_joints`**（必选）  
  - 目标关节角（6 维）。  
  - **不在此做 IK**，由配置（如 `~goal_joints`）或上层逻辑提供，相信 motion_control 侧对目标姿态的约定。

- **`obstacles`**（可选）  
  - 格式：`[((cx, cy, cz), (hx, hy, hz)), ...]`。  
  - `None` 时使用 `GAZEBO_DEFAULT_OBSTACLES` 并加地面障碍。

- **返回值**  
  - `path_joints`：RRT* 关节路径，失败为 `None`。  
  - `trajectory`：`trajectory_msgs/JointTrajectory`，可封装为 `MotionCommand(EXECUTE_TRAJECTORY)` 发到 `/motion/command`。

#### 2.3 `joints_to_trajectory(path_joints, joint_names, time_step=0.5)`

- 将关节路径转为 `JointTrajectory`，`time_step` 为相邻点时间间隔（秒）。

#### 2.4 `build_motion_command_execute_trajectory(trajectory)`

- 构造 `MotionCommand`，`command_type=EXECUTE_TRAJECTORY`，`max_velocity=1.0`，`max_acceleration=1.0`，与示教器发布方式一致，可直接发布到 `/motion/command`。

---

### 3. demo_one_shot_planning 使用说明

`nodes/demo_one_shot_planning.py` 为可运行 Demo，逻辑与示教器对齐：TF + `/joint_states` + 参数，不调用 IK/FK。

#### 3.1 前置条件

- `roscore` 已启动。  
- `motion_control_node` 已运行（订阅 `/motion/command`）。  
- TF 中存在 `base_link` → `gripper_tip_link`（或 `~tcp_link` 指定）。  
- 已配置目标关节 `~goal_joints` 或使用默认 `~goal_joints_default`。  
- 可选：Gazebo/RViz 与 `/joint_states`、点云话题，便于完整验证。

#### 3.2 启动 Demo

```bash
cd ~/robocup_ur5e
source devel/setup.bash
rosrun path_planning demo_one_shot_planning.py
```

带参数示例：

```bash
# 指定目标关节（必选其一：参数或默认）
rosrun path_planning demo_one_shot_planning.py _goal_joints:="[0, -2.0, 1.2, -1.57, -1.57, 0]"

# 指定虚拟抓取点（用于 ACO 的 goal_xyz）
rosrun path_planning demo_one_shot_planning.py _virtual_grasp_point:="[0.35, 0.0, 0.2]"
```

#### 3.3 Demo 内部流程

1. **起点位姿**  
   - 与示教器一致：**TF** 查询 `base_link` → `tcp_link`，得到当前末端位置 `start_xyz`；失败则用 `~start_xyz`。

2. **起点关节**  
   - 订阅 **`/joint_states`** 取一次当前关节为 `start_joints`；超时则用 `~home_joints`。

3. **目标**  
   - `goal_xyz`：由 `~virtual_grasp_point` 或 `get_default_virtual_grasp_point()`。  
   - `goal_joints`：由 `~goal_joints` 解析；未设置则用 `~goal_joints_default`（不在此做 IK）。

4. **障碍物**  
   - 点云话题（如 `/camera/depth/points`）有数据则用 `pointcloud_to_obstacles`；否则用 `GAZEBO_DEFAULT_OBSTACLES`。

5. **规划与下发**  
   - 调用 `plan_one_shot(start_xyz, start_joints, goal_xyz, goal_joints, obstacles)`。  
   - 用 `build_motion_command_execute_trajectory(trajectory)` 构造 `MotionCommand`，发布到 **`/motion/command`**，由 motion_control 执行。

---

### 4. 在其他节点中复用一次性规划

与示教器相同用法：不直接使用 IK/FK，只使用 TF、`/joint_states` 和配置，并通过 `MotionCommand` 与 motion_control 交互。

1. 用 **TF** 取当前末端位置 `start_xyz`（如 `base_link` → `gripper_tip_link`）。  
2. 从 **`/joint_states`** 取当前关节 `start_joints`。  
3. 从 **配置或上层** 取目标关节 `goal_joints` 与目标笛卡尔点 `goal_xyz`（如 `virtual_grasp_point`）。  
4. 调用 `plan_one_shot(...)`，将返回的轨迹用 `build_motion_command_execute_trajectory` 发布到 `/motion/command`。

示例（伪代码）：

```python
import tf2_ros
from one_shot_planner import (
    plan_one_shot,
    get_default_virtual_grasp_point,
    build_motion_command_execute_trajectory,
)
from common_msgs.msg import MotionCommand

# 与示教器一致：TF 取当前位姿
tf_buffer = tf2_ros.Buffer()
tf_listener = tf2_ros.TransformListener(tf_buffer)
trans = tf_buffer.lookup_transform("base_link", "gripper_tip_link", rospy.Time(0), rospy.Duration(0.5))
start_xyz = [trans.transform.translation.x, trans.transform.translation.y, trans.transform.translation.z]

# /joint_states 取当前关节（实际代码中可订阅一次或从回调得到）
start_joints = [...]  # 6 维

# 目标：配置或上层提供，不在此做 IK
goal_xyz = get_default_virtual_grasp_point()  # 或自定义 [x, y, z]
goal_joints = rospy.get_param("~goal_joints", [0, -2.0, 1.2, -1.57, -1.57, 0])

path_joints, traj = plan_one_shot(
    start_xyz=start_xyz,
    start_joints=start_joints,
    goal_xyz=goal_xyz,
    goal_joints=goal_joints,
    obstacles=None,
)

if path_joints and traj:
    cmd = build_motion_command_execute_trajectory(traj)
    pub = rospy.Publisher("/motion/command", MotionCommand, queue_size=10)
    pub.publish(cmd)
```

这样在 RoboCup 行为树或任务规划节点中即可复用一次性规划，与示教器、motion_control 的用法保持一致。
