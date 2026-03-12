#!/usr/bin/env python3
"""
Motion Control Node - IK/FK/Dynamics for UR5e Robot
Author: Jiaxin Liang
Responsibilities:
  - Forward Kinematics (FK): Joint angles -> End-effector pose
  - Inverse Kinematics (IK): End-effector pose -> Joint angles
  - Dynamics: Compute joint torques, velocities, accelerations
  - Low-level motion execution interface with robot controller
"""

import rospy
import numpy as np
from typing import List, Optional, Tuple

from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from common_msgs.msg import MotionCommand, GraspResult
from moveit_msgs.srv import GetPositionIK
from moveit_msgs.msg import PositionIKRequest, RobotState, MoveItErrorCodes

import tf2_ros
import tf2_geometry_msgs
import actionlib


class MotionControlNode:
    """
    Motion Control Node for UR5e Robot
    
    This node provides low-level motion control services including:
    - IK/FK computation
    - Trajectory generation
    - Joint space and Cartesian space control
    - Velocity and force control interfaces
    """
    
    def __init__(self):
        rospy.init_node('motion_control', anonymous=False)
        rospy.loginfo("=" * 60)
        rospy.loginfo("Motion Control Node Initializing")
        rospy.loginfo("=" * 60)
        
        # Robot configuration
        self.robot_name = rospy.get_param('~robot_name', 'ur5e')
        self.base_frame = rospy.get_param('~base_frame', 'base_link')
        self.ee_frame = rospy.get_param('~ee_frame', 'tool0')
        
        # Control parameters
        self.joint_names = rospy.get_param('~joint_names', [
            'shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
            'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint'
        ])
        self.num_joints = len(self.joint_names)
        
        # Current robot state (positions in same order as joint_names)
        self.current_joint_state = None
        self.current_joint_positions = None  # List[float] in joint_names order
        self.current_ee_pose = None
        
        # UR5e DH parameters (standard order j1..j6, from UR documentation)
        self.dh_params = {
            'a': [0, -0.425, -0.39225, 0, 0, 0],
            'd': [0.1625, 0, 0, 0.1333, 0.0997, 0.0996],
            'alpha': [np.pi/2, 0, 0, np.pi/2, -np.pi/2, 0],
            'offset': [0, -np.pi/2, 0, -np.pi/2, 0, 0]
        }
        self.joint_limits = {
            'shoulder_pan_joint': [-np.pi, np.pi],
            'shoulder_lift_joint': [-np.pi, np.pi],
            'elbow_joint': [-np.pi, np.pi],
            'wrist_1_joint': [-np.pi, np.pi],
            'wrist_2_joint': [-np.pi, np.pi],
            'wrist_3_joint': [-np.pi, np.pi]
        }
        
        # TF2 for transformations
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        
        # MoveIt IK: service, group, planning frame and tip link (must match your MoveIt config)
        self.moveit_ik_service = rospy.get_param('~moveit_ik_service', '/compute_ik')
        self.moveit_ik_group = rospy.get_param('~moveit_ik_group', 'manipulator')
        self.moveit_planning_frame = rospy.get_param('~moveit_planning_frame', 'world')
        self.moveit_ik_link = rospy.get_param('~moveit_ik_link', 'gripper_tip_link')
        self._moveit_ik_proxy = None  # lazy init when first needed

        # Action client for trajectory execution (Gazebo UR5e controller)
        self.trajectory_client = actionlib.SimpleActionClient(
            '/pos_joint_traj_controller/follow_joint_trajectory',
            FollowJointTrajectoryAction
        )
        rospy.loginfo("[MotionControl] Waiting for trajectory action server...")
        if not self.trajectory_client.wait_for_server(timeout=rospy.Duration(5.0)):
            rospy.logwarn("[MotionControl] Trajectory action server not found. Gazebo UR5e may not be running.")
        
        # ROS interfaces
        self._setup_subscribers()
        self._setup_publishers()
        self._setup_services()
        
        # Communication test: wait for first joint_states from Gazebo
        rospy.loginfo("[MotionControl] Communication test: waiting for /joint_states (timeout 10s)...")
        try:
            msg = rospy.wait_for_message('/joint_states', JointState, timeout=10.0)
            positions = []
            for name in self.joint_names:
                if name in msg.name:
                    positions.append(msg.position[msg.name.index(name)])
            if len(positions) == self.num_joints:
                rospy.loginfo("[MotionControl] Communication OK (joint_states received from Gazebo)")
            else:
                rospy.logwarn("[MotionControl] Communication test: joint_states received but missing joints (got %d, need %d)" % (len(positions), self.num_joints))
        except rospy.ROSException as e:
            rospy.logwarn("[MotionControl] Communication test FAILED: %s. Check Gazebo and /joint_states." % str(e))
        
        rospy.loginfo("Motion Control Node Ready")
        rospy.loginfo("=" * 60)


    def _setup_subscribers(self):
        """Setup ROS subscribers"""
        self.joint_state_sub = rospy.Subscriber(
            '/joint_states',
            JointState,
            self.joint_state_callback,
            queue_size=1
        )
        
        self.motion_command_sub = rospy.Subscriber(
            '/motion/command',
            MotionCommand,
            self.motion_command_callback,
            queue_size=10
        )


    def _setup_publishers(self):
        """Setup ROS publishers"""
        self.motion_result_pub = rospy.Publisher(
            '/motion/result',
            GraspResult,
            queue_size=10
        )
        
        self.trajectory_pub = rospy.Publisher(
            '/joint_trajectory',
            JointTrajectory,
            queue_size=10
        )


    def _setup_services(self):
        """
        Setup ROS services for synchronous IK/FK queries
        
        TODO: Implement service servers for:
        - FK service: joint_angles -> end_effector_pose
        - IK service: end_effector_pose -> joint_angles
        - Jacobian service: compute Jacobian matrix
        """
        pass


    def _rpy_to_quaternion(self, roll: float, pitch: float, yaw: float) -> List[float]:
        """Convert RPY Euler angles to quaternion [qx, qy, qz, qw]"""
        cy = np.cos(yaw * 0.5)
        sy = np.sin(yaw * 0.5)
        cp = np.cos(pitch * 0.5)
        sp = np.sin(pitch * 0.5)
        cr = np.cos(roll * 0.5)
        sr = np.sin(roll * 0.5)
        qw = cr * cp * cy + sr * sp * sy
        qx = sr * cp * cy - cr * sp * sy
        qy = cr * sp * cy + sr * cp * sy
        qz = cr * cp * sy - sr * sp * cy
        return [qx, qy, qz, qw]

    def _fk_matrix(self, joint_angles: List[float]) -> Optional[np.ndarray]:
        """Forward kinematics returning 4x4 transform matrix (base_link -> tool0)."""
        if joint_angles is None or len(joint_angles) != self.num_joints:
            return None
        a, d, alpha, offset = self.dh_params['a'], self.dh_params['d'], self.dh_params['alpha'], self.dh_params['offset']
        T = np.eye(4)
        for i in range(6):
            theta = joint_angles[i] + offset[i]
            ct, st = np.cos(theta), np.sin(theta)
            ca, sa = np.cos(alpha[i]), np.sin(alpha[i])
            A = np.array([
                [ct, -st*ca, st*sa, a[i]*ct],
                [st, ct*ca, -ct*sa, a[i]*st],
                [0, sa, ca, d[i]],
                [0, 0, 0, 1]
            ])
            T = T @ A
        return T

    def _quat_to_rotation_matrix(self, q: List[float]) -> np.ndarray:
        """Quaternion [qx,qy,qz,qw] -> 3x3 rotation matrix."""
        qx, qy, qz, qw = float(q[0]), float(q[1]), float(q[2]), float(q[3])
        return np.array([
            [1 - 2*(qy**2 + qz**2), 2*(qx*qy - qz*qw), 2*(qx*qz + qy*qw)],
            [2*(qx*qy + qz*qw), 1 - 2*(qx**2 + qz**2), 2*(qy*qz - qx*qw)],
            [2*(qx*qz - qy*qw), 2*(qy*qz + qx*qw), 1 - 2*(qx**2 + qy**2)]
        ])

    def _pose_to_matrix(self, pose: Pose) -> np.ndarray:
        """Geometry Pose (position + quaternion) -> 4x4 transform."""
        T = np.eye(4)
        T[0:3, 0:3] = self._quat_to_rotation_matrix([
            pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w])
        T[0, 3], T[1, 3], T[2, 3] = pose.position.x, pose.position.y, pose.position.z
        return T

    def _rotation_to_axis_angle(self, R: np.ndarray) -> np.ndarray:
        """Rotation matrix 3x3 -> axis-angle vector (length = angle in rad)."""
        angle = np.arccos(np.clip((np.trace(R) - 1) / 2, -1, 1))
        if angle < 1e-6:
            return np.zeros(3)
        axis = np.array([R[2, 1] - R[1, 2], R[0, 2] - R[2, 0], R[1, 0] - R[0, 1]])
        return axis * (angle / (2 * np.sin(angle)))

    def _compute_jacobian_numerical(self, q: List[float], eps: float = 1e-6) -> np.ndarray:
        """6x6 Jacobian: [linear velocity; angular velocity] w.r.t. joint angles."""
        J = np.zeros((6, self.num_joints))
        T0 = self._fk_matrix(q)
        if T0 is None:
            return J
        p0 = T0[0:3, 3]
        R0 = T0[0:3, 0:3]
        for j in range(self.num_joints):
            qp = list(q)
            qp[j] += eps
            T1 = self._fk_matrix(qp)
            if T1 is None:
                continue
            J[0:3, j] = (T1[0:3, 3] - p0) / eps
            R1 = T1[0:3, 0:3]
            J[3:6, j] = self._rotation_to_axis_angle(R1 @ R0.T) / eps
        return J

    def _get_moveit_ik_proxy(self):
        """Lazy init and return MoveIt /compute_ik service proxy, or None if unavailable."""
        if self._moveit_ik_proxy is not None:
            return self._moveit_ik_proxy
        try:
            rospy.wait_for_service(self.moveit_ik_service, timeout=0.5)
            self._moveit_ik_proxy = rospy.ServiceProxy(self.moveit_ik_service, GetPositionIK)
            rospy.loginfo("[MotionControl] MoveIt IK service connected: %s", self.moveit_ik_service)
            return self._moveit_ik_proxy
        except (rospy.ROSException, rospy.ROSInterruptException):
            return None

    # =========================================================================
    # Callbacks
    # =========================================================================

    def joint_state_callback(self, msg: JointState):
        """Update current joint state (positions in joint_names order)."""
        self.current_joint_state = msg
        positions = []
        for name in self.joint_names:
            if name in msg.name:
                idx = msg.name.index(name)
                positions.append(msg.position[idx])
        if len(positions) == self.num_joints:
            self.current_joint_positions = positions
            self.current_ee_pose = self.compute_forward_kinematics(positions)


    def motion_command_callback(self, msg: MotionCommand):
        """
        Execute motion command from higher-level planner
        
        Args:
            msg: MotionCommand specifying desired motion
            
        TODO: Implement command execution logic:
        - Parse command type (move_to_pose, move_to_joint, etc.)
        - Generate appropriate trajectory
        - Execute motion
        - Publish result
        """
        rospy.loginfo(f"[MotionControl] Received command: {msg.command_type}")
        
        try:
            if msg.command_type == MotionCommand.MOVE_TO_POSE:
                self._execute_cartesian_motion(msg.target_pose, msg)
            elif msg.command_type == MotionCommand.MOVE_TO_JOINT:
                joints = list(msg.joint_positions) if hasattr(msg.joint_positions, '__iter__') else []
                if len(joints) != self.num_joints:
                    self._publish_motion_result(GraspResult.EXECUTION_FAILED, "Invalid joint_positions length")
                else:
                    self._execute_joint_motion(joints, msg)
            elif msg.command_type == MotionCommand.EXECUTE_TRAJECTORY:
                self._execute_trajectory(msg.trajectory)
            elif msg.command_type == MotionCommand.STOP:
                self._stop_motion()
            elif msg.command_type == MotionCommand.HOME:
                self._move_to_home()
            else:
                rospy.logwarn(f"[MotionControl] Unknown command type: {msg.command_type}")
                self._publish_motion_result(GraspResult.EXECUTION_FAILED, f"Unknown command: {msg.command_type}")
        except Exception as e:
            rospy.logerr(f"[MotionControl] Motion execution failed: {e}")
            self._publish_motion_result(GraspResult.EXECUTION_FAILED, str(e))


    # =========================================================================
    # Kinematics - Forward Kinematics
    # =========================================================================

    def compute_forward_kinematics(self, joint_angles: List[float]) -> Optional[PoseStamped]:
        """
        Compute forward kinematics: joint angles -> end-effector pose (UR5e DH, standard order).
        """
        if joint_angles is None or len(joint_angles) != self.num_joints:
            return None
        a = self.dh_params['a']
        d = self.dh_params['d']
        alpha = self.dh_params['alpha']
        offset = self.dh_params['offset']
        T = np.eye(4)
        for i in range(6):
            theta = joint_angles[i] + offset[i]
            ct, st = np.cos(theta), np.sin(theta)
            ca, sa = np.cos(alpha[i]), np.sin(alpha[i])
            A = np.array([
                [ct, -st*ca, st*sa, a[i]*ct],
                [st, ct*ca, -ct*sa, a[i]*st],
                [0, sa, ca, d[i]],
                [0, 0, 0, 1]
            ])
            T = T @ A
        x, y, z = T[0, 3], T[1, 3], T[2, 3]
        roll = np.arctan2(T[2, 1], T[2, 2])
        pitch = np.arctan2(-T[2, 0], np.sqrt(T[2, 1]**2 + T[2, 2]**2))
        yaw = np.arctan2(T[1, 0], T[0, 0])
        q = self._rpy_to_quaternion(roll, pitch, yaw)
        pose = PoseStamped()
        pose.header.frame_id = self.base_frame
        pose.header.stamp = rospy.Time.now()
        pose.pose.position = Point(x, y, z)
        pose.pose.orientation = Quaternion(q[0], q[1], q[2], q[3])
        return pose


    # =========================================================================
    # Kinematics - Inverse Kinematics
    # =========================================================================

    def compute_inverse_kinematics(
        self,
        target_pose: PoseStamped,
        seed_joints: Optional[List[float]] = None
    ) -> Optional[List[float]]:
        """
        Inverse kinematics: target pose -> joint angles.
        Prefer MoveIt /compute_ik service; fall back to numerical IK if unavailable or failed.
        """
        if target_pose.header.frame_id != self.base_frame:
            transformed = self._transform_pose(target_pose, self.base_frame)
            if transformed is None:
                rospy.logerr("[MotionControl] Failed to transform target pose to base_frame")
                return None
            target_pose = transformed

        # Try MoveIt IK first (pose must be in MoveIt's planning frame and for its tip link)
        proxy = self._get_moveit_ik_proxy()
        if proxy is not None:
            pose_for_moveit = self._transform_pose(target_pose, self.moveit_planning_frame)
            if pose_for_moveit is None:
                rospy.logwarn("[MotionControl] Cannot transform pose to MoveIt planning_frame=%s, fallback to numerical", self.moveit_planning_frame)
            else:
                req = PositionIKRequest()
                req.group_name = self.moveit_ik_group
                req.pose_stamped = pose_for_moveit
                req.ik_link_name = self.moveit_ik_link
                req.avoid_collisions = False
                req.robot_state = RobotState()
                req.robot_state.is_diff = True
                req.robot_state.joint_state = JointState()
                req.robot_state.joint_state.name = list(self.joint_names)
                seed = seed_joints if (seed_joints is not None and len(seed_joints) == self.num_joints) else self.current_joint_positions
                req.robot_state.joint_state.position = list(seed) if seed else [0.0, -1.57, 1.57, -1.57, -1.57, 0.0]
                req.timeout = rospy.Duration(2.0)   # 0.5s was too short for some poses
                try:
                    resp = proxy(req)
                    if resp.error_code.val == MoveItErrorCodes.SUCCESS and resp.solution.joint_state.name:
                        # Map solution to self.joint_names order
                        sol_map = dict(zip(resp.solution.joint_state.name, resp.solution.joint_state.position))
                        out = []
                        for name in self.joint_names:
                            if name in sol_map:
                                out.append(sol_map[name])
                            else:
                                rospy.logwarn("[MotionControl] MoveIt IK solution missing joint %s, fallback to numerical", name)
                                return self._compute_ik_numerical(target_pose, seed_joints)
                        if len(out) == self.num_joints:
                            rospy.logdebug("[MotionControl] IK from MoveIt")
                            return out
                    else:
                        rospy.logwarn("[MotionControl] MoveIt IK failed (error_code=%d, -31=no solution), fallback to numerical", resp.error_code.val)
                except Exception as e:
                    rospy.logwarn("[MotionControl] MoveIt IK call failed: %s, fallback to numerical", e)

        return self._compute_ik_numerical(target_pose, seed_joints)

    def _compute_ik_numerical(
        self,
        target_pose: PoseStamped,
        seed_joints: Optional[List[float]] = None
    ) -> Optional[List[float]]:
        """Numerical IK (Jacobian pseudo-inverse). Used when MoveIt is unavailable or fails."""
        T_target = self._pose_to_matrix(target_pose.pose)
        p_target = T_target[0:3, 3]
        R_target = T_target[0:3, 0:3]
        if seed_joints is not None and len(seed_joints) == self.num_joints:
            q = list(seed_joints)
        elif self.current_joint_positions is not None:
            q = list(self.current_joint_positions)
        else:
            q = [0.0, -1.57, 1.57, -1.57, -1.57, 0.0]
        step = 0.4
        pos_tol, ori_tol = 2e-3, 2e-2
        max_iter = 120
        lambda_damp = 0.02
        for _ in range(max_iter):
            T = self._fk_matrix(q)
            if T is None:
                return None
            p = T[0:3, 3]
            R = T[0:3, 0:3]
            pos_err = p_target - p
            ori_err = self._rotation_to_axis_angle(R_target @ R.T)
            err = np.concatenate([pos_err, ori_err])
            if np.linalg.norm(pos_err) < pos_tol and np.linalg.norm(ori_err) < ori_tol:
                return q
            J = self._compute_jacobian_numerical(q)
            try:
                JJt = J @ J.T + (lambda_damp ** 2) * np.eye(6)
                dq = J.T @ np.linalg.solve(JJt, err)
            except Exception:
                return None
            dq_norm = np.linalg.norm(dq)
            if dq_norm > 0.5:
                dq = dq * (0.5 / dq_norm)
            q = list(np.array(q) + step * dq)
            for i, name in enumerate(self.joint_names):
                lo, hi = self.joint_limits.get(name, [-np.pi, np.pi])
                q[i] = np.clip(q[i], lo, hi)
        rospy.logwarn("[MotionControl] Numerical IK did not converge (max iterations)")
        return None


    def compute_ik_with_collision_check(
        self,
        target_pose: PoseStamped,
        seed_joints: Optional[List[float]] = None
    ) -> Tuple[Optional[List[float]], bool]:
        """
        Compute IK with collision checking
        
        Args:
            target_pose: Desired pose
            seed_joints: IK seed
            
        Returns:
            (joint_angles, is_valid): Joint solution and validity flag
            
        TODO: Implement IK with collision checking
        - Compute IK solution
        - Check if solution is collision-free
        - If collision detected, try alternative IK solutions (UR5e has up to 8 solutions)
        """
        joint_solution = self.compute_inverse_kinematics(target_pose, seed_joints)
        
        if joint_solution is None:
            return None, False
        
        # TODO: Add collision checking
        is_collision_free = True  # Placeholder
        
        return joint_solution, is_collision_free


    # =========================================================================
    # Dynamics
    # =========================================================================

    def compute_jacobian(self, joint_angles: List[float]) -> Optional[np.ndarray]:
        """
        Compute geometric Jacobian matrix at given configuration
        
        Args:
            joint_angles: Joint configuration
            
        Returns:
            6x6 Jacobian matrix (linear and angular velocity)
            
        TODO: Implement Jacobian computation
        - Can be derived from FK using numerical differentiation
        - Or use analytical Jacobian from kinematics library
        
        Used for:
        - Cartesian velocity control: v_ee = J * q_dot
        - Force control: tau = J^T * F_ext
        """
        # TODO: Implement Jacobian calculation
        return None


    def compute_inverse_dynamics(
        self,
        joint_positions: List[float],
        joint_velocities: List[float],
        joint_accelerations: List[float]
    ) -> Optional[List[float]]:
        """
        Compute inverse dynamics: accelerations -> joint torques
        
        Args:
            joint_positions: Joint angles
            joint_velocities: Joint velocities
            joint_accelerations: Desired joint accelerations
            
        Returns:
            List of joint torques required
            
        TODO: Implement inverse dynamics
        - Use recursive Newton-Euler algorithm or
        - Use dynamics library (KDL, PyBullet, or robot-specific)
        
        Useful for feedforward torque control
        """
        # TODO: Implement inverse dynamics
        return None


    # =========================================================================
    # Motion Execution
    # =========================================================================

    def _execute_cartesian_motion(self, target_pose: PoseStamped, cmd: MotionCommand):
        """Execute Cartesian motion: IK then joint motion."""
        rospy.loginfo("[MotionControl] Executing Cartesian motion...")
        target_joints = self.compute_inverse_kinematics(target_pose)
        if target_joints is None:
            self._publish_motion_result(GraspResult.EXECUTION_FAILED, "IK failed for target pose")
            return
        # Base duration 2.0s for small teach-pendant steps (e.g. 10mm in 2s ~= 5mm/s); scale by max_velocity
        duration = 2.0
        if cmd.max_velocity > 0:
            duration = max(0.8, 2.0 * (1.0 / max(cmd.max_velocity, 0.01)))
        success = self._execute_joint_motion_impl(target_joints, duration)
        if success:
            self._publish_motion_result(GraspResult.SUCCESS, "Cartesian motion completed")
        else:
            self._publish_motion_result(GraspResult.EXECUTION_FAILED, "Cartesian motion failed")

    def _execute_joint_motion(self, target_joints: List[float], cmd: MotionCommand):
        """Execute joint space motion."""
        rospy.loginfo("[MotionControl] Executing joint motion...")
        if not self._check_joint_limits(target_joints):
            self._publish_motion_result(GraspResult.UNREACHABLE, "Joint limit violation")
            return
        duration = 4.0
        if cmd.max_velocity > 0:
            duration = max(1.0, 4.0 * (1.0 / max(cmd.max_velocity, 0.01)))
        success = self._execute_joint_motion_impl(target_joints, duration)
        if success:
            self._publish_motion_result(GraspResult.SUCCESS, "Joint motion completed")
        else:
            self._publish_motion_result(GraspResult.EXECUTION_FAILED, "Joint motion failed")

    def _execute_joint_motion_impl(self, target_joints: List[float], duration: float) -> bool:
        """Execute joint motion via action client. Single-point goal (same as test_ur5e_control). Returns True on success."""
        if self.current_joint_positions is None:
            rospy.logerr("[MotionControl] No current joint state")
            return False
        if len(target_joints) != self.num_joints:
            rospy.logerr("[MotionControl] Invalid joint count")
            return False
        # Match test_ur5e_control: single point, positions + time_from_start only (no vel/acc)
        goal = FollowJointTrajectoryGoal()
        goal.trajectory = JointTrajectory()
        goal.trajectory.joint_names = self.joint_names
        point = JointTrajectoryPoint()
        point.positions = list(target_joints)
        point.time_from_start = rospy.Duration(duration)
        goal.trajectory.points.append(point)
        self.trajectory_client.send_goal(goal)
        if self.trajectory_client.wait_for_result(timeout=rospy.Duration(duration + 5.0)):
            return self.trajectory_client.get_result() is not None
        self.trajectory_client.cancel_goal()
        rospy.logerr("[MotionControl] Motion timed out")
        return False

    def _execute_trajectory(self, trajectory: JointTrajectory):
        """Execute pre-computed trajectory."""
        rospy.loginfo("[MotionControl] Executing trajectory...")
        if len(trajectory.points) == 0:
            self._publish_motion_result(GraspResult.EXECUTION_FAILED, "Empty trajectory")
            return
        if not trajectory.joint_names:
            trajectory.joint_names = self.joint_names
        goal = FollowJointTrajectoryGoal()
        goal.trajectory = trajectory
        self.trajectory_client.send_goal(goal)
        if self.trajectory_client.wait_for_result(timeout=rospy.Duration(60.0)):
            self._publish_motion_result(GraspResult.SUCCESS, "Trajectory completed")
        else:
            self.trajectory_client.cancel_goal()
            self._publish_motion_result(GraspResult.EXECUTION_FAILED, "Trajectory failed")

    def _stop_motion(self):
        """Emergency stop: cancel all goals."""
        rospy.logwarn("[MotionControl] Stopping motion!")
        self.trajectory_client.cancel_all_goals()
        self._publish_motion_result(GraspResult.SUCCESS, "Motion stopped")

    def _move_to_home(self):
        """Move to home configuration (standard joint order)."""
        rospy.loginfo("[MotionControl] Moving to home position...")
        home_joints = [3.4050, -0.4451, 0.1568, -2.8511, -3.1306, 0.2741]
        success = self._execute_joint_motion_impl(home_joints, 4.0)
        if success:
            self._publish_motion_result(GraspResult.SUCCESS, "Home position reached")
        else:
            self._publish_motion_result(GraspResult.EXECUTION_FAILED, "Home motion failed")


    # =========================================================================
    # Trajectory Generation
    # =========================================================================

    def generate_joint_trajectory(
        self,
        start_joints: List[float],
        goal_joints: List[float],
        duration: float,
        max_velocity: float = 1.0,
        max_acceleration: float = 1.0,
        num_points: int = 50
    ) -> JointTrajectory:
        """Generate smooth joint trajectory (cubic interpolation)."""
        trajectory = JointTrajectory()
        trajectory.joint_names = self.joint_names
        for i in range(num_points + 1):
            t = (i / num_points) * duration
            point = JointTrajectoryPoint()
            point.positions = []
            point.velocities = []
            point.accelerations = []
            for j in range(self.num_joints):
                a0, a1 = start_joints[j], 0.0
                a2 = 3.0 * (goal_joints[j] - start_joints[j]) / (duration ** 2)
                a3 = -2.0 * (goal_joints[j] - start_joints[j]) / (duration ** 3)
                point.positions.append(a0 + a1*t + a2*t**2 + a3*t**3)
                point.velocities.append(a1 + 2*a2*t + 3*a3*t**2)
                point.accelerations.append(2*a2 + 6*a3*t)
            point.time_from_start = rospy.Duration(t)
            trajectory.points.append(point)
        return trajectory


    def generate_cartesian_path(
        self,
        waypoints: List[PoseStamped],
        step_size: float = 0.01
    ) -> Optional[JointTrajectory]:
        """
        Generate trajectory through Cartesian waypoints
        
        Args:
            waypoints: List of Cartesian poses to pass through
            step_size: Step size for intermediate waypoints (meters)
            
        Returns:
            JointTrajectory in joint space
            
        TODO: Implement Cartesian path generation
        Steps:
        1. Interpolate between waypoints with fixed step size
        2. Compute IK for each intermediate point
        3. Check for IK failures and trajectory continuity
        4. Return joint space trajectory
        
        Useful for straight-line motions, circular paths, etc.
        """
        # TODO: Implement Cartesian path generation
        return None


    # =========================================================================
    # Utilities
    # =========================================================================

    def _publish_motion_result(self, status: int, message: str):
        """Publish motion execution result (GraspResult)."""
        result = GraspResult()
        result.status = status
        result.message = message
        result.execution_time = 0.0
        self.motion_result_pub.publish(result)


    def _check_joint_limits(self, joint_angles: List[float]) -> bool:
        """Check if joint angles are within limits (same order as joint_names)."""
        if len(joint_angles) != self.num_joints:
            return False
        for i, name in enumerate(self.joint_names):
            if name not in self.joint_limits:
                continue
            lo, hi = self.joint_limits[name]
            if not (lo <= joint_angles[i] <= hi):
                rospy.logwarn(f"[MotionControl] {name} = {joint_angles[i]:.3f} outside [{lo:.3f}, {hi:.3f}]")
                return False
        return True


    def _transform_pose(self, pose: PoseStamped, target_frame: str) -> Optional[PoseStamped]:
        """
        Transform pose to target frame using TF2
        
        Args:
            pose: Input pose
            target_frame: Target reference frame
            
        Returns:
            Transformed pose or None if transform fails
        """
        try:
            transform = self.tf_buffer.lookup_transform(
                target_frame,
                pose.header.frame_id,
                rospy.Time(0),
                rospy.Duration(1.0)
            )
            return tf2_geometry_msgs.do_transform_pose(pose, transform)
        except (tf2_ros.LookupException, tf2_ros.ExtrapolationException) as e:
            rospy.logwarn(f"[MotionControl] TF transform failed: {e}")
            return None


    def run(self):
        """Main loop"""
        rospy.spin()


if __name__ == '__main__':
    try:
        node = MotionControlNode()
        node.run()
    except rospy.ROSInterruptException:
        pass
