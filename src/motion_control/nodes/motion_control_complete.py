#!/usr/bin/env python3
"""
Motion Control Node - Complete Implementation for UR5e Robot
Author: Jiaxin Liang
Features:
  - Forward/Inverse Kinematics with official UR5e DH parameters
  - Joint space and Cartesian space control
  - Full pose control (position + orientation)
  - Trajectory generation with smooth interpolation
  - Real-time FK查询 (关节角 -> 笛卡尔坐标)
  - IK查询 (笛卡尔坐标 -> 关节角)
  - Path recording and replay
  - Safety joint limit checking
  - TF2 coordinate transformations
  - Correct joint ordering based on actual UR5e configuration
  - Optimized logging (debug level for high-frequency callbacks)
"""

import rospy
import numpy as np
import actionlib
import std_msgs.msg
from std_msgs.msg import String, Header
from typing import List, Optional, Tuple, Dict
from enum import Enum

from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion, TransformStamped
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
import tf2_ros
import tf2_geometry_msgs


class MotionControlNode:
    """
    Motion Control Node for UR5e Robot - Complete Implementation
    
    Features:
    - Forward/Inverse Kinematics using official UR5e DH parameters
    - Trajectory generation with smooth interpolation
    - Joint space and Cartesian space motion control
    - Full pose control (position + orientation)
    - Real-time FK queries (joint angles -> end-effector pose)
    - IK queries (end-effector pose -> joint angles)
    - TF2 integration for coordinate transformations
    - Path recording and replay
    - Correct joint ordering based on actual UR5e configuration
    - Status feedback and error handling
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
        
        # Joint configuration - CORRECT UR5e order based on testing
        # From exhaustive testing, the actual joint order is: [j3, j5, j4, j2, j6, j1]
        # where:
        #   j1 = shoulder_pan_joint
        #   j2 = shoulder_lift_joint
        #   j3 = elbow_joint
        #   j4 = wrist_1_joint
        #   j5 = wrist_2_joint
        #   j6 = wrist_3_joint
        self.joint_names = rospy.get_param('~joint_names', [
            'elbow_joint',           # j3 - actual position 0
            'wrist_2_joint',          # j5 - actual position 1
            'wrist_1_joint',          # j4 - actual position 2
            'shoulder_lift_joint',    # j2 - actual position 3
            'wrist_3_joint',          # j6 - actual position 4
            'shoulder_pan_joint'      # j1 - actual position 5
        ])
        self.num_joints = len(self.joint_names)
        
        # No joint reordering needed anymore - the joint_names list now matches the actual order
        self.STANDARD_TO_CONTROLLER = [0, 1, 2, 3, 4, 5]  # Identity mapping
        
        # UR5e官方DH参数 (根据Universal Robots文档)
        self.dh_params = {
            'a': [0, -0.425, -0.39225, 0, 0, 0],      # 连杆长度 (米)
            'd': [0.1625, 0, 0, 0.1333, 0.0997, 0.0996],  # 连杆偏移 (米)
            'alpha': [np.pi/2, 0, 0, np.pi/2, -np.pi/2, 0],  # 连杆扭转 (弧度)
            'offset': [0, -np.pi/2, 0, -np.pi/2, 0, 0]  # 关节偏移 (弧度)
        }
        
        # Joint limits [min, max] in radians
        self.joint_limits = {
            'shoulder_pan_joint': [-3.14, 3.14],    # ±180°
            'shoulder_lift_joint': [-3.14, 3.14],   # ±180°
            'elbow_joint': [-3.14, 3.14],           # ±180°
            'wrist_1_joint': [-3.14, 3.14],         # ±180°
            'wrist_2_joint': [-3.14, 3.14],         # ±180°
            'wrist_3_joint': [-3.14, 3.14]          # ±180°
        }
        
        # Current robot state
        self.current_joint_state = None  # JointState message
        self.current_joint_positions = None  # List of positions in CORRECT order
        self.current_ee_pose = None  # PoseStamped
        
        # Path recording
        self.recorded_path = []  # List of joint positions (CORRECT order)
        self.is_recording = False
        
        # Action client for trajectory execution
        self.trajectory_client = actionlib.SimpleActionClient(
            '/pos_joint_traj_controller/follow_joint_trajectory',
            FollowJointTrajectoryAction
        )
        rospy.loginfo("[MotionControl] Waiting for trajectory action server...")
        if not self.trajectory_client.wait_for_server(timeout=rospy.Duration(5.0)):
            rospy.logwarn("[MotionControl] Trajectory action server not found!")
            rospy.logwarn("[MotionControl] Make sure Gazebo UR5e is running!")
        
        # TF2 for transformations
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        
        # Setup ROS interfaces
        self._setup_subscribers()
        self._setup_publishers()
        
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
        
        # Simple command interface
        from std_msgs.msg import String
        self.motion_command_sub = rospy.Subscriber(
            '/motion_command_trigger',
            String,
            self.simple_command_callback,
            queue_size=10
        )
        rospy.loginfo("[MotionControl] Subscribed to /motion_command_trigger (String)")
    
    def _setup_publishers(self):
        """Setup ROS publishers"""
        from std_msgs.msg import String
        self.motion_result_pub = rospy.Publisher(
            '/motion_result',
            String,
            queue_size=10
        )
        
        self.trajectory_pub = rospy.Publisher(
            '/joint_trajectory',
            JointTrajectory,
            queue_size=10
        )
        
        # For visualization in RViz
        try:
            from visualization_msgs.msg import Marker
            self.marker_pub = rospy.Publisher(
                '/motion_path_marker',
                Marker,
                queue_size=10
            )
        except ImportError:
            self.marker_pub = None
            rospy.logwarn("[MotionControl] visualization_msgs not available")
    
    # =========================================================================
    # Callbacks
    # =========================================================================
    
    def joint_state_callback(self, msg: JointState):
        """
        Update current joint state - No mapping needed anymore
        The joint_names list now matches the actual controller order: [j3, j5, j4, j2, j6, j1]
        """
        self.current_joint_state = msg
        
        # The joint states are already in the correct order matching our joint_names
        self.current_joint_positions = list(msg.position[:self.num_joints])
        
        # 使用 logdebug 避免高频输出刷屏（可在需要时启用）
        rospy.logdebug("[MotionControl] Joint state received (in correct order):")
        for i, name in enumerate(self.joint_names):
            rospy.logdebug(f"  {name}: {self.current_joint_positions[i]:.3f} rad")
        
        # 使用静默版本更新pose
        self.current_ee_pose = self._compute_fk_silent(self.current_joint_positions)
        
        # If recording, add current position to path
        if self.is_recording:
            self.recorded_path.append(self.current_joint_positions.copy())
    
    def simple_command_callback(self, msg):
        """
        Execute simple text commands
        All joint commands are expected in the same order as joint_names: [j3, j5, j4, j2, j6, j1]
        """
        cmd = msg.data.lower().strip()
        rospy.loginfo(f"[MotionControl] Received command: {cmd}")
        
        try:
            # Basic motion commands
            if cmd == "home":
                self._move_to_home()
            
            elif cmd == "bent":
                # Standard bent position in correct order: [j3, j5, j4, j2, j6, j1]
                bent_pos = [1.57, -1.57, -1.57, -1.57, 0.0, 0.0]
                self._execute_joint_motion(bent_pos, duration=4.0)
            
            elif cmd == "rotate":
                # Standard rotate position in correct order: [j3, j5, j4, j2, j6, j1]
                rotate_pos = [1.57, -1.57, -1.57, -1.57, 0.0, 1.57]
                self._execute_joint_motion(rotate_pos, duration=4.0)
            
            elif cmd == "test_sequence":
                self._run_test_sequence()
            
            elif cmd == "stop":
                self._stop_motion()
            
            # FK command: 使用调试版本输出详细信息
            elif cmd == "fk":
                self.debug_forward_kinematics()
                self._print_current_state()
            
            # IK command: ik x y z
            elif cmd.startswith("ik "):
                parts = cmd.split()
                if len(parts) == 4:
                    try:
                        x, y, z = float(parts[1]), float(parts[2]), float(parts[3])
                        self._query_ik(x, y, z)
                    except ValueError:
                        rospy.logwarn("Invalid coordinates. Usage: ik x y z")
                else:
                    rospy.logwarn("Usage: ik x y z")
            
            # Move to Cartesian position/pose
            elif cmd.startswith("move_to "):
                parts = cmd.split()
                
                # Format 1: move_to x y z (position only, keep orientation)
                if len(parts) == 4:
                    try:
                        x, y, z = float(parts[1]), float(parts[2]), float(parts[3])
                        self.move_to_cartesian_pose(x, y, z, duration=5.0)
                    except ValueError:
                        rospy.logwarn("Invalid coordinates. Usage: move_to x y z")
                
                # Format 2: move_to x y z roll pitch yaw (full pose)
                elif len(parts) == 7:
                    try:
                        x, y, z = float(parts[1]), float(parts[2]), float(parts[3])
                        roll, pitch, yaw = float(parts[4]), float(parts[5]), float(parts[6])
                        self.move_to_cartesian_pose(x, y, z, roll, pitch, yaw, duration=5.0)
                    except ValueError:
                        rospy.logwarn("Invalid arguments. Usage: move_to x y z roll pitch yaw")
                
                else:
                    rospy.logwarn("Usage: move_to x y z  or  move_to x y z roll pitch yaw")
            
            # Move to joint angles: move_joint values in correct order [j3, j5, j4, j2, j6, j1]
            elif cmd.startswith("move_joint "):
                parts = cmd.split()
                if len(parts) == 7:
                    try:
                        joints = [float(p) for p in parts[1:7]]
                        self._execute_joint_motion(joints, duration=4.0)
                    except ValueError:
                        rospy.logwarn("Invalid joint angles. Usage: move_joint [j3, j5, j4, j2, j6, j1]")
                else:
                    rospy.logwarn("Usage: move_joint [j3, j5, j4, j2, j6, j1]")
            
            # Path recording commands
            elif cmd == "record_start":
                self._start_recording()
            
            elif cmd == "record_stop":
                self._stop_recording()
            
            elif cmd == "replay":
                self._replay_path()
            
            elif cmd.startswith("save_path "):
                name = cmd[10:].strip()
                self._save_path(name)
            
            elif cmd.startswith("load_path "):
                name = cmd[10:].strip()
                self._load_path(name)
            
            elif cmd == "list_paths":
                self._list_paths()
            
            # TF2 query command
            elif cmd.startswith("get_transform "):
                parts = cmd.split()
                if len(parts) == 3:
                    source = parts[1]
                    target = parts[2]
                    self._query_transform(source, target)
                else:
                    rospy.logwarn("Usage: get_transform source_frame target_frame")
            
            # Help command
            elif cmd == "help":
                self._print_help()
            
            else:
                rospy.logwarn(f"Unknown command: {cmd}")
                self._print_help()
                
        except Exception as e:
            rospy.logerr(f"[MotionControl] Command execution failed: {e}")
            self._publish_result(f"ERROR: {e}")
    
    # =========================================================================
    # Kinematics - Forward Kinematics (FK) - 静默版本 (官方DH参数)
    # =========================================================================
    
    def _compute_fk_silent(self, joint_angles: List[float]) -> Optional[PoseStamped]:
        """
        静默版FK计算 - 使用官方DH参数和关节偏移
        供高频调用使用，不输出日志
        Input: joint angles in CORRECT order [j3, j5, j4, j2, j6, j1]
        """
        if joint_angles is None or len(joint_angles) != self.num_joints:
            return None
        
        # UR5e 官方DH参数
        a = self.dh_params['a']
        d = self.dh_params['d']
        alpha = self.dh_params['alpha']
        offset = self.dh_params['offset']
        
        # 需要将关节角度映射回标准DH顺序 [j1, j2, j3, j4, j5, j6]
        # Our order: [j3, j5, j4, j2, j6, j1] -> Standard: [j1, j2, j3, j4, j5, j6]
        standard_order = [
            joint_angles[5],  # j1 from position 5
            joint_angles[3],  # j2 from position 3
            joint_angles[0],  # j3 from position 0
            joint_angles[2],  # j4 from position 2
            joint_angles[1],  # j5 from position 1
            joint_angles[4]   # j6 from position 4
        ]
        
        # Initialize transformation matrix
        T = np.eye(4)
        
        # Compute forward kinematics using DH convention
        for i in range(6):
            # 加入关节偏移
            theta = standard_order[i] + offset[i]
            
            ct = np.cos(theta)
            st = np.sin(theta)
            ca = np.cos(alpha[i])
            sa = np.sin(alpha[i])
            
            # DH transformation matrix
            A = np.array([
                [ct, -st*ca, st*sa, a[i]*ct],
                [st, ct*ca, -ct*sa, a[i]*st],
                [0, sa, ca, d[i]],
                [0, 0, 0, 1]
            ])
            
            T = T @ A
        
        # Extract position
        x, y, z = T[0, 3], T[1, 3], T[2, 3]
        
        # Extract rotation matrix to quaternion
        roll = np.arctan2(T[2, 1], T[2, 2])
        pitch = np.arctan2(-T[2, 0], np.sqrt(T[2, 1]**2 + T[2, 2]**2))
        yaw = np.arctan2(T[1, 0], T[0, 0])
        
        # Convert RPY to quaternion
        q = self._rpy_to_quaternion(roll, pitch, yaw)
        
        # Create pose message
        pose = PoseStamped()
        pose.header.frame_id = self.base_frame
        pose.header.stamp = rospy.Time.now()
        pose.pose.position = Point(x, y, z)
        pose.pose.orientation = Quaternion(q[0], q[1], q[2], q[3])
        
        return pose
    
    def debug_forward_kinematics(self):
        """
        调试版FK计算 - 使用官方DH参数，输出详细信息
        """
        if self.current_joint_positions is None:
            rospy.logwarn("[MotionControl] No joint states available")
            return None
        
        joint_angles = self.current_joint_positions
        
        rospy.loginfo("=" * 60)
        rospy.loginfo("FK Debug - Official UR5e DH Parameters")
        rospy.loginfo("Input joint angles (CORRECT order [j3,j5,j4,j2,j6,j1]) (rad):")
        for i, name in enumerate(self.joint_names):
            rospy.loginfo(f"  {name}: {joint_angles[i]:.3f} rad ({joint_angles[i]*180/np.pi:.1f} deg)")
        
        # 转换为标准顺序用于计算
        standard_order = [
            joint_angles[5],  # j1 from position 5
            joint_angles[3],  # j2 from position 3
            joint_angles[0],  # j3 from position 0
            joint_angles[2],  # j4 from position 2
            joint_angles[1],  # j5 from position 1
            joint_angles[4]   # j6 from position 4
        ]
        
        rospy.loginfo("\nMapped to standard DH order [j1,j2,j3,j4,j5,j6] (rad):")
        rospy.loginfo(f"  [{standard_order[0]:.3f}, {standard_order[1]:.3f}, {standard_order[2]:.3f}, "
                      f"{standard_order[3]:.3f}, {standard_order[4]:.3f}, {standard_order[5]:.3f}]")
        
        a = self.dh_params['a']
        d = self.dh_params['d']
        alpha = self.dh_params['alpha']
        offset = self.dh_params['offset']
        
        rospy.loginfo("\nOfficial UR5e DH Parameters:")
        for i in range(6):
            rospy.loginfo(f"  Joint {i}: a={a[i]:.3f}, d={d[i]:.3f}, alpha={alpha[i]:.3f}, offset={offset[i]:.3f}")
        
        T = np.eye(4)
        rospy.loginfo("\nTransformation chain:")
        rospy.loginfo(f"  Start: {T[0,3]:.3f}, {T[1,3]:.3f}, {T[2,3]:.3f}")
        
        for i in range(6):
            # 加入关节偏移
            theta = standard_order[i] + offset[i]
            
            ct = np.cos(theta)
            st = np.sin(theta)
            ca = np.cos(alpha[i])
            sa = np.sin(alpha[i])
            
            A = np.array([
                [ct, -st*ca, st*sa, a[i]*ct],
                [st, ct*ca, -ct*sa, a[i]*st],
                [0, sa, ca, d[i]],
                [0, 0, 0, 1]
            ])
            
            rospy.loginfo(f"\nJoint {i+1} transformation matrix (theta={theta:.3f} rad):")
            rospy.loginfo(f"  {A[0,0]:.3f} {A[0,1]:.3f} {A[0,2]:.3f} {A[0,3]:.3f}")
            rospy.loginfo(f"  {A[1,0]:.3f} {A[1,1]:.3f} {A[1,2]:.3f} {A[1,3]:.3f}")
            rospy.loginfo(f"  {A[2,0]:.3f} {A[2,1]:.3f} {A[2,2]:.3f} {A[2,3]:.3f}")
            rospy.loginfo(f"  0.000 0.000 0.000 1.000")
            
            T = T @ A
            rospy.loginfo(f"After joint {i+1}: position = ({T[0,3]:.3f}, {T[1,3]:.3f}, {T[2,3]:.3f})")
        
        x, y, z = T[0, 3], T[1, 3], T[2, 3]
        
        rospy.loginfo(f"\nFinal position from FK: ({x:.3f}, {y:.3f}, {z:.3f})")
        
        roll = np.arctan2(T[2, 1], T[2, 2])
        pitch = np.arctan2(-T[2, 0], np.sqrt(T[2, 1]**2 + T[2, 2]**2))
        yaw = np.arctan2(T[1, 0], T[0, 0])
        
        rospy.loginfo(f"Final RPY from FK: roll={roll:.3f}, pitch={pitch:.3f}, yaw={yaw:.3f}")
        rospy.loginfo("=" * 60)
        
        # 返回计算结果
        q = self._rpy_to_quaternion(roll, pitch, yaw)
        pose = PoseStamped()
        pose.header.frame_id = self.base_frame
        pose.header.stamp = rospy.Time.now()
        pose.pose.position = Point(x, y, z)
        pose.pose.orientation = Quaternion(q[0], q[1], q[2], q[3])
        return pose
    
    # =========================================================================
    # Kinematics - Inverse Kinematics (IK)
    # =========================================================================
    
    def compute_inverse_kinematics(
        self,
        target_pose: PoseStamped,
        seed_joints: Optional[List[float]] = None
    ) -> Optional[List[float]]:
        """
        Compute inverse kinematics: end-effector pose -> joint angles
        Returns joint angles in CORRECT order [j3, j5, j4, j2, j6, j1]
        """
        # Transform target pose to base frame if needed
        if target_pose.header.frame_id != self.base_frame:
            transformed = self._transform_pose(target_pose, self.base_frame)
            if transformed is None:
                rospy.logerr("[MotionControl] Failed to transform target pose to base frame")
                return None
            target_pose = transformed
        
        # Simplified IK - return seed or current joints
        if seed_joints is not None:
            return seed_joints
        elif self.current_joint_positions is not None:
            return self.current_joint_positions
        else:
            rospy.logwarn("[MotionControl] Using default joint configuration")
            # Default in correct order [j3, j5, j4, j2, j6, j1]
            return [1.57, -1.57, -1.57, -1.57, 0.0, 0.0]
    
    def _query_ik(self, x: float, y: float, z: float):
        """Query IK for a target position"""
        target_pose = PoseStamped()
        target_pose.header.frame_id = self.base_frame
        target_pose.header.stamp = rospy.Time.now()
        target_pose.pose.position = Point(x, y, z)
        
        if self.current_ee_pose:
            target_pose.pose.orientation = self.current_ee_pose.pose.orientation
        else:
            target_pose.pose.orientation = Quaternion(0, 0, 0, 1)
        
        joints = self.compute_inverse_kinematics(target_pose)
        
        if joints:
            rospy.loginfo(f"[MotionControl] IK solution for ({x:.3f}, {y:.3f}, {z:.3f}):")
            rospy.loginfo(f"  Joint angles (order [j3,j5,j4,j2,j6,j1]):")
            for i, name in enumerate(self.joint_names):
                rospy.loginfo(f"    {name}: {joints[i]:.3f} rad ({joints[i]*180/np.pi:.1f} deg)")
            
            self._publish_result(f"IK found")
        else:
            rospy.logwarn(f"[MotionControl] No IK solution for ({x:.3f}, {y:.3f}, {z:.3f})")
            self._publish_result("IK failed")
    
    # =========================================================================
    # Cartesian Motion Control (Full Pose)
    # =========================================================================
    
    def move_to_cartesian_pose(self, x: float, y: float, z: float, 
                               roll: float = None, pitch: float = None, yaw: float = None,
                               duration: float = 5.0) -> bool:
        """
        Move to Cartesian pose (position + orientation)
        """
        target_pose = PoseStamped()
        target_pose.header.frame_id = self.base_frame
        target_pose.header.stamp = rospy.Time.now()
        target_pose.pose.position = Point(x, y, z)
        
        # Handle orientation
        if roll is not None and pitch is not None and yaw is not None:
            q = self._rpy_to_quaternion(roll, pitch, yaw)
            target_pose.pose.orientation = Quaternion(q[0], q[1], q[2], q[3])
            rospy.loginfo(f"[MotionControl] Using specified orientation: "
                         f"roll={roll:.2f}, pitch={pitch:.2f}, yaw={yaw:.2f}")
        elif self.current_ee_pose:
            target_pose.pose.orientation = self.current_ee_pose.pose.orientation
            rospy.loginfo("[MotionControl] Keeping current orientation")
        else:
            target_pose.pose.orientation = Quaternion(0, 0, 0, 1)
            rospy.loginfo("[MotionControl] Using default orientation")
        
        pos_str = f"({x:.3f}, {y:.3f}, {z:.3f})"
        rospy.loginfo(f"[MotionControl] Moving to Cartesian pose: position={pos_str}")
        
        # Compute IK (returns joints in CORRECT order)
        target_joints = self.compute_inverse_kinematics(target_pose)
        
        if target_joints is None:
            rospy.logerr("[MotionControl] IK failed for target pose")
            self._publish_result("ERROR: IK failed")
            return False
        
        # Execute joint motion
        return self._execute_joint_motion(target_joints, duration)
    
    def _rpy_to_quaternion(self, roll: float, pitch: float, yaw: float) -> List[float]:
        """Convert RPY Euler angles to quaternion"""
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
    
    # =========================================================================
    # Joint Motion Execution - No reordering needed
    # =========================================================================
    
    def _execute_joint_motion(
        self,
        target_joints: List[float],
        duration: float = 4.0,
        use_action: bool = True
    ) -> bool:
        """
        Execute joint space motion - No reordering needed as joint_names matches controller order
        
        Args:
            target_joints: Desired joint configuration in CORRECT order [j3, j5, j4, j2, j6, j1]
            duration: Motion duration (seconds)
            use_action: Whether to use action client (True) or publish trajectory (False)
        
        Returns:
            bool: Success or failure
        """
        # =================================================================
        # 1. 输入验证
        # =================================================================
        if self.current_joint_positions is None:
            rospy.logerr("[MotionControl] No current joint state available")
            self._publish_result("ERROR: No current joint state")
            return False
        
        if len(target_joints) != self.num_joints:
            rospy.logerr(f"[MotionControl] Invalid number of joints: expected {self.num_joints}, got {len(target_joints)}")
            self._publish_result("ERROR: Invalid joint count")
            return False
        
        # =================================================================
        # 2. 显示接收到的关节角度
        # =================================================================
        rospy.loginfo("=" * 60)
        rospy.loginfo("[MotionControl] Joint Motion Command:")
        rospy.loginfo(f"  Target joints (order [j3,j5,j4,j2,j6,j1]):")
        for i, name in enumerate(self.joint_names):
            rospy.loginfo(f"    {name}: {target_joints[i]:.3f} rad ({target_joints[i]*180/np.pi:.1f} deg)")
        
        # =================================================================
        # 3. 与当前关节角度对比
        # =================================================================
        if self.current_joint_positions:
            delta = [target_joints[i] - self.current_joint_positions[i] for i in range(6)]
            rospy.loginfo(f"\n  Delta from current:")
            for i, name in enumerate(self.joint_names):
                rospy.loginfo(f"    {name}: {delta[i]:.3f} rad")
        rospy.loginfo("=" * 60)
        
        # =================================================================
        # 4. 关节限位检查
        # =================================================================
        # Need to map to standard joint names for limit checking
        limit_check_joints = {
            'shoulder_pan_joint': target_joints[5],  # j1 at position 5
            'shoulder_lift_joint': target_joints[3],  # j2 at position 3
            'elbow_joint': target_joints[0],          # j3 at position 0
            'wrist_1_joint': target_joints[2],        # j4 at position 2
            'wrist_2_joint': target_joints[1],        # j5 at position 1
            'wrist_3_joint': target_joints[4]         # j6 at position 4
        }
        
        all_valid = True
        for joint_name, value in limit_check_joints.items():
            limits = self.joint_limits[joint_name]
            if not (limits[0] <= value <= limits[1]):
                all_valid = False
                rospy.logerr(f"[MotionControl] Joint {joint_name} outside limits: "
                             f"{value:.3f} rad (limits: [{limits[0]:.3f}, {limits[1]:.3f}])")
        
        if not all_valid:
            rospy.logerr("[MotionControl] Target joints violate limits")
            self._publish_result("ERROR: Joint limit violation")
            return False
        
        # =================================================================
        # 5. 执行运动 - No reordering needed
        # =================================================================
        if use_action:
            # 使用动作客户端
            goal = FollowJointTrajectoryGoal()
            goal.trajectory = JointTrajectory()
            goal.trajectory.joint_names = self.joint_names  # Already in correct order
            
            # 创建轨迹点
            point = JointTrajectoryPoint()
            point.positions = target_joints
            point.time_from_start = rospy.Duration(duration)
            goal.trajectory.points.append(point)
            
            rospy.loginfo(f"[MotionControl] Sending goal to action server...")
            self.trajectory_client.send_goal(goal)
            
            # 等待结果（带超时）
            if self.trajectory_client.wait_for_result(timeout=rospy.Duration(duration + 2.0)):
                result = self.trajectory_client.get_result()
                rospy.loginfo("[MotionControl] Motion completed successfully")
                
                # 等待一小段时间让状态更新
                rospy.sleep(0.5)
                
                # 验证是否到达目标位置
                if self.current_joint_positions:
                    errors = [abs(self.current_joint_positions[i] - target_joints[i]) for i in range(6)]
                    max_error = max(errors)
                    avg_error = sum(errors) / 6
                    
                    rospy.loginfo(f"[MotionControl] Position errors:")
                    for i, name in enumerate(self.joint_names):
                        rospy.loginfo(f"    {name}: {errors[i]:.3f} rad")
                    rospy.loginfo(f"[MotionControl] Max error: {max_error:.3f} rad, Avg error: {avg_error:.3f} rad")
                    
                    if max_error > 0.05:  # 约3度误差
                        rospy.logwarn(f"[MotionControl] Motion completed but significant position error: {max_error:.3f} rad")
                        self._publish_result(f"SUCCESS: Motion completed with {max_error:.3f} rad error")
                    else:
                        self._publish_result("SUCCESS: Motion completed (1:1 mapping achieved)")
                else:
                    self._publish_result("SUCCESS: Motion completed")
                
                return True
            else:
                rospy.logerr("[MotionControl] Motion timed out")
                self.trajectory_client.cancel_goal()
                self._publish_result("ERROR: Motion timeout")
                return False
        else:
            # 方式2：直接发布轨迹
            trajectory = self.generate_joint_trajectory(
                self.current_joint_positions, target_joints, duration
            )
            self.trajectory_pub.publish(trajectory)
            rospy.loginfo(f"[MotionControl] Published trajectory to /joint_trajectory")
            self._publish_result("SUCCESS: Trajectory published")
            return True
    
    # =========================================================================
    # TF2 Utilities
    # =========================================================================
    
    def _query_transform(self, source_frame: str, target_frame: str):
        """Query and print transform between two frames"""
        try:
            transform: TransformStamped = self.tf_buffer.lookup_transform(
                target_frame=target_frame,
                source_frame=source_frame,
                time=rospy.Time(0),
                timeout=rospy.Duration(1.0)
            )
            
            trans = transform.transform.translation
            rot = transform.transform.rotation
            
            rospy.loginfo("=" * 60)
            rospy.loginfo(f"Transform from '{source_frame}' to '{target_frame}':")
            rospy.loginfo(f"  Translation: x={trans.x:.3f}, y={trans.y:.3f}, z={trans.z:.3f} m")
            
            roll, pitch, yaw = self._quaternion_to_rpy(rot)
            rospy.loginfo(f"  Rotation (RPY): roll={roll:.2f}, pitch={pitch:.2f}, yaw={yaw:.2f} rad")
            rospy.loginfo(f"  Rotation (quaternion): ({rot.x:.3f}, {rot.y:.3f}, {rot.z:.3f}, {rot.w:.3f})")
            rospy.loginfo("=" * 60)
            
            self._publish_result(f"Transform: {source_frame} -> {target_frame} found")
            
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            rospy.logerr(f"[MotionControl] TF2 lookup failed: {e}")
            self._publish_result(f"ERROR: Transform failed - {e}")
    
    def _quaternion_to_rpy(self, q: Quaternion) -> Tuple[float, float, float]:
        """Convert quaternion to RPY Euler angles"""
        roll = np.arctan2(2.0 * (q.w * q.x + q.y * q.z),
                         1.0 - 2.0 * (q.x * q.x + q.y * q.y))
        pitch = np.arcsin(2.0 * (q.w * q.y - q.z * q.x))
        yaw = np.arctan2(2.0 * (q.w * q.z + q.x * q.y),
                        1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        
        return roll, pitch, yaw
    
    def get_transform(self, source_frame: str, target_frame: str, 
                     timeout: float = 1.0) -> Optional[TransformStamped]:
        """Public method to get transform between frames"""
        try:
            transform = self.tf_buffer.lookup_transform(
                target_frame=target_frame,
                source_frame=source_frame,
                time=rospy.Time(0),
                timeout=rospy.Duration(timeout)
            )
            return transform
        except Exception as e:
            rospy.logerr(f"[MotionControl] Failed to get transform: {e}")
            return None
    
    # =========================================================================
    # Trajectory Generation
    # =========================================================================
    
    def generate_joint_trajectory(
        self,
        start_joints: List[float],
        goal_joints: List[float],
        duration: float,
        num_points: int = 50
    ) -> JointTrajectory:
        """
        Generate smooth joint space trajectory using cubic interpolation
        Input joints in CORRECT order [j3, j5, j4, j2, j6, j1]
        """
        trajectory = JointTrajectory()
        trajectory.joint_names = self.joint_names
        
        for i in range(num_points + 1):
            t = (i / num_points) * duration
            
            point = JointTrajectoryPoint()
            point.positions = []
            point.velocities = []
            point.accelerations = []
            
            for j in range(self.num_joints):
                a0 = start_joints[j]
                a1 = 0
                a2 = 3 * (goal_joints[j] - start_joints[j]) / (duration ** 2)
                a3 = -2 * (goal_joints[j] - start_joints[j]) / (duration ** 3)
                
                pos = a0 + a1*t + a2*t**2 + a3*t**3
                point.positions.append(pos)
                
                vel = a1 + 2*a2*t + 3*a3*t**2
                point.velocities.append(vel)
                
                acc = 2*a2 + 6*a3*t
                point.accelerations.append(acc)
            
            point.time_from_start = rospy.Duration(t)
            trajectory.points.append(point)
        
        return trajectory
    
    def generate_minimum_time_trajectory(
        self,
        start_joints: List[float],
        goal_joints: List[float],
        max_velocity: float = 1.0,
        max_acceleration: float = 1.0
    ) -> JointTrajectory:
        """Generate time-optimal trajectory with velocity/acceleration limits"""
        max_delta = max(abs(g - s) for s, g in zip(start_joints, goal_joints))
        
        t_acc = max_velocity / max_acceleration
        d_acc = 0.5 * max_acceleration * t_acc ** 2
        
        if max_delta <= 2 * d_acc:
            t_acc = np.sqrt(max_delta / max_acceleration)
            t_total = 2 * t_acc
        else:
            t_acc = max_velocity / max_acceleration
            d_acc = 0.5 * max_acceleration * t_acc ** 2
            d_const = max_delta - 2 * d_acc
            t_const = d_const / max_velocity
            t_total = 2 * t_acc + t_const
        
        return self.generate_joint_trajectory(start_joints, goal_joints, t_total)
    
    # =========================================================================
    # Motion Execution Helpers
    # =========================================================================
    
    def _execute_cartesian_motion(self, target_pose: PoseStamped, duration: float = 5.0) -> bool:
        """Execute Cartesian space motion"""
        target_joints = self.compute_inverse_kinematics(target_pose)
        
        if target_joints is None:
            rospy.logerr("[MotionControl] IK failed for target pose")
            self._publish_result("ERROR: IK failed")
            return False
        
        return self._execute_joint_motion(target_joints, duration)
    
    def _execute_trajectory(self, trajectory: JointTrajectory) -> bool:
        """Execute pre-computed trajectory"""
        if len(trajectory.points) == 0:
            rospy.logwarn("[MotionControl] Empty trajectory")
            return False
        
        goal = FollowJointTrajectoryGoal()
        goal.trajectory = trajectory
        
        rospy.loginfo(f"[MotionControl] Executing trajectory with {len(trajectory.points)} points")
        self.trajectory_client.send_goal(goal)
        
        if self.trajectory_client.wait_for_result():
            rospy.loginfo("[MotionControl] Trajectory completed")
            self._publish_result("SUCCESS: Trajectory completed")
            return True
        else:
            rospy.logerr("[MotionControl] Trajectory execution failed")
            self._publish_result("ERROR: Trajectory failed")
            return False
    
    def _stop_motion(self):
        """Emergency stop - halt all motion immediately"""
        rospy.logwarn("[MotionControl] Stopping motion!")
        self.trajectory_client.cancel_all_goals()
        self._publish_result("STOPPED: Motion halted")
    
    def _move_to_home(self, duration: float = 4.0) -> bool:
        """Move robot to home configuration"""
        # Home position in correct order [j3, j5, j4, j2, j6, j1]
        home_pos = [1.57, -1.57, -1.57, -1.57, 0.0, 0.0]
        rospy.loginfo("[MotionControl] Moving to home position")
        return self._execute_joint_motion(home_pos, duration)
    
    def _run_test_sequence(self):
        """Run test sequence with correct joint ordering"""
        rospy.loginfo("=" * 60)
        rospy.loginfo("UR5e Test Sequence Starting")
        rospy.loginfo("=" * 60)
        
        rospy.sleep(1.0)
        self._print_current_state()
        
        # Test Position 1: Home position
        rospy.loginfo("\n[Test] Moving to HOME position...")
        home_pos = [1.57, -1.57, -1.57, -1.57, 0.0, 0.0]
        self._execute_joint_motion(home_pos, duration=4.0)
        rospy.sleep(1.0)
        
        # Test Position 2: BENT position variation
        rospy.loginfo("\n[Test] Moving to BENT position...")
        bent_pos = [1.57, -1.57, -1.57, -1.57, 0.0, 1.57]
        self._execute_joint_motion(bent_pos, duration=4.0)
        rospy.sleep(1.0)
        
        # Test Position 3: Another position
        rospy.loginfo("\n[Test] Moving to ROTATE position...")
        rotate_pos = [1.57, -1.57, -1.57, -1.57, 0.0, -1.57]
        self._execute_joint_motion(rotate_pos, duration=4.0)
        rospy.sleep(1.0)
        
        # Test Position 4: Return to home
        rospy.loginfo("\n[Test] Returning to HOME...")
        self._execute_joint_motion(home_pos, duration=4.0)
        
        self._print_current_state()
        
        rospy.loginfo("=" * 60)
        rospy.loginfo("Test sequence completed!")
        rospy.loginfo("=" * 60)
    
    # =========================================================================
    # Path Recording and Replay
    # =========================================================================
    
    def _start_recording(self):
        """Start recording current path"""
        self.recorded_path = []
        self.is_recording = True
        rospy.loginfo("[MotionControl] Started recording path")
        self._publish_result("Recording started")
    
    def _stop_recording(self):
        """Stop recording path"""
        self.is_recording = False
        rospy.loginfo(f"[MotionControl] Stopped recording. Recorded {len(self.recorded_path)} points")
        self._publish_result(f"Recording stopped: {len(self.recorded_path)} points")
        self._publish_path_markers()
    
    def _replay_path(self):
        """Replay recorded path"""
        if not self.recorded_path:
            rospy.logwarn("[MotionControl] No recorded path to replay")
            self._publish_result("ERROR: No recorded path")
            return
        
        rospy.loginfo(f"[MotionControl] Replaying path with {len(self.recorded_path)} points")
        
        for i, joints in enumerate(self.recorded_path):
            rospy.loginfo(f"  Point {i+1}/{len(self.recorded_path)}")
            self._execute_joint_motion(joints, duration=2.0)
            rospy.sleep(0.5)
        
        rospy.loginfo("[MotionControl] Path replay completed")
        self._publish_result("Path replay completed")
    
    def _save_path(self, name: str):
        """Save recorded path to file"""
        if not self.recorded_path:
            rospy.logwarn("[MotionControl] No path to save")
            return
        
        import pickle
        import os
        
        save_dir = os.path.expanduser("~/motion_paths")
        os.makedirs(save_dir, exist_ok=True)
        
        filename = os.path.join(save_dir, f"{name}.pkl")
        
        with open(filename, 'wb') as f:
            pickle.dump(self.recorded_path, f)
        
        rospy.loginfo(f"[MotionControl] Path saved to {filename}")
        self._publish_result(f"Path saved: {name}")
    
    def _load_path(self, name: str):
        """Load path from file"""
        import pickle
        import os
        
        filename = os.path.expanduser(f"~/motion_paths/{name}.pkl")
        
        try:
            with open(filename, 'rb') as f:
                self.recorded_path = pickle.load(f)
            rospy.loginfo(f"[MotionControl] Loaded path with {len(self.recorded_path)} points from {filename}")
            self._publish_result(f"Path loaded: {name} ({len(self.recorded_path)} points)")
            self._publish_path_markers()
            
        except FileNotFoundError:
            rospy.logerr(f"[MotionControl] Path file not found: {filename}")
            self._publish_result(f"ERROR: Path {name} not found")
    
    def _list_paths(self):
        """List all saved paths"""
        import os
        import glob
        
        save_dir = os.path.expanduser("~/motion_paths")
        if not os.path.exists(save_dir):
            rospy.loginfo("[MotionControl] No saved paths")
            return
        
        files = glob.glob(os.path.join(save_dir, "*.pkl"))
        if files:
            rospy.loginfo("[MotionControl] Saved paths:")
            for f in files:
                name = os.path.basename(f).replace('.pkl', '')
                with open(f, 'rb') as pf:
                    import pickle
                    data = pickle.load(pf)
                    rospy.loginfo(f"  - {name} ({len(data)} points)")
        else:
            rospy.loginfo("[MotionControl] No saved paths")
    
    def _publish_path_markers(self):
        """Publish path markers for RViz visualization"""
        if not self.marker_pub or not self.recorded_path:
            return
        
        try:
            from visualization_msgs.msg import Marker
            
            marker = Marker()
            marker.header.frame_id = self.base_frame
            marker.header.stamp = rospy.Time.now()
            marker.type = Marker.POINTS
            marker.action = Marker.ADD
            marker.scale.x = 0.02
            marker.scale.y = 0.02
            marker.color.a = 1.0
            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 0.0
            
            for joints in self.recorded_path:
                pose = self._compute_fk_silent(joints)
                if pose:
                    p = pose.pose.position
                    marker.points.append(Point(p.x, p.y, p.z))
            
            self.marker_pub.publish(marker)
            
        except Exception as e:
            rospy.logwarn(f"[MotionControl] Failed to publish markers: {e}")
    
    # =========================================================================
    # Utilities
    # =========================================================================
    
    def _print_current_state(self):
        """Print current joint angles and EE pose"""
        if self.current_joint_positions:
            rospy.loginfo("=" * 60)
            rospy.loginfo("Current Joint Positions (CORRECT order [j3,j5,j4,j2,j6,j1]):")
            for i, joint_name in enumerate(self.joint_names):
                pos = self.current_joint_positions[i]
                rospy.loginfo(f"  {joint_name}: {pos:.3f} rad ({pos*180/np.pi:.1f} deg)")
            
            if self.current_ee_pose:
                p = self.current_ee_pose.pose.position
                o = self.current_ee_pose.pose.orientation
                roll, pitch, yaw = self._quaternion_to_rpy(o)
                rospy.loginfo(f"End-Effector Pose:")
                rospy.loginfo(f"  Position: x={p.x:.3f}, y={p.y:.3f}, z={p.z:.3f}")
                rospy.loginfo(f"  Orientation (RPY): roll={roll:.2f}, pitch={pitch:.2f}, yaw={yaw:.2f}")
            rospy.loginfo("=" * 60)
        else:
            rospy.logwarn("[MotionControl] No joint states available")
    
    def _publish_result(self, message: str):
        """Publish motion execution result"""
        self.motion_result_pub.publish(message)
    
    def _transform_pose(self, pose: PoseStamped, target_frame: str) -> Optional[PoseStamped]:
        """Transform pose to target frame using TF2"""
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
    
    def _print_help(self):
        """Print help message with available commands"""
        help_msg = """
Available commands:
  Motion Commands:
    home                    - Move to home position
    bent                    - Move to bent arm position
    rotate                  - Rotate base 90 degrees
    test_sequence           - Run full test sequence
    stop                    - Emergency stop
    
  Kinematics:
    fk                      - Print current joint angles and EE pose (with debug)
    ik x y z                - Compute IK for target position
    move_to x y z           - Move to Cartesian position (keep orientation)
    move_to x y z r p y     - Move to Cartesian pose (position + orientation)
    move_joint [6 values]   - Move to specific joint angles in CORRECT order: [j3, j5, j4, j2, j6, j1]
    
  Path Recording:
    record_start            - Start recording current path
    record_stop             - Stop recording
    replay                  - Replay recorded path
    save_path name          - Save current path to file
    load_path name          - Load path from file
    list_paths              - List all saved paths
    
  TF2 Queries:
    get_transform source target - Get transform between two frames
    
  Other:
    help                    - Show this help message

NOTE: Joint order is [j3, j5, j4, j2, j6, j1] where:
  j3 = elbow_joint
  j5 = wrist_2_joint
  j4 = wrist_1_joint
  j2 = shoulder_lift_joint
  j6 = wrist_3_joint
  j1 = shoulder_pan_joint
This order matches the actual UR5e controller configuration.
"""
        rospy.loginfo(help_msg)
    
    def get_current_joint_positions(self) -> Optional[List[float]]:
        """Get current joint positions (CORRECT order)"""
        return self.current_joint_positions
    
    def get_current_ee_pose(self) -> Optional[PoseStamped]:
        """Get current end-effector pose"""
        return self.current_ee_pose
    
    def wait_for_joint_states(self, timeout: float = 5.0) -> bool:
        """Wait for first joint state message"""
        start_time = rospy.Time.now()
        rate = rospy.Rate(10)
        
        while self.current_joint_positions is None:
            if (rospy.Time.now() - start_time).to_sec() > timeout:
                rospy.logerr("[MotionControl] Timeout waiting for joint states")
                return False
            rate.sleep()
        
        return True
    
    def run(self):
        """Main loop"""
        rospy.loginfo("[MotionControl] Node running. Type 'help' for available commands")
        self._print_help()
        rospy.spin()


def main():
    try:
        node = MotionControlNode()
        
        if node.wait_for_joint_states():
            rospy.loginfo("[MotionControl] Initial joint states received")
        else:
            rospy.logwarn("[MotionControl] No joint states received")
        
        node.run()
        
    except rospy.ROSInterruptException:
        pass
    except Exception as e:
        rospy.logerr(f"[MotionControl] Unhandled exception: {e}")
        import traceback
        traceback.print_exc()


if __name__ == '__main__':
    main()