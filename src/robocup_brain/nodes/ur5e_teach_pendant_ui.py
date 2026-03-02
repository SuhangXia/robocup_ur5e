#!/usr/bin/env python3
"""
UR5e Teach Pendant UI - 软示教器界面
纯 UI 脚本，不包含运动控制逻辑
所有运动控制函数由同事在 motion_control_node.py 中实现

Author: Suhang Xia
Usage:
  rosrun robocup_brain ur5e_teach_pendant_ui.py
"""

import sys
import threading
import rospy
from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
    QPushButton, QLabel, QDoubleSpinBox, QGroupBox, QGridLayout,
    QTextEdit
)
from PyQt5.QtCore import Qt, QTimer, pyqtSignal
from PyQt5.QtGui import QFont

# ROS Messages
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
from sensor_msgs.msg import JointState
from common_msgs.msg import MotionCommand, GraspResult
import tf2_ros
import tf2_geometry_msgs
import tf.transformations as tf_trans


class UR5eTeachPendantUI(QMainWindow):
    """
    UR5e 软示教器界面类
    注意：此类仅负责 UI 显示和按钮事件响应
    所有实际的运动控制功能需要调用 motion_control_node.py 中的接口
    """

    pose_updated_signal = pyqtSignal(dict)
    status_updated_signal = pyqtSignal(str)
    joints_updated_signal = pyqtSignal(list)

    def __init__(self):
        super().__init__()
        self.setWindowTitle("UR5e Teach Pendant")
        self.setGeometry(100, 100, 800, 700)

        self.linear_step = 0.01
        self.rotation_step = 0.087
        self.current_pose = None      # dict with x,y,z,roll,pitch,yaw for display
        self.current_pose_stamped = None  # PoseStamped for sending to motion_control
        self.current_joints = None
        self.is_moving = False
        self.recorded_points_list = []  # list of {"pose": dict, "joints": list, "index": int}

        self.init_ros()
        self.init_ui()

        self.update_timer = QTimer()
        self.update_timer.timeout.connect(self.on_update_timer)
        self.update_timer.start(100)

        self.pose_updated_signal.connect(self.on_pose_updated)
        self.status_updated_signal.connect(self.on_status_updated)
        self.joints_updated_signal.connect(self.on_joints_updated)

    def init_ros(self):
        try:
            rospy.init_node('ur5e_teach_pendant_ui', anonymous=True)
            self.motion_cmd_pub = rospy.Publisher('/motion/command', MotionCommand, queue_size=10)
            self.joint_state_sub = rospy.Subscriber('/joint_states', JointState, self.on_joint_state_received)
            self.motion_result_sub = rospy.Subscriber('/motion/result', GraspResult, self.on_motion_result_received)
            self.tf_buffer = tf2_ros.Buffer()
            self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
            # Run rospy callbacks in a background thread (rospy.spin_once not in older rospy)
            self._ros_thread = threading.Thread(target=self._ros_spin_thread, daemon=True)
            self._ros_thread.start()
            rospy.loginfo("[TeachPendantUI] ROS initialized")
        except Exception as e:
            rospy.logerr(f"[TeachPendantUI] ROS init failed: {e}")

    def _ros_spin_thread(self):
        """Background thread: run rospy.spin() so callbacks (joint_states, motion/result, TF) run."""
        try:
            rospy.spin()
        except rospy.exceptions.ROSInterruptException:
            pass
        except Exception:
            pass

    def _get_current_pose_from_tf(self):
        """
        Get current end-effector pose (base_link -> tcp_link).
        tcp_link should match MoveIt's tip link (e.g. gripper_tip_link) so IK is consistent.
        Returns (PoseStamped or None, pose_dict or None). pose_dict has x,y,z,roll,pitch,yaw.
        """
        tcp_link = rospy.get_param('~tcp_link', 'gripper_tip_link')  # same as MoveIt ik_link
        try:
            trans = self.tf_buffer.lookup_transform(
                'base_link', tcp_link, rospy.Time(0), rospy.Duration(0.5)
            )
            pose = PoseStamped()
            pose.header = trans.header
            pose.pose.position = Point(
                trans.transform.translation.x,
                trans.transform.translation.y,
                trans.transform.translation.z
            )
            pose.pose.orientation = Quaternion(
                trans.transform.rotation.x,
                trans.transform.rotation.y,
                trans.transform.rotation.z,
                trans.transform.rotation.w
            )
            r = trans.transform.rotation
            q = [r.x, r.y, r.z, r.w]
            roll, pitch, yaw = tf_trans.euler_from_quaternion(q)
            pose_dict = {
                'x': pose.pose.position.x, 'y': pose.pose.position.y, 'z': pose.pose.position.z,
                'roll': roll, 'pitch': pitch, 'yaw': yaw
            }
            return pose, pose_dict
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            rospy.logdebug("[TeachPendantUI] TF lookup failed: %s" % str(e))
            return None, None

    def _send_move_to_pose(self, target_pose_stamped):
        """Publish MotionCommand MOVE_TO_POSE."""
        target_pose_stamped.header.stamp = rospy.Time.now()
        target_pose_stamped.header.frame_id = 'base_link'
        cmd = MotionCommand()
        cmd.command_type = MotionCommand.MOVE_TO_POSE
        cmd.target_pose = target_pose_stamped
        cmd.max_velocity = 1.0   # 1.0 => ~5s for Cartesian; 0.5 => ~10s (slower)
        cmd.max_acceleration = 1.0
        self.motion_cmd_pub.publish(cmd)
        self.is_moving = True
        self.status_updated_signal.emit("Status: Moving...")

    def init_ui(self):
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        main_layout = QHBoxLayout(central_widget)
        left_panel = QWidget()
        right_panel = QWidget()
        main_layout.addWidget(left_panel, 2)
        main_layout.addWidget(right_panel, 1)
        left_layout = QVBoxLayout(left_panel)

        pose_group = QGroupBox("Current EE Pose (Base Link -> Tool0)")
        pose_layout = QGridLayout(pose_group)
        pose_layout.addWidget(QLabel("<b>Position (m):</b>"), 0, 0, 1, 6)
        self.pose_labels = {}
        for i, coord in enumerate(['X', 'Y', 'Z']):
            label_value = QLabel("--")
            label_value.setStyleSheet("font-family: monospace; font-size: 12px;")
            self.pose_labels[f'pos_{coord}'] = label_value
            pose_layout.addWidget(QLabel(f"{coord}:"), 1, i * 2)
            pose_layout.addWidget(label_value, 1, i * 2 + 1)
        pose_layout.addWidget(QLabel("<b>Orientation (rad):</b>"), 2, 0, 1, 6)
        for i, coord in enumerate(['Roll', 'Pitch', 'Yaw']):
            label_value = QLabel("--")
            label_value.setStyleSheet("font-family: monospace; font-size: 12px;")
            self.pose_labels[f'rot_{coord}'] = label_value
            pose_layout.addWidget(QLabel(f"{coord}:"), 3, i * 2)
            pose_layout.addWidget(label_value, 3, i * 2 + 1)
        self.btn_refresh_pose = QPushButton("Refresh Pose")
        self.btn_refresh_pose.clicked.connect(self.on_refresh_pose_clicked)
        pose_layout.addWidget(self.btn_refresh_pose, 4, 0, 1, 6)
        left_layout.addWidget(pose_group)

        step_group = QGroupBox("Step Size")
        step_layout = QGridLayout(step_group)
        step_layout.addWidget(QLabel("Linear step (m):"), 0, 0)
        self.linear_spinbox = QDoubleSpinBox()
        self.linear_spinbox.setRange(0.001, 0.1)
        self.linear_spinbox.setValue(self.linear_step)
        self.linear_spinbox.setSingleStep(0.001)
        self.linear_spinbox.setDecimals(3)
        self.linear_spinbox.valueChanged.connect(self.on_linear_step_changed)
        step_layout.addWidget(self.linear_spinbox, 0, 1)
        for name, value in [("0.1mm", 0.0001), ("1mm", 0.001), ("5mm", 0.005), ("1cm", 0.01), ("5cm", 0.05)]:
            btn = QPushButton(name)
            btn.clicked.connect(lambda checked, v=value: self.linear_spinbox.setValue(v))
            step_layout.addWidget(btn)
        step_layout.addWidget(QLabel("Rotation step (deg):"), 1, 0)
        self.rotation_spinbox = QDoubleSpinBox()
        self.rotation_spinbox.setRange(0.1, 45.0)
        self.rotation_spinbox.setValue(self.rotation_step * 180 / 3.14159)
        self.rotation_spinbox.setSingleStep(0.5)
        self.rotation_spinbox.setDecimals(1)
        self.rotation_spinbox.valueChanged.connect(self.on_rotation_step_changed)
        step_layout.addWidget(self.rotation_spinbox, 1, 1)
        for name, value in [("0.1°", 0.1), ("0.5°", 0.5), ("1°", 1.0), ("5°", 5.0), ("10°", 10.0)]:
            btn = QPushButton(name)
            btn.clicked.connect(lambda checked, v=value: self.rotation_spinbox.setValue(v))
            step_layout.addWidget(btn)
        left_layout.addWidget(step_group)

        trans_group = QGroupBox("Translation")
        trans_layout = QGridLayout(trans_group)
        for row, (axis, neg_lbl, pos_lbl, on_neg, on_pos) in enumerate([
            ("X", "-X ←", "+X →", self.on_x_negative_clicked, self.on_x_positive_clicked),
            ("Y", "-Y ↓", "+Y ↑", self.on_y_negative_clicked, self.on_y_positive_clicked),
            ("Z", "-Z ↓", "+Z ↑", self.on_z_negative_clicked, self.on_z_positive_clicked),
        ]):
            trans_layout.addWidget(QLabel(f"<b>{axis}:</b>"), row, 0)
            b1, b2 = QPushButton(neg_lbl), QPushButton(pos_lbl)
            b1.setStyleSheet("background-color: #ffcccc;")
            b2.setStyleSheet("background-color: #ccffcc;")
            b1.clicked.connect(on_neg)
            b2.clicked.connect(on_pos)
            trans_layout.addWidget(b1, row, 1)
            trans_layout.addWidget(b2, row, 2)
        left_layout.addWidget(trans_group)

        rot_group = QGroupBox("Rotation")
        rot_layout = QGridLayout(rot_group)
        for row, (name, neg_lbl, pos_lbl, on_neg, on_pos) in enumerate([
            ("Roll", "-Roll ↺", "+Roll ↻", self.on_roll_negative_clicked, self.on_roll_positive_clicked),
            ("Pitch", "-Pitch ↺", "+Pitch ↻", self.on_pitch_negative_clicked, self.on_pitch_positive_clicked),
            ("Yaw", "-Yaw ↺", "+Yaw ↻", self.on_yaw_negative_clicked, self.on_yaw_positive_clicked),
        ]):
            rot_layout.addWidget(QLabel(f"<b>{name}:</b>"), row, 0)
            b1, b2 = QPushButton(neg_lbl), QPushButton(pos_lbl)
            b1.setStyleSheet("background-color: #ffcccc;")
            b2.setStyleSheet("background-color: #ccffcc;")
            b1.clicked.connect(on_neg)
            b2.clicked.connect(on_pos)
            rot_layout.addWidget(b1, row, 1)
            rot_layout.addWidget(b2, row, 2)
        left_layout.addWidget(rot_group)

        func_group = QGroupBox("Actions")
        func_layout = QHBoxLayout(func_group)
        self.btn_home = QPushButton("Home")
        self.btn_home.clicked.connect(self.on_home_clicked)
        self.btn_stop = QPushButton("E-Stop")
        self.btn_stop.setStyleSheet("background-color: #ff6666; font-weight: bold;")
        self.btn_stop.clicked.connect(self.on_stop_clicked)
        self.btn_record = QPushButton("Record Point")
        self.btn_record.clicked.connect(self.on_record_point_clicked)
        func_layout.addWidget(self.btn_home)
        func_layout.addWidget(self.btn_stop)
        func_layout.addWidget(self.btn_record)
        left_layout.addWidget(func_group)

        self.status_label = QLabel("Status: Ready | Waiting for motion_control_node...")
        self.status_label.setStyleSheet("padding: 5px; background-color: #f0f0f0;")
        self.status_label.setAlignment(Qt.AlignCenter)
        left_layout.addWidget(self.status_label)

        right_layout = QVBoxLayout(right_panel)
        joints_group = QGroupBox("Joint Angles (rad)")
        joints_layout = QVBoxLayout(joints_group)
        self.joint_labels = []
        for name in ['J1 (Shoulder Pan)', 'J2 (Shoulder Lift)', 'J3 (Elbow)', 'J4 (Wrist 1)', 'J5 (Wrist 2)', 'J6 (Wrist 3)']:
            h = QHBoxLayout()
            h.addWidget(QLabel(f"{name}:"))
            lb = QLabel("--")
            lb.setStyleSheet("font-family: monospace;")
            self.joint_labels.append(lb)
            h.addWidget(lb)
            joints_layout.addLayout(h)
        right_layout.addWidget(joints_group)
        log_group = QGroupBox("Log")
        self.log_text = QTextEdit()
        self.log_text.setReadOnly(True)
        # QTextEdit has no setMaximumBlockCount (only QPlainTextEdit does)
        log_group_layout = QVBoxLayout(log_group)
        log_group_layout.addWidget(self.log_text)
        right_layout.addWidget(log_group)
        points_group = QGroupBox("Recorded Points")
        points_layout = QVBoxLayout(points_group)
        self.recorded_points = QTextEdit()
        self.recorded_points.setReadOnly(True)
        self.recorded_points.setMaximumHeight(150)
        points_layout.addWidget(self.recorded_points)
        self.btn_clear_points = QPushButton("Clear Points")
        self.btn_clear_points.clicked.connect(self.on_clear_points_clicked)
        points_layout.addWidget(self.btn_clear_points)
        right_layout.addWidget(points_group)

    def on_linear_step_changed(self, value):
        self.linear_step = value
        self.log(f"Linear step set: {value*1000:.1f} mm")

    def on_rotation_step_changed(self, value):
        self.rotation_step = value * 3.14159 / 180.0
        self.log(f"Rotation step set: {value:.1f} deg")

    def on_x_negative_clicked(self):
        pose_stamped, _ = self._get_current_pose_from_tf()
        if pose_stamped is None:
            self.log("X-: No current pose (TF base_link->tool0)")
            return
        pose_stamped.pose.position.x -= self.linear_step
        self._send_move_to_pose(pose_stamped)
        self.log(f"X- {-self.linear_step*1000:.1f} mm")
    def on_x_positive_clicked(self):
        pose_stamped, _ = self._get_current_pose_from_tf()
        if pose_stamped is None:
            self.log("X+: No current pose")
            return
        pose_stamped.pose.position.x += self.linear_step
        self._send_move_to_pose(pose_stamped)
        self.log(f"X+ +{self.linear_step*1000:.1f} mm")
    def on_y_negative_clicked(self):
        pose_stamped, _ = self._get_current_pose_from_tf()
        if pose_stamped is None:
            self.log("Y-: No current pose")
            return
        pose_stamped.pose.position.y -= self.linear_step
        self._send_move_to_pose(pose_stamped)
        self.log(f"Y- -{self.linear_step*1000:.1f} mm")
    def on_y_positive_clicked(self):
        pose_stamped, _ = self._get_current_pose_from_tf()
        if pose_stamped is None:
            self.log("Y+: No current pose")
            return
        pose_stamped.pose.position.y += self.linear_step
        self._send_move_to_pose(pose_stamped)
        self.log(f"Y+ +{self.linear_step*1000:.1f} mm")
    def on_z_negative_clicked(self):
        pose_stamped, _ = self._get_current_pose_from_tf()
        if pose_stamped is None:
            self.log("Z-: No current pose")
            return
        pose_stamped.pose.position.z -= self.linear_step
        self._send_move_to_pose(pose_stamped)
        self.log(f"Z- -{self.linear_step*1000:.1f} mm")
    def on_z_positive_clicked(self):
        pose_stamped, _ = self._get_current_pose_from_tf()
        if pose_stamped is None:
            self.log("Z+: No current pose")
            return
        pose_stamped.pose.position.z += self.linear_step
        self._send_move_to_pose(pose_stamped)
        self.log(f"Z+ +{self.linear_step*1000:.1f} mm")
    def on_roll_negative_clicked(self):
        pose_stamped, pose_dict = self._get_current_pose_from_tf()
        if pose_stamped is None or pose_dict is None:
            self.log("Roll-: No current pose")
            return
        r, p, y = pose_dict['roll'] - self.rotation_step, pose_dict['pitch'], pose_dict['yaw']
        q = tf_trans.quaternion_from_euler(r, p, y)
        pose_stamped.pose.orientation = Quaternion(q[0], q[1], q[2], q[3])
        self._send_move_to_pose(pose_stamped)
        self.log(f"Roll- -{self.rotation_step*180/3.14159:.1f} deg")
    def on_roll_positive_clicked(self):
        pose_stamped, pose_dict = self._get_current_pose_from_tf()
        if pose_stamped is None or pose_dict is None:
            self.log("Roll+: No current pose")
            return
        r, p, y = pose_dict['roll'] + self.rotation_step, pose_dict['pitch'], pose_dict['yaw']
        q = tf_trans.quaternion_from_euler(r, p, y)
        pose_stamped.pose.orientation = Quaternion(q[0], q[1], q[2], q[3])
        self._send_move_to_pose(pose_stamped)
        self.log(f"Roll+ +{self.rotation_step*180/3.14159:.1f} deg")
    def on_pitch_negative_clicked(self):
        pose_stamped, pose_dict = self._get_current_pose_from_tf()
        if pose_stamped is None or pose_dict is None:
            self.log("Pitch-: No current pose")
            return
        r, p, y = pose_dict['roll'], pose_dict['pitch'] - self.rotation_step, pose_dict['yaw']
        q = tf_trans.quaternion_from_euler(r, p, y)
        pose_stamped.pose.orientation = Quaternion(q[0], q[1], q[2], q[3])
        self._send_move_to_pose(pose_stamped)
        self.log(f"Pitch- -{self.rotation_step*180/3.14159:.1f} deg")
    def on_pitch_positive_clicked(self):
        pose_stamped, pose_dict = self._get_current_pose_from_tf()
        if pose_stamped is None or pose_dict is None:
            self.log("Pitch+: No current pose")
            return
        r, p, y = pose_dict['roll'], pose_dict['pitch'] + self.rotation_step, pose_dict['yaw']
        q = tf_trans.quaternion_from_euler(r, p, y)
        pose_stamped.pose.orientation = Quaternion(q[0], q[1], q[2], q[3])
        self._send_move_to_pose(pose_stamped)
        self.log(f"Pitch+ +{self.rotation_step*180/3.14159:.1f} deg")
    def on_yaw_negative_clicked(self):
        pose_stamped, pose_dict = self._get_current_pose_from_tf()
        if pose_stamped is None or pose_dict is None:
            self.log("Yaw-: No current pose")
            return
        r, p, y = pose_dict['roll'], pose_dict['pitch'], pose_dict['yaw'] - self.rotation_step
        q = tf_trans.quaternion_from_euler(r, p, y)
        pose_stamped.pose.orientation = Quaternion(q[0], q[1], q[2], q[3])
        self._send_move_to_pose(pose_stamped)
        self.log(f"Yaw- -{self.rotation_step*180/3.14159:.1f} deg")
    def on_yaw_positive_clicked(self):
        pose_stamped, pose_dict = self._get_current_pose_from_tf()
        if pose_stamped is None or pose_dict is None:
            self.log("Yaw+: No current pose")
            return
        r, p, y = pose_dict['roll'], pose_dict['pitch'], pose_dict['yaw'] + self.rotation_step
        q = tf_trans.quaternion_from_euler(r, p, y)
        pose_stamped.pose.orientation = Quaternion(q[0], q[1], q[2], q[3])
        self._send_move_to_pose(pose_stamped)
        self.log(f"Yaw+ +{self.rotation_step*180/3.14159:.1f} deg")
    def on_home_clicked(self):
        cmd = MotionCommand()
        cmd.command_type = MotionCommand.HOME
        self.motion_cmd_pub.publish(cmd)
        self.is_moving = True
        self.status_updated_signal.emit("Status: Going Home...")
        self.log("Go to Home")
    def on_stop_clicked(self):
        cmd = MotionCommand()
        cmd.command_type = MotionCommand.STOP
        self.motion_cmd_pub.publish(cmd)
        self.is_moving = False
        self.status_updated_signal.emit("Status: E-Stop sent")
        self.log("E-Stop")
    def on_record_point_clicked(self):
        _, pose_dict = self._get_current_pose_from_tf()
        if pose_dict is None:
            self.log("Cannot record: current pose unknown (TF)")
            return
        self.current_pose = pose_dict
        idx = len(self.recorded_points_list) + 1
        self.recorded_points_list.append({
            "pose": dict(pose_dict),
            "joints": list(self.current_joints) if self.current_joints else None,
            "index": idx
        })
        self._update_recorded_points_display()
        self.log("Record point #%d" % idx)
    def on_clear_points_clicked(self):
        self.recorded_points_list.clear()
        self._update_recorded_points_display()
        self.log("Clear points")
    def _update_recorded_points_display(self):
        lines = []
        for rec in self.recorded_points_list:
            p = rec["pose"]
            lines.append("P#%d: x=%.3f y=%.3f z=%.3f" % (rec["index"], p["x"], p["y"], p["z"]))
        self.recorded_points.setPlainText("\n".join(lines) if lines else "(none)")
    def on_refresh_pose_clicked(self):
        pose_stamped, pose_dict = self._get_current_pose_from_tf()
        if pose_dict is None:
            self.log("Refresh pose: TF failed (base_link->tool0)")
            return
        self.current_pose = pose_dict
        self.current_pose_stamped = pose_stamped
        self.pose_updated_signal.emit(pose_dict)
        self.log("Refresh pose OK")

    def on_joint_state_received(self, msg):
        joints = []
        for name in ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']:
            if name in msg.name:
                joints.append(msg.position[msg.name.index(name)])
        if len(joints) == 6:
            self.joints_updated_signal.emit(joints)

    def on_motion_result_received(self, msg):
        if msg.status == GraspResult.SUCCESS:
            self.status_updated_signal.emit("Status: Motion OK")
            self.log("OK %s" % msg.message)
        else:
            self.status_updated_signal.emit("Status: Motion Failed")
            self.log("Failed %s" % msg.message)
        self.is_moving = False

    def on_pose_updated(self, pose_dict):
        self.pose_labels['pos_X'].setText(f"{pose_dict['x']:.4f}")
        self.pose_labels['pos_Y'].setText(f"{pose_dict['y']:.4f}")
        self.pose_labels['pos_Z'].setText(f"{pose_dict['z']:.4f}")
        self.pose_labels['rot_Roll'].setText(f"{pose_dict['roll']:.4f}")
        self.pose_labels['rot_Pitch'].setText(f"{pose_dict['pitch']:.4f}")
        self.pose_labels['rot_Yaw'].setText(f"{pose_dict['yaw']:.4f}")

    def on_status_updated(self, status_text):
        self.status_label.setText(status_text)

    def on_joints_updated(self, joints):
        self.current_joints = joints
        for i, angle in enumerate(joints):
            if i < len(self.joint_labels):
                self.joint_labels[i].setText(f"{angle:.4f}")

    def on_update_timer(self):
        pass

    def log(self, message):
        from datetime import datetime
        self.log_text.append(f"[{datetime.now().strftime('%H:%M:%S')}] {message}")

    def closeEvent(self, event):
        self.update_timer.stop()
        rospy.signal_shutdown("Teach Pendant UI closed")
        event.accept()


def main():
    app = QApplication(sys.argv)
    app.setStyle('Fusion')
    app.setFont(QFont("Sans", 9))
    window = UR5eTeachPendantUI()
    window.show()
    try:
        sys.exit(app.exec_())
    except rospy.ROSInterruptException:
        pass


if __name__ == '__main__':
    main()
