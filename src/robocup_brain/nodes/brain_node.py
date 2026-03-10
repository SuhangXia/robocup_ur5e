#!/usr/bin/env python3
"""
RoboCup Brain Node - 基于行为树的决策系统
Architecture: py_trees_ros
Strategy: 增量扫描 -> 优先级评估(YOLO+TF) -> 请求抓取(GraspNet) -> 放置(MoveIt)
"""

import rospy
import py_trees
import py_trees_ros
from py_trees.common import Status

import tf2_ros
import tf2_geometry_msgs
from geometry_msgs.msg import PointStamped
import actionlib
from moveit_msgs.msg import MoveGroupAction, MoveGroupGoal
import actionlib_msgs.msg as action_msgs

# 假设的自定义消息
from common_msgs.msg import DetectedObjectArray # YOLO发出的带Header的数组
# from common_msgs.msg import GraspRequestAction # 建议改成 Action 形式调用 GraspNet

class MoveToOverviewBehavior(py_trees.behaviour.Behaviour):
    """阶段1：移动到高空俯视点，准备拍照"""
    def __init__(self, name="MoveToOverview"):
        super().__init__(name)
        # TODO: 初始化 MoveIt client 等
        
    def update(self):
        rospy.loginfo("[Brain] Moving to Overview Pose...")
        # 实际代码中这里应该是一个非阻塞的 MoveIt Action 检查
        # 假设到达了俯视点
        return Status.SUCCESS


class EvaluateTargetsBehavior(py_trees.behaviour.Behaviour):
    """阶段2：接收YOLO数据，TF转换，按比赛规则打分，选出最佳目标"""
    def __init__(self, name="EvaluateTargets"):
        super().__init__(name)
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.latest_msg = None
        self.sub = None

    def setup(self):
        rospy.loginfo("[Brain] EvaluateTargets: Setup")
        self.sub = rospy.Subscriber(
            "/perception/detected_objects",
            DetectedObjectArray,
            self._yolo_callback
        )
        return True

    def _yolo_callback(self, msg):
        self.latest_msg = msg

    def _score_object(self, obj_label):
        """比赛优先级打分系统"""
        score_map = {
            "green_cube": 100, "purple_cube": 100, # Cube 优先级最高（能称重刷分）
            "red_can": 80, "red_bottle": 80,       # 30分高优
            "yellow_can": 60, "spam": 60,          # 20分
            "green_can": 40, "blue_bottle": 40     # 10分保底
        }
        return score_map.get(obj_label.lower(), 0)

    def update(self):
        if self.latest_msg is None or len(self.latest_msg.objects) == 0:
            rospy.loginfo_throttle(2.0, "[Brain] Waiting for YOLO objects...")
            return Status.RUNNING

        best_target = None
        highest_score = -1
        best_point_base_link = None

        # 尝试获取图像拍摄那一刻的 TF
        try:
            trans = self.tf_buffer.lookup_transform(
                "base_link", 
                self.latest_msg.header.frame_id, 
                self.latest_msg.header.stamp, 
                rospy.Duration(0.5)
            )
            
            # 遍历所有物体，转换坐标并打分
            for obj in self.latest_msg.objects:
                # 构造相机的 PointStamped
                pt_camera = PointStamped()
                pt_camera.header = self.latest_msg.header
                pt_camera.point = obj.center_point # YOLO给的相机系下的3D点
                
                # TF 转换到 base_link
                pt_base = tf2_geometry_msgs.do_transform_point(pt_camera, trans)
                
                score = self._score_object(obj.label)
                if score > highest_score:
                    highest_score = score
                    best_target = obj
                    best_point_base_link = pt_base

            if best_target:
                rospy.loginfo(f"[Brain] Selected Target: {best_target.label}, Score: {highest_score}")
                # 写入黑板，供下一阶段使用
                blackboard = py_trees.blackboard.Blackboard()
                return Status.SUCCESS

        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            rospy.logwarn(f"[Brain] TF Error: {e}")
            return Status.FAILURE

        return Status.RUNNING


class RequestGraspPoseBehavior(py_trees.behaviour.Behaviour):
    """阶段3：把最佳 3D 点发给 GraspNet，等待抓取位姿 (建议用 Action/Service)"""
    def __init__(self, name="RequestGraspPose"):
        super().__init__(name)
        # TODO: 初始化与 GraspNet 通信的 Service/Action Client
        self.request_sent = False

    def update(self):
        if not self.request_sent:
            rospy.loginfo(f"[Brain] Requesting GraspNet to crop and process point: {target_point.point.x:.2f}, {target_point.point.y:.2f}")
            # 发送请求给 Muye 的节点...
            self.request_sent = True
            return Status.RUNNING
            
        # 假设这里非阻塞地检查 GraspNet 算完了没有
        # if graspnet_done:
        #     blackboard.set("target_grasp_pose", result_pose)
        #     self.request_sent = False
        #     return Status.SUCCESS
        
        return Status.RUNNING


class ExecutePickAndPlaceBehavior(py_trees.behaviour.Behaviour):
    """阶段4：非阻塞执行抓取与放置"""
    def __init__(self, name="ExecutePickAndPlace"):
        super().__init__(name)
        self.client = None
        self.goal_sent = False
        
    def setup(self):
        self.client = actionlib.SimpleActionClient('/move_group', MoveGroupAction)
        return True

    def update(self):
        # 初始化检查
        if not self.client.wait_for_server(rospy.Duration(0.1)):
            return Status.RUNNING

        # 1. 如果还没发指令，发送指令
        if not self.goal_sent:
            blackboard = py_trees.blackboard.Blackboard()
            target_pose = blackboard.get("target_grasp_pose")
            
            goal = MoveGroupGoal()
            # ... 填充 goal ...
            
            rospy.loginfo("[Brain] Sending Goal to MoveIt...")
            self.client.send_goal(goal)
            self.goal_sent = True
            return Status.RUNNING
            
        # 2. 如果已经发了指令，检查状态（这解决了你之前代码的阻塞问题！）
        state = self.client.get_state()
        
        if state in [action_msgs.GoalStatus.ACTIVE, action_msgs.GoalStatus.PENDING]:
            return Status.RUNNING # 机械臂正在动，树继续 Tick，不卡死！
            
        elif state == action_msgs.GoalStatus.SUCCEEDED:
            rospy.loginfo("[Brain] Pick and Place SUCCESS!")
            self.goal_sent = False # 重置状态，为下一次抓取做准备
            return Status.SUCCESS
            
        else:
            rospy.logerr(f"[Brain] MoveIt Failed with state: {state}")
            self.goal_sent = False
            return Status.FAILURE


def create_behavior_tree():
    """
    修改为循环抓取结构
    """
    # 主工作流序列（带记忆，成功后重置）
    main_sequence = py_trees.composites.Sequence(
        name="MainTaskSequence",
        memory=True, # 记住执行到了哪一步
        children=[
            MoveToOverviewBehavior(),
            EvaluateTargetsBehavior(),
            RequestGraspPoseBehavior(),
            ExecutePickAndPlaceBehavior()
        ]
    )
    
    # 将整个流程包装成一个循环（成功后继续下一次）
    root = py_trees.decorators.Loop(
        name="GameLoop",
        child=py_trees.composites.Selector(
            name="TaskOrRecovery",
            children=[
                main_sequence,
                # RecoveryBehavior() # 如果 MainTask 失败，走恢复逻辑
            ]
        )
    )
    return root

# ... main() 函数保持不变 ...

def main():
    rospy.init_node('robocup_brain', anonymous=False)
    rospy.loginfo("=" * 50)
    rospy.loginfo("RoboCup Brain Node Starting")
    rospy.loginfo("=" * 50)
    
    # 创建行为树
    root = create_behavior_tree()
    
    # 创建行为树管理器
    behaviour_tree = py_trees_ros.trees.BehaviourTree(root)
    
    # 设置更新频率
    rate = rospy.Rate(10)  # 10 Hz
    
    rospy.loginfo("[Brain] Behavior Tree initialized. Starting main loop...")
    
    try:
        while not rospy.is_shutdown():
            behaviour_tree.tick()
            rate.sleep()
    except KeyboardInterrupt:
        rospy.loginfo("[Brain] Shutting down...")

#11
if __name__ == '__main__':
    main()
