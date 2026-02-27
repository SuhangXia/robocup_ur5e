#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
获取 Gazebo 相机视角并保存图像
订阅: /camera/rgb/image_raw
保存: /tmp/camera_view_*.jpg
"""

import os
import time
import atexit
import sys
import rospy
import cv2
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image

# #region agent log
def _debug_log(hypothesis_id, location, message, data=None, run_id="run1"):
    try:
        payload = {
            "id": f"log_{int(time.time()*1000)}",
            "timestamp": int(time.time() * 1000),
            "location": location,
            "message": message,
            "data": data or {},
            "runId": run_id,
            "hypothesisId": hypothesis_id,
        }
        log_path = "/home/liufazhan/robocup_ur5e/.cursor/debug.log"
        os.makedirs(os.path.dirname(log_path), exist_ok=True)
        with open(log_path, "a", encoding="utf-8") as f:
            f.write(str(payload).replace("'", "\"") + "\n")
    except Exception:
        pass
# #endregion agent log

# #region agent log
def _log_uncaught(exc_type, exc, tb):
    _debug_log("H1", "camera_view_capture_node.py:excepthook", "uncaught_exception", {
        "type": str(exc_type),
        "error": str(exc)
    })
    sys.__excepthook__(exc_type, exc, tb)

def _log_exit():
    _debug_log("H1", "camera_view_capture_node.py:atexit", "process_exit")

sys.excepthook = _log_uncaught
atexit.register(_log_exit)
# #endregion agent log


class CameraViewCapture:
    def __init__(self):
        # #region agent log
        _debug_log("H1", "camera_view_capture_node.py:__init__", "init_start")
        # #endregion agent log
        # #region agent log
        _debug_log("H1", "camera_view_capture_node.py:__init__", "env_vars", {
            "ROS_MASTER_URI": os.environ.get("ROS_MASTER_URI"),
            "ROS_IP": os.environ.get("ROS_IP")
        })
        # #endregion agent log
        try:
            rospy.init_node('camera_view_capture', anonymous=True)
        except BaseException as e:
            # #region agent log
            _debug_log("H1", "camera_view_capture_node.py:__init__", "init_node_failed", {"error": str(e)})
            # #endregion agent log
            raise
        # #region agent log
        _debug_log("H1", "camera_view_capture_node.py:__init__", "after_init_node")
        # #endregion agent log
        self.bridge = CvBridge()
        # #region agent log
        _debug_log("H2", "camera_view_capture_node.py:__init__", "bridge_created")
        # #endregion agent log
        self.image_topic = rospy.get_param('~image_topic', '/camera/rgb/image_raw')
        self.save_dir = rospy.get_param('~save_dir', '/tmp')
        self.save_interval = float(rospy.get_param('~save_interval', 1.0))
        self.last_save_time = 0.0
        # #region agent log
        _debug_log("H2", "camera_view_capture_node.py:__init__", "params_loaded", {
            "image_topic": self.image_topic,
            "save_dir": self.save_dir,
            "save_interval": self.save_interval
        })
        # #endregion agent log

        os.makedirs(self.save_dir, exist_ok=True)
        rospy.loginfo("订阅图像话题: %s", self.image_topic)
        rospy.loginfo("保存目录: %s", self.save_dir)
        rospy.loginfo("保存间隔: %.2f 秒", self.save_interval)
        # #region agent log
        _debug_log("H2", "camera_view_capture_node.py:config", "config_loaded", {
            "topic": self.image_topic,
            "save_dir": self.save_dir,
            "save_interval": self.save_interval
        })
        # #endregion agent log

        self.sub = rospy.Subscriber(self.image_topic, Image, self.callback, queue_size=1)
        # #region agent log
        _debug_log("H3", "camera_view_capture_node.py:__init__", "subscriber_created", {"topic": self.image_topic})
        # #endregion agent log
        self._received_count = 0
        # #region agent log
        _debug_log("H3", "camera_view_capture_node.py:__init__", "init_complete")
        # #endregion agent log

        def _check_messages(_event):
            if self._received_count == 0:
                try:
                    topics = rospy.get_published_topics()
                    topic_names = [t[0] for t in topics]
                except Exception as e:
                    topic_names = []
                    _debug_log("H4", "camera_view_capture_node.py:check_messages", "topics_error", {"error": str(e)})
                _debug_log("H4", "camera_view_capture_node.py:check_messages", "no_messages", {
                    "topic": self.image_topic,
                    "published_topics_count": len(topic_names),
                    "published_topics_sample": topic_names[:10]
                })
        rospy.Timer(rospy.Duration(5.0), _check_messages, oneshot=True)

    def callback(self, msg):
        try:
            self._received_count += 1
            # #region agent log
            _debug_log("H3", "camera_view_capture_node.py:callback", "callback_start", {
                "encoding": getattr(msg, "encoding", None),
                "width": getattr(msg, "width", None),
                "height": getattr(msg, "height", None)
            })
            # #endregion agent log
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except CvBridgeError as e:
            rospy.logerr("CvBridge 转换失败: %s", str(e))
            # #region agent log
            _debug_log("H4", "camera_view_capture_node.py:callback", "cv_bridge_error", {"error": str(e)})
            # #endregion agent log
            return

        # 当前 ROS 时间（通常在 Gazebo 中为仿真时间）
        now_time = rospy.Time.now()
        now = now_time.to_sec()
        if now - self.last_save_time < self.save_interval:
            # #region agent log
            _debug_log("H5", "camera_view_capture_node.py:callback", "skip_save_interval", {
                "now": now,
                "last_save_time": self.last_save_time
            })
            # #endregion agent log
            return

        # 图像对应的时间戳：优先使用消息头中的时间（Gazebo 仿真时间）
        sim_stamp = getattr(msg, "header", None).stamp if hasattr(msg, "header") else now_time
        sim_time = sim_stamp.to_sec()

        filename = os.path.join(self.save_dir, f"camera_view_{int(sim_time*1000)}.jpg")
        cv2.imwrite(filename, frame)
        self.last_save_time = now
        rospy.loginfo("已保存图像: %s | sim_time=%.3f (sec=%d, nsec=%d)",
                      filename, sim_time, sim_stamp.secs, sim_stamp.nsecs)
        # #region agent log
        _debug_log("H6", "camera_view_capture_node.py:callback", "saved_image", {"filename": filename})
        # #endregion agent log


if __name__ == '__main__':
    try:
        node = CameraViewCapture()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
