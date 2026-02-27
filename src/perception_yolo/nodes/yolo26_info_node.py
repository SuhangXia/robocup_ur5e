#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
YOLO26 检测节点（终端输出）
订阅:
  - /camera/rgb/image_raw
  - /camera/depth/image_raw
"""

import os
import time

import message_filters
import numpy as np
import rospy
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image


class Yolo26InfoNode:
    def __init__(self):
        rospy.init_node("yolo26_info", anonymous=True)

        self.bridge = CvBridge()
        self.model_path = rospy.get_param("~model_path", "/workspace/weights/yolo/yolo26m.pt")
        self.conf_threshold = float(rospy.get_param("~confidence_threshold", 0.5))
        self.image_topic = rospy.get_param("~image_topic", "/camera/rgb/image_raw")
        self.depth_topic = rospy.get_param("~depth_topic", "/camera/depth/image_raw")
        self.print_interval = float(rospy.get_param("~print_interval", 0.2))
        self._last_print = 0.0

        try:
            from ultralytics import YOLO
        except Exception as exc:
            raise RuntimeError(f"ultralytics not installed: {exc}")

        if not os.path.exists(self.model_path):
            raise FileNotFoundError(self.model_path)

        self.model = YOLO(self.model_path)
        rospy.loginfo("YOLO model: %s", self.model_path)
        rospy.loginfo("Subscribing RGB: %s", self.image_topic)
        rospy.loginfo("Subscribing Depth: %s", self.depth_topic)
        rospy.loginfo("Confidence threshold: %.2f", self.conf_threshold)

        rgb_sub = message_filters.Subscriber(self.image_topic, Image)
        depth_sub = message_filters.Subscriber(self.depth_topic, Image)
        self.sync = message_filters.ApproximateTimeSynchronizer(
            [rgb_sub, depth_sub], queue_size=10, slop=0.1
        )
        self.sync.registerCallback(self.image_callback)

    def image_callback(self, rgb_msg, depth_msg):
        now = time.time()
        if now - self._last_print < self.print_interval:
            return

        try:
            frame = self.bridge.imgmsg_to_cv2(rgb_msg, desired_encoding="bgr8")
            depth = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding="passthrough")
        except CvBridgeError as exc:
            rospy.logerr("CvBridge error: %s", str(exc))
            return

        results = self.model(frame, verbose=False)
        if not results:
            return

        h, w = frame.shape[:2]
        for result in results:
            if result.boxes is None:
                continue
            for box in result.boxes:
                conf = float(box.conf[0])
                if conf < self.conf_threshold:
                    continue
                cls_id = int(box.cls[0])
                label = result.names[cls_id]
                x1, y1, x2, y2 = box.xyxy[0].cpu().numpy().astype(int)

                x1 = max(0, min(x1, w - 1))
                x2 = max(0, min(x2, w - 1))
                y1 = max(0, min(y1, h - 1))
                y2 = max(0, min(y2, h - 1))

                cx = int((x1 + x2) / 2)
                cy = int((y1 + y2) / 2)

                dist = None
                try:
                    d = depth[cy, cx]
                    if np.issubdtype(depth.dtype, np.integer):
                        dist = float(d) / 1000.0
                    else:
                        dist = float(d)
                except Exception:
                    dist = None

                rospy.loginfo(
                    "Detected %s conf=%.2f bbox=(%d,%d,%d,%d) center=(%d,%d) distance=%s",
                    label, conf, x1, y1, x2, y2, cx, cy,
                    f"{dist:.2f}m" if dist is not None else "n/a"
                )

        self._last_print = now

    def run(self):
        rospy.spin()


if __name__ == "__main__":
    try:
        node = Yolo26InfoNode()
        node.run()
    except rospy.ROSInterruptException:
        pass
