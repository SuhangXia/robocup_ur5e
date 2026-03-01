#!/usr/bin/env python3
"""
JUST FOR SAMPLE, NOT USED IN THE PROJECT
YOLO Object Detector Node
Authors: Fazhan & Ruiyi
Environment: CUDA 12.0 + ROS Noetic (with CPU fallback for Mac/non-NVIDIA users)
"""

import rospy
import cv2
import torch
import numpy as np
from cv_bridge import CvBridge, CvBridgeError

from sensor_msgs.msg import Image, RegionOfInterest
from common_msgs.msg import DetectedObject


class YOLODetectorNode:
    def __init__(self):
        rospy.init_node('yolo_detector', anonymous=False)
        rospy.loginfo("=" * 60)
        rospy.loginfo("YOLO Detector Node Initializing")
        rospy.loginfo("=" * 60)
        
        # 设备选择：优先使用 CUDA，回退到 CPU
        self.device = self._setup_device()
        
        # 参数配置
        self.model_path = rospy.get_param('~model_path', '/workspace/weights/yolo/yolov8n.pt')
        self.confidence_threshold = rospy.get_param('~confidence_threshold', 0.5)
        self.image_topic = rospy.get_param('~image_topic', '/camera/color/image_raw')
        
        # 加载 YOLO 模型
        self.model = self._load_model()
        
        # ROS 接口
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber(
            self.image_topic,
            Image,
            self.image_callback,
            queue_size=1,
            buff_size=2**24  # 16MB buffer
        )
        
        self.detection_pub = rospy.Publisher(
            '/perception/detected_objects',
            DetectedObject,
            queue_size=10
        )
        
        self.viz_pub = rospy.Publisher(
            '/perception/detection_viz',
            Image,
            queue_size=1
        )
        
        rospy.loginfo(f"[YOLO] Device: {self.device}")
        rospy.loginfo(f"[YOLO] Model: {self.model_path}")
        rospy.loginfo(f"[YOLO] Subscribing to: {self.image_topic}")
        rospy.loginfo("[YOLO] Initialization complete. Ready to detect!")
        
    def _setup_device(self):
        """设置计算设备，支持 Mac M-chip 和非 NVIDIA GPU"""
        if torch.cuda.is_available():
            device = torch.device('cuda')
            rospy.loginfo(f"[YOLO] CUDA available: {torch.cuda.get_device_name(0)}")
            rospy.loginfo(f"[YOLO] CUDA version: {torch.version.cuda}")
        elif torch.backends.mps.is_available():
            # Mac M-chip 支持
            device = torch.device('mps')
            rospy.loginfo("[YOLO] Using Apple Metal Performance Shaders (MPS)")
        else:
            device = torch.device('cpu')
            rospy.logwarn("[YOLO] No GPU detected. Running on CPU (slower inference)")
        
        return device
        
    def _load_model(self):
        """加载 YOLO 模型（支持 YOLOv8 或其他版本）"""
        try:
            from ultralytics import YOLO
            model = YOLO(self.model_path)
            model.to(self.device)
            rospy.loginfo(f"[YOLO] Model loaded successfully: {self.model_path}")
            return model
        except ImportError:
            rospy.logerr("[YOLO] ultralytics not installed. Install with: pip install ultralytics")
            raise
        except Exception as e:
            rospy.logerr(f"[YOLO] Failed to load model: {e}")
            raise
            
    def image_callback(self, msg):
        """处理图像并执行物体检测"""
        try:
            # ROS Image -> OpenCV
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except CvBridgeError as e:
            rospy.logerr(f"[YOLO] CV Bridge Error: {e}")
            return
            
        # 执行推理
        results = self.model(cv_image, verbose=False)
        
        # 解析结果并发布
        for result in results:
            boxes = result.boxes
            
            if boxes is None or len(boxes) == 0:
                continue
                
            for box in boxes:
                # 提取检测信息
                conf = float(box.conf[0])
                
                if conf < self.confidence_threshold:
                    continue
                    
                cls_id = int(box.cls[0])
                label = result.names[cls_id]
                
                # 边界框 (xyxy format)
                x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
                
                # 构造 ROI 消息
                roi = RegionOfInterest()
                roi.x_offset = int(x1)
                roi.y_offset = int(y1)
                roi.width = int(x2 - x1)
                roi.height = int(y2 - y1)
                roi.do_rectify = False
                
                # 发布 DetectedObject
                detection_msg = DetectedObject()
                detection_msg.label = label
                detection_msg.score = conf
                detection_msg.roi = roi
                
                self.detection_pub.publish(detection_msg)
                rospy.loginfo(f"[YOLO] Detected: {label} ({conf:.2f})")
                
        # 可视化（可选）
        self._publish_visualization(cv_image, results)
        
    def _publish_visualization(self, image, results):
        """发布带检测框的可视化图像"""
        try:
            # 使用 ultralytics 的内置绘图
            annotated_frame = results[0].plot()
            
            # OpenCV -> ROS Image
            viz_msg = self.bridge.cv2_to_imgmsg(annotated_frame, encoding='bgr8')
            self.viz_pub.publish(viz_msg)
        except Exception as e:
            rospy.logwarn(f"[YOLO] Visualization failed: {e}")
            
    def run(self):
        """保持节点运行"""
        rospy.spin()


if __name__ == '__main__':
    try:
        node = YOLODetectorNode()
        node.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("[YOLO] Shutting down")
