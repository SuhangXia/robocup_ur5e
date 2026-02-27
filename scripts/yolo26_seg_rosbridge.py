#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
GPU YOLO26-Seg 语义分割 + 3D 掩码点云
通过 rosbridge 订阅 RGB/Depth/CameraInfo，
发布:
  - /perception/yolo26_seg_detections  (std_msgs/String, JSON)
  - /perception/yolo26_seg_cloud       (sensor_msgs/PointCloud2, 带 label 的 3D 点云)
"""

import argparse
import base64
import json
import struct
import time

import cv2
import numpy as np
import roslibpy
from ultralytics import YOLO


def decode_image(msg):
    data = msg["data"]
    if isinstance(data, str):
        data = base64.b64decode(data)
    else:
        data = bytes(data)

    h = msg["height"]
    w = msg["width"]
    encoding = msg["encoding"]

    if encoding == "rgb8":
        img = np.frombuffer(data, dtype=np.uint8).reshape(h, w, 3)
        return cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
    if encoding == "bgr8":
        return np.frombuffer(data, dtype=np.uint8).reshape(h, w, 3)
    if encoding == "mono8":
        return np.frombuffer(data, dtype=np.uint8).reshape(h, w)

    raise ValueError(f"Unsupported encoding: {encoding}")


def decode_depth(msg):
    data = msg["data"]
    if isinstance(data, str):
        data = base64.b64decode(data)
    else:
        data = bytes(data)

    h = msg["height"]
    w = msg["width"]
    encoding = msg["encoding"]

    if encoding == "16UC1":
        depth = np.frombuffer(data, dtype=np.uint16).reshape(h, w)
        return depth.astype(np.float32) / 1000.0
    if encoding == "32FC1":
        return np.frombuffer(data, dtype=np.float32).reshape(h, w)

    raise ValueError(f"Unsupported depth encoding: {encoding}")


def mask_to_3d_points(mask, depth, fx, fy, cx, cy, max_points=5000):
    """将 2D 掩码 + 深度图转成 3D 点云，下采样避免数据量过大"""
    vs, us = np.where(mask > 0)
    if len(vs) == 0:
        return np.empty((0, 3), dtype=np.float32)

    if len(vs) > max_points:
        indices = np.random.choice(len(vs), max_points, replace=False)
        vs = vs[indices]
        us = us[indices]

    zs = depth[vs, us].astype(np.float32)
    valid = (zs > 0) & (zs < 10.0)
    vs, us, zs = vs[valid], us[valid], zs[valid]

    xs = (us.astype(np.float32) - cx) * zs / fx
    ys = (vs.astype(np.float32) - cy) * zs / fy

    return np.stack([xs, ys, zs], axis=1)


def build_pointcloud2_msg(all_points, all_labels, frame_id="camera_rgb_optical_frame"):
    """构建 PointCloud2 消息（XYZL: float32 x3 + uint32 label）"""
    fields = [
        {"name": "x", "offset": 0, "datatype": 7, "count": 1},
        {"name": "y", "offset": 4, "datatype": 7, "count": 1},
        {"name": "z", "offset": 8, "datatype": 7, "count": 1},
        {"name": "label", "offset": 12, "datatype": 6, "count": 1},
    ]
    point_step = 16
    n = len(all_points)

    buf = bytearray(n * point_step)
    for i in range(n):
        struct.pack_into("fffI", buf, i * point_step,
                         all_points[i][0], all_points[i][1], all_points[i][2],
                         all_labels[i])

    now_sec = int(time.time())
    now_nsec = int((time.time() - now_sec) * 1e9)

    return {
        "header": {
            "seq": 0,
            "stamp": {"secs": now_sec, "nsecs": now_nsec},
            "frame_id": frame_id,
        },
        "height": 1,
        "width": n,
        "fields": fields,
        "is_bigendian": False,
        "point_step": point_step,
        "row_step": n * point_step,
        "data": base64.b64encode(bytes(buf)).decode("ascii"),
        "is_dense": True,
    }


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=9090)
    parser.add_argument("--model", default="/workspace/weights/yolo/yolo26m-seg.pt")
    parser.add_argument("--rgb", default="/camera/rgb/image_raw")
    parser.add_argument("--depth", default="/camera/depth/image_raw")
    parser.add_argument("--camera-info", default="/camera/rgb/camera_info")
    parser.add_argument("--conf", type=float, default=0.5)
    parser.add_argument("--print-interval", type=float, default=0.3)
    parser.add_argument("--max-points-per-obj", type=int, default=3000)
    args = parser.parse_args()

    model = YOLO(args.model)
    print(f"Loaded seg model: {args.model}")

    last_print = 0.0
    latest_depth = {"msg": None}
    cam = {"fx": None, "fy": None, "cx": None, "cy": None}

    ros = roslibpy.Ros(host=args.host, port=args.port)
    ros.run()
    print(f"Connected to rosbridge at {args.host}:{args.port}")

    depth_topic = roslibpy.Topic(ros, args.depth, "sensor_msgs/Image")
    rgb_topic = roslibpy.Topic(ros, args.rgb, "sensor_msgs/Image")
    info_topic = roslibpy.Topic(ros, args.camera_info, "sensor_msgs/CameraInfo")
    det_topic = roslibpy.Topic(ros, "/perception/yolo26_seg_detections", "std_msgs/String")
    cloud_topic = roslibpy.Topic(ros, "/perception/yolo26_seg_cloud", "sensor_msgs/PointCloud2")

    def on_camera_info(msg):
        if cam["fx"] is None:
            K = msg["K"]
            cam["fx"], cam["fy"] = K[0], K[4]
            cam["cx"], cam["cy"] = K[2], K[5]
            print(f"Camera intrinsics: fx={cam['fx']:.1f} fy={cam['fy']:.1f} "
                  f"cx={cam['cx']:.1f} cy={cam['cy']:.1f}")

    def on_depth(msg):
        latest_depth["msg"] = msg

    def on_rgb(msg):
        nonlocal last_print
        now = time.time()
        if now - last_print < args.print_interval:
            return
        if cam["fx"] is None:
            return

        frame = decode_image(msg)
        depth_msg = latest_depth["msg"]
        depth = None
        if depth_msg is not None:
            try:
                depth = decode_depth(depth_msg)
            except Exception:
                depth = None

        results = model(frame, verbose=False)
        if not results:
            return

        h, w = frame.shape[:2]
        detections = []
        all_cloud_points = []
        all_cloud_labels = []

        for result in results:
            if result.boxes is None:
                continue

            masks_data = result.masks
            for i, box in enumerate(result.boxes):
                conf = float(box.conf[0])
                if conf < args.conf:
                    continue

                cls_id = int(box.cls[0])
                label = result.names[cls_id]
                x1, y1, x2, y2 = box.xyxy[0].cpu().numpy().astype(int)
                x1 = max(0, min(x1, w - 1))
                x2 = max(0, min(x2, w - 1))
                y1 = max(0, min(y1, h - 1))
                y2 = max(0, min(y2, h - 1))
                cx_px = int((x1 + x2) / 2)
                cy_px = int((y1 + y2) / 2)

                dist = None
                if depth is not None:
                    try:
                        dist = float(depth[cy_px, cx_px])
                    except Exception:
                        pass

                pos_3d = None
                if dist is not None and dist > 0 and cam["fx"] is not None:
                    px = (cx_px - cam["cx"]) * dist / cam["fx"]
                    py = (cy_px - cam["cy"]) * dist / cam["fy"]
                    pos_3d = [round(px, 3), round(py, 3), round(dist, 3)]

                mask_points_count = 0
                if masks_data is not None and i < len(masks_data.data) and depth is not None:
                    mask_2d = masks_data.data[i].cpu().numpy()
                    mask_resized = cv2.resize(mask_2d, (w, h), interpolation=cv2.INTER_NEAREST)
                    binary_mask = (mask_resized > 0.5).astype(np.uint8)

                    pts_3d = mask_to_3d_points(
                        binary_mask, depth,
                        cam["fx"], cam["fy"], cam["cx"], cam["cy"],
                        max_points=args.max_points_per_obj
                    )
                    mask_points_count = len(pts_3d)

                    for pt in pts_3d:
                        all_cloud_points.append(pt)
                        all_cloud_labels.append(cls_id)

                det = {
                    "label": label,
                    "confidence": round(conf, 2),
                    "bbox": [int(x1), int(y1), int(x2), int(y2)],
                    "center": [cx_px, cy_px],
                    "distance_m": round(dist, 2) if dist is not None else None,
                    "position_camera": pos_3d,
                    "mask_3d_points": mask_points_count,
                }
                detections.append(det)

                print(
                    f"Detected {label} conf={conf:.2f} "
                    f"bbox=({x1},{y1},{x2},{y2}) center=({cx_px},{cy_px}) "
                    f"distance={'n/a' if dist is None else f'{dist:.2f}m'} "
                    f"pos_cam={pos_3d if pos_3d else 'n/a'} "
                    f"mask_3d_pts={mask_points_count}"
                )

        if detections:
            det_topic.publish(roslibpy.Message({
                "data": json.dumps(detections, ensure_ascii=False)
            }))

        if all_cloud_points:
            pc2_msg = build_pointcloud2_msg(all_cloud_points, all_cloud_labels)
            cloud_topic.publish(roslibpy.Message(pc2_msg))

        last_print = now

    info_topic.subscribe(on_camera_info)
    depth_topic.subscribe(on_depth)
    rgb_topic.subscribe(on_rgb)

    print("Waiting for data...")
    try:
        while ros.is_connected:
            time.sleep(0.5)
    except KeyboardInterrupt:
        pass
    finally:
        info_topic.unsubscribe()
        depth_topic.unsubscribe()
        rgb_topic.unsubscribe()
        det_topic.unadvertise()
        cloud_topic.unadvertise()
        ros.terminate()
        print("Shutdown complete.")


if __name__ == "__main__":
    main()
