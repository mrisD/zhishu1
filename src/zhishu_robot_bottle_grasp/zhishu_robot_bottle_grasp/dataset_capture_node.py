#!/usr/bin/env python3
"""Capture Gazebo RGB frames and bootstrap YOLO segmentation labels.

The red simulation material is used only to generate ground-truth masks.  The
resulting model still performs normal image inference at runtime.
"""

from pathlib import Path

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image


class DatasetCaptureNode(Node):
    def __init__(self):
        super().__init__("bottle_dataset_capture")
        self.declare_parameter("image_topic", "/camera_head/color/image_raw")
        self.declare_parameter("output_dir", "/tmp/bottle_yolo_dataset")
        self.declare_parameter("max_images", 80)
        self.declare_parameter("sample_period", 0.35)
        self.declare_parameter("minimum_area", 180)
        self._bridge = CvBridge()
        self._last_stamp = None
        self._count = 0
        self._root = Path(self.get_parameter("output_dir").value)
        for split in ("train", "val"):
            (self._root / "images" / split).mkdir(parents=True, exist_ok=True)
            (self._root / "labels" / split).mkdir(parents=True, exist_ok=True)
        self.create_subscription(
            Image,
            self.get_parameter("image_topic").value,
            self._on_image,
            qos_profile_sensor_data,
        )
        self.get_logger().info(f"Capturing YOLO dataset into {self._root}")

    def _on_image(self, message):
        stamp = message.header.stamp.sec + message.header.stamp.nanosec * 1e-9
        period = float(self.get_parameter("sample_period").value)
        if self._last_stamp is not None and stamp - self._last_stamp < period:
            return
        image = self._bridge.imgmsg_to_cv2(message, desired_encoding="bgr8")
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, (0, 80, 45), (12, 255, 255))
        mask |= cv2.inRange(hsv, (168, 80, 45), (180, 255, 255))
        ys, xs = np.nonzero(mask)
        if xs.size < int(self.get_parameter("minimum_area").value):
            return
        points = np.column_stack((xs, ys)).astype(np.int32)
        hull = cv2.convexHull(points).reshape(-1, 2)
        if len(hull) < 3:
            return
        height, width = image.shape[:2]
        normalized = []
        for x, y in hull:
            normalized.extend((float(x) / width, float(y) / height))
        split = "val" if self._count % 5 == 0 else "train"
        stem = f"sim_{self._count:05d}"
        cv2.imwrite(str(self._root / "images" / split / f"{stem}.jpg"), image)
        label = "0 " + " ".join(f"{value:.6f}" for value in normalized) + "\n"
        (self._root / "labels" / split / f"{stem}.txt").write_text(label)
        self._count += 1
        self._last_stamp = stamp
        if self._count % 10 == 0:
            self.get_logger().info(f"Captured {self._count} labelled frames")
        if self._count >= int(self.get_parameter("max_images").value):
            self.get_logger().info("Dataset capture complete")
            rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = DatasetCaptureNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass
        if rclpy.ok():
            rclpy.shutdown()
