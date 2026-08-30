#!/usr/bin/env python3
"""Capture one head-camera frame and report visible ArUco target markers."""

import sys
import time

import cv2
from cv_bridge import CvBridge
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image


class CameraTargetCheck(Node):
    def __init__(self):
        super().__init__("handeye_camera_target_check")
        self.declare_parameter("image_topic", "/camera_head/color/image_raw")
        self.declare_parameter("output_path", "/tmp/handeye_camera_check.png")
        self.bridge = CvBridge()
        self.marker_count = None
        topic = self.get_parameter("image_topic").value
        self.create_subscription(Image, topic, self.on_image, qos_profile_sensor_data)

    def on_image(self, message):
        if self.marker_count is not None:
            return
        image = self.bridge.imgmsg_to_cv2(message, desired_encoding="bgr8")
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_5X5_250)
        parameters = cv2.aruco.DetectorParameters_create()
        corners, ids, _ = cv2.aruco.detectMarkers(
            gray, dictionary, parameters=parameters
        )
        self.marker_count = 0 if ids is None else len(ids)
        if ids is not None:
            cv2.aruco.drawDetectedMarkers(image, corners, ids)
        output_path = self.get_parameter("output_path").value
        cv2.imwrite(output_path, image)
        self.get_logger().info(
            f"captured {image.shape[1]}x{image.shape[0]}, "
            f"detected {self.marker_count} marker(s), saved {output_path}"
        )


def main():
    rclpy.init()
    node = CameraTargetCheck()
    deadline = time.monotonic() + 10.0
    while rclpy.ok() and node.marker_count is None and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.2)
    success = node.marker_count is not None
    if not success:
        node.get_logger().error("no camera image received within 10 seconds")
    node.destroy_node()
    rclpy.shutdown()
    return 0 if success else 1


if __name__ == "__main__":
    sys.exit(main())
