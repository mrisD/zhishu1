#!/usr/bin/env python3
"""Capture one head-camera frame and report visible ChArUco target features."""

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
        self.declare_parameter("minimum_markers", 1)
        self.declare_parameter("minimum_charuco_corners", 4)
        self.bridge = CvBridge()
        self.marker_count = None
        self.charuco_corner_count = None
        topic = self.get_parameter("image_topic").value
        self.create_subscription(Image, topic, self.on_image, qos_profile_sensor_data)

    def on_image(self, message):
        if self.marker_count is not None:
            return
        image = self.bridge.imgmsg_to_cv2(message, desired_encoding="bgr8")
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        board = cv2.aruco.CharucoBoard_create(7, 5, 0.030, 0.022, dictionary)
        parameters = cv2.aruco.DetectorParameters_create()
        corners, ids, _ = cv2.aruco.detectMarkers(
            gray, dictionary, parameters=parameters
        )
        self.marker_count = 0 if ids is None else len(ids)
        self.charuco_corner_count = 0
        if ids is not None:
            cv2.aruco.drawDetectedMarkers(image, corners, ids)
            _, charuco_corners, charuco_ids = cv2.aruco.interpolateCornersCharuco(
                corners, ids, gray, board
            )
            if charuco_ids is not None:
                self.charuco_corner_count = len(charuco_ids)
                cv2.aruco.drawDetectedCornersCharuco(
                    image, charuco_corners, charuco_ids
                )
        output_path = self.get_parameter("output_path").value
        cv2.imwrite(output_path, image)
        self.get_logger().info(
            f"captured {image.shape[1]}x{image.shape[0]}, "
            f"detected {self.marker_count} marker(s) and "
            f"{self.charuco_corner_count} ChArUco corner(s), saved {output_path}"
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
        node.get_logger().error(
            "The target cannot be sampled without an image stream. Check the "
            "Gemini Pro USB connection, or start the Gazebo workflow with: ros2 "
            "launch zhishu_robot_handeye_calibration handeye_gazebo.launch.py"
        )
    else:
        minimum_markers = node.get_parameter("minimum_markers").value
        minimum_charuco_corners = node.get_parameter("minimum_charuco_corners").value
        if node.marker_count < minimum_markers:
            node.get_logger().error(
                f"detected {node.marker_count} marker(s), expected at least "
                f"{minimum_markers}"
            )
            success = False
        if node.charuco_corner_count < minimum_charuco_corners:
            node.get_logger().error(
                f"detected {node.charuco_corner_count} ChArUco corner(s), expected "
                f"at least {minimum_charuco_corners}"
            )
            success = False
    node.destroy_node()
    rclpy.shutdown()
    return 0 if success else 1


if __name__ == "__main__":
    sys.exit(main())
