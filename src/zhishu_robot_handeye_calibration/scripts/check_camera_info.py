#!/usr/bin/env python3
"""Validate that the real RGB image and calibrated CameraInfo agree."""

import math
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image


def stamp_seconds(stamp):
    """Convert a ROS builtin time message to floating-point seconds."""
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


class CameraInfoCheck(Node):
    """Wait for one RGB image/CameraInfo pair and validate their contract."""

    def __init__(self):
        """Declare expected stream properties and create subscriptions."""
        super().__init__("gemini_pro_camera_info_check")
        self.image_topic = self.declare_parameter(
            "image_topic", "/camera_head/color/image_raw"
        ).value
        self.info_topic = self.declare_parameter(
            "camera_info_topic", "/camera_head/color/camera_info"
        ).value
        self.expected_width = self.declare_parameter("expected_width", 640).value
        self.expected_height = self.declare_parameter("expected_height", 480).value
        self.expected_frame_id = self.declare_parameter(
            "expected_frame_id", "head_camera_optical_frame"
        ).value
        self.image = None
        self.camera_info = None
        self.create_subscription(
            Image, self.image_topic, self._on_image, qos_profile_sensor_data
        )
        self.create_subscription(
            CameraInfo, self.info_topic, self._on_camera_info, qos_profile_sensor_data
        )

    def _on_image(self, message):
        self.image = message

    def _on_camera_info(self, message):
        self.camera_info = message

    def ready(self):
        """Return whether both required messages have arrived."""
        return self.image is not None and self.camera_info is not None

    def validate(self):
        """Validate calibration values, dimensions, frame IDs, and timestamps."""
        errors = []
        warnings = []
        image = self.image
        info = self.camera_info

        if image.width != self.expected_width or image.height != self.expected_height:
            errors.append(
                f"image size is {image.width}x{image.height}, expected "
                f"{self.expected_width}x{self.expected_height}"
            )
        if info.width != image.width or info.height != image.height:
            errors.append(
                f"CameraInfo size {info.width}x{info.height} does not match image "
                f"{image.width}x{image.height}"
            )
        if image.header.frame_id != self.expected_frame_id:
            errors.append(
                f"image frame_id is '{image.header.frame_id}', expected "
                f"'{self.expected_frame_id}'"
            )
        if info.header.frame_id != image.header.frame_id:
            errors.append(
                f"CameraInfo frame_id '{info.header.frame_id}' does not match image "
                f"frame_id '{image.header.frame_id}'"
            )
        if len(info.k) != 9 or info.k[0] <= 0.0 or info.k[4] <= 0.0:
            errors.append("CameraInfo K is missing or has non-positive focal lengths")
        elif not math.isclose(info.k[8], 1.0, rel_tol=0.0, abs_tol=1e-9):
            errors.append("CameraInfo K[8] must be 1")
        if len(info.p) != 12 or info.p[0] <= 0.0 or info.p[5] <= 0.0:
            errors.append("CameraInfo P is missing or has non-positive focal lengths")
        if info.distortion_model != "plumb_bob":
            warnings.append(
                f"distortion_model is '{info.distortion_model}', expected 'plumb_bob'"
            )
        if len(info.d) < 5:
            errors.append("CameraInfo D must contain at least five coefficients")

        image_time = stamp_seconds(image.header.stamp)
        info_time = stamp_seconds(info.header.stamp)
        if image_time > 0.0 and info_time > 0.0:
            delta = abs(image_time - info_time)
            if delta > 0.1:
                warnings.append(
                    f"latest image and CameraInfo stamps differ by {delta:.3f} s"
                )

        for warning in warnings:
            self.get_logger().warning(warning)
        for error in errors:
            self.get_logger().error(error)
        if errors:
            return False

        self.get_logger().info(
            "PASS: RGB image and CameraInfo are consistent: "
            f"{image.width}x{image.height}, encoding={image.encoding}, "
            f"frame_id={image.header.frame_id}, fx={info.k[0]:.3f}, "
            f"fy={info.k[4]:.3f}, cx={info.k[2]:.3f}, cy={info.k[5]:.3f}"
        )
        return True


def main():
    """Wait for camera messages, report validation, and return a shell status."""
    rclpy.init()
    node = CameraInfoCheck()
    deadline = time.monotonic() + 10.0
    while rclpy.ok() and not node.ready() and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.2)

    if not node.ready():
        node.get_logger().error(
            f"timed out waiting for {node.image_topic} and {node.info_topic}"
        )
        success = False
    else:
        success = node.validate()

    node.destroy_node()
    rclpy.shutdown()
    return 0 if success else 1


if __name__ == "__main__":
    sys.exit(main())
