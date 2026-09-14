#!/usr/bin/env python3
"""Relay Gazebo CameraInfo to the image_transport conventional topic."""

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo


class CameraInfoRelay(Node):
    def __init__(self):
        super().__init__("handeye_camera_info_relay")
        self.declare_parameter("input_topic", "/camera_head/camera_info")
        self.declare_parameter("output_topic", "/camera_head/color/camera_info")
        output_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.publisher = self.create_publisher(
            CameraInfo, self.get_parameter("output_topic").value, output_qos
        )
        self.subscription = self.create_subscription(
            CameraInfo,
            self.get_parameter("input_topic").value,
            self._relay,
            qos_profile_sensor_data,
        )
        self.get_logger().info(
            "Relaying CameraInfo from %s to %s"
            % (
                self.get_parameter("input_topic").value,
                self.get_parameter("output_topic").value,
            )
        )

    def _relay(self, message):
        self.publisher.publish(message)


def main():
    rclpy.init()
    node = CameraInfoRelay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
