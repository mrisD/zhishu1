#!/usr/bin/env python3
"""Reject calibration samples when the movable head has left its lock pose."""

import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


class HeadLockGuard(Node):
    def __init__(self):
        super().__init__("handeye_head_lock_guard")
        self.yaw_target = self.declare_parameter("head_yaw_target", 0.0).value
        self.pitch_target = self.declare_parameter("head_pitch_target", 0.0).value
        self.tolerance = self.declare_parameter("tolerance", 0.002).value
        self.was_locked = None
        self.create_subscription(JointState, "/joint_states", self.on_state, 10)

    def on_state(self, message):
        positions = dict(zip(message.name, message.position))
        if "head_yaw_joint" not in positions or "head_pitch_joint" not in positions:
            return
        locked = (
            math.fabs(positions["head_yaw_joint"] - self.yaw_target) <= self.tolerance
            and math.fabs(positions["head_pitch_joint"] - self.pitch_target)
            <= self.tolerance
        )
        if locked != self.was_locked:
            if locked:
                self.get_logger().info(
                    "Head is locked; Eye-to-Hand samples may be collected"
                )
            else:
                self.get_logger().error(
                    "Head moved outside calibration tolerance; do not take samples"
                )
            self.was_locked = locked


def main():
    rclpy.init()
    node = HeadLockGuard()
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
