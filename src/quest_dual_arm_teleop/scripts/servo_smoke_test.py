#!/usr/bin/env python3
import argparse
import time

import rclpy
from geometry_msgs.msg import TwistStamped
from rclpy.node import Node


class SmokeTest(Node):
    def __init__(self, topic: str, frame: str):
        super().__init__("servo_smoke_test")
        self.publisher = self.create_publisher(TwistStamped, topic, 10)
        self.frame = frame

    def publish(self, axis: str, speed: float):
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame
        target = msg.twist.linear if axis.startswith("linear_") else msg.twist.angular
        component = axis.split("_", 1)[1]
        setattr(target, component, speed)
        self.publisher.publish(msg)


def main():
    parser = argparse.ArgumentParser(description="Publish a low-speed MoveIt Servo test command")
    parser.add_argument("--arm", choices=["left", "right"], default="left")
    parser.add_argument(
        "--axis",
        choices=["linear_x", "linear_y", "linear_z", "angular_x", "angular_y", "angular_z"],
        default="linear_z",
    )
    parser.add_argument("--speed", type=float, default=0.01)
    parser.add_argument("--duration", type=float, default=1.0)
    parser.add_argument("--frame", default="base_link")
    args = parser.parse_args()

    rclpy.init()
    topic = f"/{args.arm}_servo/delta_twist_cmds"
    node = SmokeTest(topic, args.frame)
    end_time = time.monotonic() + args.duration
    try:
        while rclpy.ok() and time.monotonic() < end_time:
            node.publish(args.axis, args.speed)
            rclpy.spin_once(node, timeout_sec=0.0)
            time.sleep(0.02)
        for _ in range(10):
            node.publish(args.axis, 0.0)
            rclpy.spin_once(node, timeout_sec=0.0)
            time.sleep(0.02)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
