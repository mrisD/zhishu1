#!/usr/bin/env python3
import argparse
import time

import rclpy
from geometry_msgs.msg import TwistStamped
from moveit_msgs.srv import ServoCommandType
from rclpy.node import Node


class SmokeTest(Node):
    def __init__(self, topic: str, frame: str):
        super().__init__("servo_smoke_test")
        self.publisher = self.create_publisher(TwistStamped, topic, 10)
        self.command_type_client = self.create_client(
            ServoCommandType, topic.replace("/delta_twist_cmds", "/switch_command_type")
        )
        self.frame = frame

    def select_command_type(self, command_type: int, timeout_sec: float = 5.0) -> None:
        if not self.command_type_client.wait_for_service(timeout_sec=timeout_sec):
            raise RuntimeError("MoveIt Servo switch_command_type service is unavailable")
        request = ServoCommandType.Request()
        request.command_type = command_type
        future = self.command_type_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout_sec)
        response = future.result()
        if response is None or not response.success:
            raise RuntimeError("MoveIt Servo rejected the requested command type")

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
    node.select_command_type(ServoCommandType.Request.TWIST)
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
