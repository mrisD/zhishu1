#!/usr/bin/env python3
import argparse
import time

import rclpy
from control_msgs.msg import JointJog
from moveit_msgs.srv import ServoCommandType
from rclpy.node import Node


LEFT_JOINTS = [
    "left_shoulder_pitch_joint",
    "left_shoulder_roll_joint",
    "left_shoulder_yaw_joint",
    "left_elbow_pitch_joint",
    "left_wrist_yaw_joint",
    "left_wrist_pitch_joint",
    "left_wrist_roll_joint",
]

RIGHT_JOINTS = [name.replace("left_", "right_", 1) for name in LEFT_JOINTS]


class JointSmokeTest(Node):
    def __init__(self, topic: str):
        super().__init__("servo_joint_smoke_test")
        self.publisher = self.create_publisher(JointJog, topic, 10)
        self.command_type_client = self.create_client(
            ServoCommandType, topic.replace("/delta_joint_cmds", "/switch_command_type")
        )

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

    def publish_velocity(self, joint_name: str, velocity: float):
        msg = JointJog()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.joint_names = [joint_name]
        msg.velocities = [velocity]
        self.publisher.publish(msg)


def main():
    parser = argparse.ArgumentParser(
        description="Publish a low-speed JointJog directly to MoveIt Servo"
    )
    parser.add_argument("--arm", choices=["left", "right"], default="left")
    parser.add_argument("--joint", default="elbow_pitch")
    parser.add_argument("--speed", type=float, default=0.05)
    parser.add_argument("--duration", type=float, default=1.0)
    args = parser.parse_args()

    available = LEFT_JOINTS if args.arm == "left" else RIGHT_JOINTS
    if args.joint.startswith(args.arm + "_"):
        joint_name = args.joint
    else:
        joint_name = f"{args.arm}_{args.joint}_joint"

    if joint_name not in available:
        raise SystemExit(
            f"Unsupported joint '{joint_name}'. Available: {', '.join(available)}"
        )

    rclpy.init()
    node = JointSmokeTest(f"/{args.arm}_servo/delta_joint_cmds")
    node.select_command_type(ServoCommandType.Request.JOINT_JOG)
    end_time = time.monotonic() + args.duration

    try:
        while rclpy.ok() and time.monotonic() < end_time:
            node.publish_velocity(joint_name, args.speed)
            rclpy.spin_once(node, timeout_sec=0.0)
            time.sleep(0.02)

        # Explicit stop frames.
        for _ in range(10):
            node.publish_velocity(joint_name, 0.0)
            rclpy.spin_once(node, timeout_sec=0.0)
            time.sleep(0.02)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
