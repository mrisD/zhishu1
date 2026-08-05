#!/usr/bin/env python3
"""Non-moving health check for the live Quest TCP-to-ROS link."""

import argparse
import math
import sys
import time

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from std_msgs.msg import Bool


class QuestLinkTest(Node):
    def __init__(self):
        super().__init__("quest_link_test")
        self.connected = False
        self.pose_counts = {"left": 0, "right": 0}
        self.invalid_pose = False
        self.deadman = {"left": False, "right": False}

        self.create_subscription(Bool, "/quest/connected", self._on_connected, 10)
        for arm in ("left", "right"):
            self.create_subscription(
                PoseStamped,
                f"/quest/{arm}_controller/pose",
                lambda msg, side=arm: self._on_pose(side, msg),
                10,
            )
            self.create_subscription(
                Bool,
                f"/quest/{arm}_controller/deadman",
                lambda msg, side=arm: self._on_deadman(side, msg),
                10,
            )

    def _on_connected(self, msg: Bool) -> None:
        self.connected = msg.data

    def _on_deadman(self, arm: str, msg: Bool) -> None:
        self.deadman[arm] = msg.data

    def _on_pose(self, arm: str, msg: PoseStamped) -> None:
        values = (
            msg.pose.position.x,
            msg.pose.position.y,
            msg.pose.position.z,
            msg.pose.orientation.x,
            msg.pose.orientation.y,
            msg.pose.orientation.z,
            msg.pose.orientation.w,
        )
        norm = math.sqrt(sum(value * value for value in values[3:]))
        if (
            not all(math.isfinite(value) for value in values)
            or abs(norm - 1.0) > 0.02
            or not msg.header.frame_id
        ):
            self.invalid_pose = True
        self.pose_counts[arm] += 1


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--duration", type=float, default=5.0)
    parser.add_argument("--min-poses", type=int, default=10)
    args = parser.parse_args()

    rclpy.init()
    node = QuestLinkTest()
    deadline = time.monotonic() + args.duration
    try:
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)

        failures = []
        if not node.connected:
            failures.append("/quest/connected is false")
        if node.invalid_pose:
            failures.append("received a non-finite or non-normalized controller pose")
        for arm, count in node.pose_counts.items():
            if count < args.min_poses:
                failures.append(
                    f"{arm} controller produced only {count} poses "
                    f"(expected at least {args.min_poses})"
                )

        if failures:
            node.get_logger().error("FAIL Quest link: " + "; ".join(failures))
            return 1

        node.get_logger().info(
            "PASS Quest link: "
            f"left={node.pose_counts['left']} poses, "
            f"right={node.pose_counts['right']} poses, "
            f"deadman={node.deadman}"
        )
        return 0
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
