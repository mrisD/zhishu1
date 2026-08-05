#!/usr/bin/env python3
"""End-to-end MoveIt Servo test intended for fake/mock ros2_control hardware."""

import argparse
import math
import socket
import struct
import sys
import time
from typing import Callable, Dict, Iterable

import rclpy
from control_msgs.msg import JointJog
from geometry_msgs.msg import PoseStamped, TwistStamped
from moveit_msgs.msg import ServoStatus
from moveit_msgs.srv import ServoCommandType
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.parameter_client import AsyncParameterClient
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool


ARM_JOINTS = {
    arm: [
        f"{arm}_shoulder_pitch_joint",
        f"{arm}_shoulder_roll_joint",
        f"{arm}_shoulder_yaw_joint",
        f"{arm}_elbow_pitch_joint",
        f"{arm}_wrist_yaw_joint",
        f"{arm}_wrist_pitch_joint",
        f"{arm}_wrist_roll_joint",
    ]
    for arm in ("left", "right")
}


class MockServoIntegrationTest(Node):
    def __init__(self, arm: str, timeout: float, tcp_port: int):
        super().__init__("mock_servo_integration_test")
        self.arm = arm
        self.timeout = timeout
        self.tcp_port = tcp_port
        self.joint_positions: Dict[str, float] = {}
        self.last_status = None
        self.quest_connected_seen = False
        self.quest_pose_counts = {"left": 0, "right": 0}
        self.quest_deadman_seen = False

        servo_ns = f"/{arm}_servo"
        self.joint_pub = self.create_publisher(
            JointJog, f"{servo_ns}/delta_joint_cmds", 10
        )
        self.twist_pub = self.create_publisher(
            TwistStamped, f"{servo_ns}/delta_twist_cmds", 10
        )
        self.switch_client = self.create_client(
            ServoCommandType, f"{servo_ns}/switch_command_type"
        )
        self.parameter_client = AsyncParameterClient(self, servo_ns)
        self.create_subscription(JointState, "/joint_states", self._on_joint_state, 10)
        self.create_subscription(
            ServoStatus, f"{servo_ns}/status", self._on_status, 10
        )
        self.create_subscription(Bool, "/quest/connected", self._on_connected, 10)
        for side in ("left", "right"):
            self.create_subscription(
                PoseStamped,
                f"/quest/{side}_controller/pose",
                lambda _msg, hand=side: self._on_quest_pose(hand),
                10,
            )
        self.create_subscription(
            Bool, f"/quest/{arm}_controller/deadman", self._on_deadman, 10
        )

    def _on_joint_state(self, msg: JointState) -> None:
        for name, position in zip(msg.name, msg.position):
            if math.isfinite(position):
                self.joint_positions[name] = position

    def _on_status(self, msg: ServoStatus) -> None:
        self.last_status = msg

    def _on_connected(self, msg: Bool) -> None:
        self.quest_connected_seen = self.quest_connected_seen or msg.data

    def _on_quest_pose(self, hand: str) -> None:
        self.quest_pose_counts[hand] += 1

    def _on_deadman(self, msg: Bool) -> None:
        self.quest_deadman_seen = self.quest_deadman_seen or msg.data

    def spin_until(self, predicate: Callable[[], bool], description: str) -> None:
        deadline = time.monotonic() + self.timeout
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
            if predicate():
                return
        raise RuntimeError(f"Timed out waiting for {description}")

    def spin_for(self, duration: float, publish: Callable[[], None]) -> None:
        deadline = time.monotonic() + duration
        while rclpy.ok() and time.monotonic() < deadline:
            publish()
            rclpy.spin_once(self, timeout_sec=0.0)
            time.sleep(0.02)

    def set_collision_checking(self, enabled: bool) -> None:
        if not self.parameter_client.wait_for_services(timeout_sec=self.timeout):
            raise RuntimeError("Servo parameter service is unavailable")
        future = self.parameter_client.set_parameters(
            [
                Parameter(
                    "moveit_servo.check_collisions",
                    Parameter.Type.BOOL,
                    enabled,
                )
            ]
        )
        rclpy.spin_until_future_complete(self, future, timeout_sec=self.timeout)
        response = future.result()
        results = response.results if response is not None else []
        if not results or not all(result.successful for result in results):
            reasons = [result.reason for result in results or []]
            raise RuntimeError(f"Could not change collision checking: {reasons}")

    def switch_command_type(self, command_type: int) -> None:
        if not self.switch_client.wait_for_service(timeout_sec=self.timeout):
            raise RuntimeError("Servo switch_command_type service is unavailable")
        request = ServoCommandType.Request()
        request.command_type = command_type
        future = self.switch_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=self.timeout)
        response = future.result()
        if response is None or not response.success:
            raise RuntimeError(f"Servo rejected command type {command_type}")

    def snapshot(self, names: Iterable[str]) -> Dict[str, float]:
        return {name: self.joint_positions[name] for name in names}

    @staticmethod
    def max_delta(before: Dict[str, float], after: Dict[str, float]) -> float:
        return max(abs(after[name] - before[name]) for name in before)

    def publish_joints(
        self, joint_names: Iterable[str], velocities: Iterable[float]
    ) -> None:
        msg = JointJog()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.joint_names = list(joint_names)
        msg.velocities = list(velocities)
        self.joint_pub.publish(msg)

    def publish_joint(self, joint_name: str, velocity: float) -> None:
        self.publish_joints([joint_name], [velocity])

    def publish_twist(self, speed: float) -> None:
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "base_link"
        msg.twist.linear.x = speed
        self.twist_pub.publish(msg)

    def halt_joint(self, joint_name: str) -> None:
        self.spin_for(0.25, lambda: self.publish_joint(joint_name, 0.0))

    def halt_twist(self) -> None:
        self.spin_for(0.25, lambda: self.publish_twist(0.0))

    @staticmethod
    def quest_hand(z: float, grip: float) -> bytes:
        return struct.pack(
            "<9fI",
            0.0,
            1.0,
            z,
            0.0,
            0.0,
            0.0,
            1.0,
            grip,
            0.0,
            1,
        )

    def test_quest_pipeline(self, joints: Iterable[str]) -> None:
        before = self.snapshot(joints)
        with socket.create_connection(
            ("127.0.0.1", self.tcp_port), timeout=self.timeout
        ) as client:
            for sequence in range(100):
                packet = struct.pack(
                    "<IIIId", 0x31525651, 1, sequence, 0, sequence * 0.02
                )
                packet += self.quest_hand(sequence * 0.001, 1.0)
                packet += self.quest_hand(0.0, 0.0)
                client.sendall(packet)
                rclpy.spin_once(self, timeout_sec=0.0)
                time.sleep(0.02)

            # Release the deadman and send several frames so the pipeline emits
            # a final zero command before the TCP connection closes.
            for sequence in range(100, 110):
                packet = struct.pack(
                    "<IIIId", 0x31525651, 1, sequence, 0, sequence * 0.02
                )
                packet += self.quest_hand(0.1, 0.0)
                packet += self.quest_hand(0.0, 0.0)
                client.sendall(packet)
                rclpy.spin_once(self, timeout_sec=0.0)
                time.sleep(0.02)

        self.spin_for(0.5, lambda: None)
        after = self.snapshot(joints)
        delta = self.max_delta(before, after)
        failures = []
        if not self.quest_connected_seen:
            failures.append("Quest TCP connection was not observed")
        if min(self.quest_pose_counts.values()) < 10:
            failures.append(f"pose counts were {self.quest_pose_counts}")
        if not self.quest_deadman_seen:
            failures.append("left deadman was never true")
        if delta < 0.001:
            failures.append(f"mock joint delta was only {delta:.6f} rad")
        if failures:
            raise RuntimeError("Quest-to-mock pipeline failed: " + "; ".join(failures))
        self.get_logger().info(
            "PASS Quest TCP -> pose/deadman -> Servo -> mock joints: "
            f"poses={self.quest_pose_counts}, max joint delta={delta:.6f} rad"
        )

    def run(self) -> None:
        joints = ARM_JOINTS[self.arm]
        elbow = f"{self.arm}_elbow_pitch_joint"
        self.spin_until(
            lambda: all(name in self.joint_positions for name in joints),
            "mock joint states",
        )

        # The default all-zero mock pose can be close to collision. Disable only
        # for this isolated fake-hardware test and always restore it in finally.
        self.set_collision_checking(False)
        try:
            self.switch_command_type(ServoCommandType.Request.JOINT_JOG)
            joint_before = self.snapshot(joints)
            # Fake ros2_control starts every joint at zero, which is a kinematic
            # singularity for this arm. Exercise JointJog while moving the mock
            # arm to a bent, non-singular pose before testing Cartesian Servo.
            safe_joints = joints
            side = 1.0 if self.arm == "left" else -1.0
            # Directions approximate the known-good bent pose documented in
            # zhishu_robot_moveit_config/config/initial_positions.yaml.
            safe_velocities = [
                -1.0 * side,
                -0.15 * side,
                1.0 * side,
                0.60,
                0.30 * side,
                0.40,
                0.30 * side,
            ]
            self.spin_for(
                16.0, lambda: self.publish_joints(safe_joints, safe_velocities)
            )
            self.spin_for(
                0.25,
                lambda: self.publish_joints(
                    safe_joints, [0.0] * len(safe_joints)
                ),
            )
            joint_after = self.snapshot(joints)
            joint_delta = self.max_delta(joint_before, joint_after)
            if joint_delta < 0.02:
                raise RuntimeError(
                    f"Joint Servo motion too small: max delta={joint_delta:.6f} rad"
                )
            self.get_logger().info(
                f"PASS joint Servo: max delta={joint_delta:.6f} rad"
            )

            self.switch_command_type(ServoCommandType.Request.TWIST)
            twist_before = self.snapshot(joints)
            self.spin_for(1.0, lambda: self.publish_twist(0.03))
            self.halt_twist()
            twist_after = self.snapshot(joints)
            twist_delta = self.max_delta(twist_before, twist_after)
            if twist_delta < 0.001:
                status = (
                    f"{self.last_status.code}: {self.last_status.message}"
                    if self.last_status is not None
                    else "unavailable"
                )
                raise RuntimeError(
                    "Cartesian Servo produced no mock joint motion: "
                    f"max delta={twist_delta:.6f} rad, status={status}"
                )
            self.get_logger().info(
                f"PASS Cartesian Servo: max joint delta={twist_delta:.6f} rad"
            )
            self.test_quest_pipeline(joints)
        finally:
            self.halt_twist()
            self.set_collision_checking(True)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--arm", choices=["left", "right"], default="left")
    parser.add_argument("--timeout", type=float, default=30.0)
    parser.add_argument("--tcp-port", type=int, default=5006)
    # launch_ros appends ROS-specific arguments to every Node process.
    args, _ = parser.parse_known_args()

    rclpy.init()
    node = MockServoIntegrationTest(args.arm, args.timeout, args.tcp_port)
    try:
        node.run()
        node.get_logger().info("PASS mock MoveIt Servo integration test")
        return 0
    except Exception as exc:  # noqa: BLE001 - test runner must report all failures
        node.get_logger().error(f"FAIL mock MoveIt Servo integration test: {exc}")
        return 1
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
