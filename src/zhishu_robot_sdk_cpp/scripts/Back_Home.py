#!/usr/bin/env python3
"""Move both arms back to the unified dual-arm Home posture.

This program is the Python counterpart of the C++ ``RobotSdk::moveHome()`` API. When a unified
Home record exists (it is written by ``RobotSdk::setHomePose()``) the two saved end-effector
poses are published on ``both_arms/pose_command``, which the SDK Commander bridge executes with
the dual-arm ``moveBothPoses`` path. Without a saved record the built-in SRDF named target
``both_arms_home`` is used instead; a saved record is never mixed with the built-in posture.

No motion is sent unless ``execute_motion:=true`` is set:

    ros2 run zhishu_robot_sdk_cpp Back_Home.py --ros-args -p execute_motion:=true
"""

from __future__ import annotations

from dataclasses import dataclass
import json
import math
import os
from pathlib import Path
import sys
import time
from typing import Optional, Tuple

from geometry_msgs.msg import Pose, PoseArray
import rclpy
from rclpy.node import Node
from rclpy.publisher import Publisher
from std_msgs.msg import Empty, String
from zhishu_robot_interfaces.msg import CommanderStatus


HOME_NAMED_TARGET = 'both_arms_home'
HOME_POSE_VERSION = 1
DEFAULT_NAMESPACE = '/zhishu_robot_sdk_commander'
DEFAULT_HOME_POSE_FILE = '~/.ros/zhishu_robot_sdk_cpp/home_pose.json'
DEFAULT_TIMEOUT = 60.0
DEFAULT_DISCOVERY_TIMEOUT = 5.0
EXIT_SUCCESS = 0
EXIT_MOTION_FAILED = 1
EXIT_CONFIGURATION = 2
EXIT_INTERRUPTED = 130


@dataclass(frozen=True)
class HomeRecord:
    """One unified dual-arm Home captured as two end-effector poses."""

    frame_id: str
    left: Pose
    right: Pose
    captured_at: str = ''


def _read_pose(value: object, label: str) -> Tuple[str, Pose]:
    if not isinstance(value, dict):
        raise ValueError(f'home record requires a "{label}" object')

    frame_id = value.get('frame_id')
    if not isinstance(frame_id, str) or not frame_id.strip():
        raise ValueError(f'"{label}" home pose requires a non-empty frame_id')

    position = value.get('position')
    orientation = value.get('orientation')
    if not isinstance(position, dict) or not isinstance(orientation, dict):
        raise ValueError(f'"{label}" home pose requires position and orientation')

    def number(container: dict, key: str) -> float:
        raw = container.get(key)
        if isinstance(raw, bool) or not isinstance(raw, (int, float)):
            raise ValueError(f'"{label}" home pose {key} must be a number')
        result = float(raw)
        if not math.isfinite(result):
            raise ValueError(f'"{label}" home pose {key} must be finite')
        return result

    pose = Pose()
    pose.position.x = number(position, 'x')
    pose.position.y = number(position, 'y')
    pose.position.z = number(position, 'z')
    pose.orientation.x = number(orientation, 'x')
    pose.orientation.y = number(orientation, 'y')
    pose.orientation.z = number(orientation, 'z')
    pose.orientation.w = number(orientation, 'w')

    norm = math.hypot(
        pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w)
    if not math.isfinite(norm) or norm <= 1e-9:
        raise ValueError(f'"{label}" home pose requires a nonzero quaternion')
    return frame_id.strip(), pose


def read_home_pose(path: Path) -> HomeRecord:
    """Read the unified Home record written by ``RobotSdk::setHomePose()``."""
    try:
        document = json.loads(path.read_text(encoding='utf-8'))
    except OSError as error:
        raise ValueError(f'cannot read {path}: {error}') from error
    except json.JSONDecodeError as error:
        raise ValueError(f'{path} is not valid JSON: {error}') from error

    if not isinstance(document, dict):
        raise ValueError(f'{path} must contain a JSON object')
    if document.get('version') != HOME_POSE_VERSION:
        raise ValueError(f'{path} requires "version": {HOME_POSE_VERSION}')

    left_frame, left = _read_pose(document.get('left'), 'left')
    right_frame, right = _read_pose(document.get('right'), 'right')
    if left_frame != right_frame:
        raise ValueError(
            f'left frame {left_frame} and right frame {right_frame} differ; '
            'both_arms/pose_command requires one common frame')

    captured_at = document.get('captured_at')
    return HomeRecord(
        frame_id=left_frame,
        left=left,
        right=right,
        captured_at=captured_at if isinstance(captured_at, str) else '',
    )


def format_pose(pose: Pose) -> str:
    return (
        f'position=({pose.position.x:.4f}, {pose.position.y:.4f}, {pose.position.z:.4f}) m '
        f'quaternion=({pose.orientation.x:.4f}, {pose.orientation.y:.4f}, '
        f'{pose.orientation.z:.4f}, {pose.orientation.w:.4f})'
    )


class BackHome(Node):
    """Publisher for the SDK Commander bridge; it never plans motion by itself."""

    def __init__(self) -> None:
        super().__init__('zhishu_robot_sdk_back_home')
        self.declare_parameter('commander_namespace', DEFAULT_NAMESPACE)
        self.declare_parameter('execute_motion', False)
        self.declare_parameter('target_name', HOME_NAMED_TARGET)
        self.declare_parameter('home_pose_file', '')
        self.declare_parameter('timeout_sec', DEFAULT_TIMEOUT)
        self.declare_parameter('discovery_timeout_sec', DEFAULT_DISCOVERY_TIMEOUT)

        namespace = self._topic_namespace()
        self.pose_publisher = self.create_publisher(
            PoseArray, f'{namespace}/both_arms/pose_command', 10)
        self.named_publisher = self.create_publisher(
            String, f'{namespace}/both_arms/named_target', 10)
        self._stop_publisher = self.create_publisher(Empty, f'{namespace}/stop', 10)
        self._status: Optional[CommanderStatus] = None
        self._status_subscription = self.create_subscription(
            CommanderStatus, f'{namespace}/status_structured', self._on_status, 10)

    def _topic_namespace(self) -> str:
        configured = str(self.get_parameter('commander_namespace').value).strip().strip('/')
        return f'/{configured}' if configured else '/'

    def _on_status(self, message: CommanderStatus) -> None:
        self._status = message

    def home_pose_path(self) -> Path:
        configured = str(self.get_parameter('home_pose_file').value).strip()
        return Path(os.path.expanduser(configured or DEFAULT_HOME_POSE_FILE))

    def pose_array(self, record: HomeRecord) -> PoseArray:
        message = PoseArray()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = record.frame_id
        message.poses = [record.left, record.right]
        return message

    def clear_status(self) -> None:
        self._status = None

    def wait_for_subscriber(self, publisher: Publisher, timeout: float) -> bool:
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if publisher.get_subscription_count() > 0:
                return True
            rclpy.spin_once(self, timeout_sec=0.1)
        return publisher.get_subscription_count() > 0

    def wait_for_status(self, command: str, timeout: float) -> Optional[CommanderStatus]:
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            message = self._status
            if message is not None and message.command == command:
                self._status = None
                return message
        return None

    def stop(self) -> None:
        # Give the stop sample a short window to leave DDS before the process exits.
        self._stop_publisher.publish(Empty())
        deadline = time.monotonic() + 0.5
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)


def back_home(node: BackHome) -> int:
    namespace = str(node.get_parameter('commander_namespace').value).strip(' /')
    if not namespace:
        node.get_logger().error('commander_namespace must not be empty')
        return EXIT_CONFIGURATION

    target_name = str(node.get_parameter('target_name').value).strip()
    if not target_name:
        node.get_logger().error('target_name must not be empty')
        return EXIT_CONFIGURATION

    timeout = float(node.get_parameter('timeout_sec').value)
    discovery_timeout = float(node.get_parameter('discovery_timeout_sec').value)
    if timeout <= 0.0 or discovery_timeout <= 0.0:
        node.get_logger().error('timeout_sec and discovery_timeout_sec must be positive')
        return EXIT_CONFIGURATION

    path = node.home_pose_path()
    try:
        saved_exists = path.exists()
    except OSError as error:
        node.get_logger().error(f'Cannot inspect the Home record {path}: {error}')
        return EXIT_CONFIGURATION

    record: Optional[HomeRecord] = None
    if saved_exists:
        try:
            record = read_home_pose(path)
        except ValueError as error:
            node.get_logger().error(f'Saved Home record is unusable: {error}')
            return EXIT_CONFIGURATION
        node.get_logger().info(
            f'Unified Home record {path} (captured {record.captured_at or "unknown"})')
    else:
        node.get_logger().info(
            f'No unified Home record at {path}; the built-in {target_name} posture is used')

    if record is None:
        command = 'both_arms/named_target'
        description = f'the built-in {target_name} named target'
        publisher = node.named_publisher
    else:
        command = 'both_arms/pose_command'
        description = f'two saved {record.frame_id} end-effector poses'
        publisher = node.pose_publisher

    if not bool(node.get_parameter('execute_motion').value):
        node.get_logger().warn(
            f'No motion sent: both arms would move back to {description} through {command}. '
            'Set execute_motion:=true after a safety check.')
        return EXIT_SUCCESS

    if record is not None:
        node.get_logger().info(f'  left target: {format_pose(record.left)}')
        node.get_logger().info(f'  right target: {format_pose(record.right)}')

    if not node.wait_for_subscriber(publisher, discovery_timeout):
        node.get_logger().error(
            f'No SDK Commander subscriber on {publisher.topic_name}; '
            'start the SDK Commander bridge first')
        return EXIT_CONFIGURATION

    node.clear_status()
    if record is None:
        node.named_publisher.publish(String(data=target_name))
    else:
        node.pose_publisher.publish(node.pose_array(record))
    node.get_logger().info(f'Published {command}; waiting up to {timeout:.1f} s for the result')

    status = node.wait_for_status(command, timeout)
    if status is None:
        node.stop()
        node.get_logger().error(
            f'No Commander feedback for {command} within {timeout:.1f} s; '
            'a MoveIt stop request was sent')
        return EXIT_MOTION_FAILED
    if not status.success:
        node.get_logger().error(f'Back Home failed [{status.error_name}] {status.message}')
        return EXIT_MOTION_FAILED

    node.get_logger().info(f'Both arms reached Home: {status.message or "success"}')
    return EXIT_SUCCESS


def main(args=None) -> int:
    rclpy.init(args=args)
    node = BackHome()
    try:
        return back_home(node)
    except KeyboardInterrupt:
        node.stop()
        node.get_logger().warn('Interrupted; a MoveIt stop request was sent')
        return EXIT_INTERRUPTED
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main())
