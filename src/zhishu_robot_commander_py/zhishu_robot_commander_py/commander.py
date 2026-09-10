"""A small synchronous Python client for the C++ Commander ROS bridge."""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum, IntEnum
import math
import threading
import time
from typing import Iterable, Optional, Union
import uuid

from zhishu_robot_interfaces.msg import CommanderStatus
from geometry_msgs.msg import PoseArray, PoseStamped
import rclpy
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import Empty, Float64, Float64MultiArray, String


DEFAULT_NAMESPACE = '/zhishu_robot_commander'
DEFAULT_TIMEOUT = 60.0
DEFAULT_DISCOVERY_TIMEOUT = 5.0
GRIPPER_MINIMUM = 0.0
GRIPPER_MAXIMUM = 0.06


class ErrorCode(IntEnum):
    """Error codes shared with ``zhishu_robot_commander_cpp::ErrorCode``."""

    SUCCESS = 0
    INVALID_ARGUMENT = 1
    TARGET_REJECTED = 2
    PLANNING_FAILED = 3
    EXECUTION_FAILED = 4
    BUSY = 5
    CANCELLED = 6
    TIMEOUT = 7
    NOT_READY = 8
    INTERNAL_ERROR = 9


class ArmGroup(str, Enum):
    LEFT = 'left'
    RIGHT = 'right'
    BOTH = 'both'


class GripperGroup(str, Enum):
    LEFT = 'left'
    RIGHT = 'right'
    BOTH = 'both'


@dataclass(frozen=True)
class Result:
    """Result returned by every customer command."""

    success: bool
    code: ErrorCode
    message: str = ''
    native_code: int = 0
    command: str = ''
    error_name: str = ''

    def __bool__(self) -> bool:
        return self.success

    def raise_for_error(self) -> 'Result':
        """Raise :class:`CommanderError` when the command failed."""
        if not self.success:
            raise CommanderError(self)
        return self


class CommanderError(RuntimeError):
    """Exception raised by :meth:`Result.raise_for_error`."""

    def __init__(self, result: Result) -> None:
        self.result = result
        detail = result.message or result.error_name or result.code.name
        label = result.command or 'commander command'
        super().__init__(f'{label}: {detail}')


@dataclass(frozen=True)
class PoseTarget:
    """ROS-independent Cartesian pose accepted by the customer API."""

    x: float
    y: float
    z: float
    qx: float = 0.0
    qy: float = 0.0
    qz: float = 0.0
    qw: float = 1.0
    frame_id: str = 'base_link'

    def to_message(self) -> PoseStamped:
        values = (self.x, self.y, self.z, self.qx, self.qy, self.qz, self.qw)
        if not all(math.isfinite(float(value)) for value in values):
            raise ValueError('pose values must be finite')
        if not self.frame_id.strip():
            raise ValueError('frame_id must not be empty')

        quaternion_norm = math.sqrt(
            self.qx * self.qx
            + self.qy * self.qy
            + self.qz * self.qz
            + self.qw * self.qw
        )
        if quaternion_norm <= 1e-12:
            raise ValueError('pose quaternion must not be zero')

        message = PoseStamped()
        message.header.frame_id = self.frame_id
        message.pose.position.x = float(self.x)
        message.pose.position.y = float(self.y)
        message.pose.position.z = float(self.z)
        message.pose.orientation.x = float(self.qx) / quaternion_norm
        message.pose.orientation.y = float(self.qy) / quaternion_norm
        message.pose.orientation.z = float(self.qz) / quaternion_norm
        message.pose.orientation.w = float(self.qw) / quaternion_norm
        return message


@dataclass
class _PendingCommand:
    expected_status: str
    result: Optional[Result] = None
    timed_out: bool = False


PoseLike = Union[PoseTarget, PoseStamped]


class Commander:
    """
    Customer-facing client for a running C++ Commander bridge.

    The class owns an isolated ROS context and a background executor. Public motion
    methods block until the C++ bridge publishes a structured result or the local
    timeout expires. One instance intentionally permits only one in-flight motion.
    """

    def __init__(
        self,
        namespace: str = DEFAULT_NAMESPACE,
        *,
        default_timeout: float = DEFAULT_TIMEOUT,
        discovery_timeout: float = DEFAULT_DISCOVERY_TIMEOUT,
        node_name: Optional[str] = None,
    ) -> None:
        self._namespace = self._normalize_namespace(namespace)
        self._default_timeout = self._positive_timeout(
            default_timeout, 'default_timeout'
        )
        self._discovery_timeout = self._positive_timeout(
            discovery_timeout, 'discovery_timeout'
        )
        self._condition = threading.Condition()
        self._pending: Optional[_PendingCommand] = None
        self._closed = False
        self._last_late_result: Optional[Result] = None

        self._context = Context()
        rclpy.init(args=None, context=self._context)
        resolved_name = node_name or f'customer_commander_{uuid.uuid4().hex[:8]}'
        self._node = Node(resolved_name, context=self._context)

        self._publishers = {
            'left_arm/named_target': self._publisher(String, 'left_arm/named_target'),
            'right_arm/named_target': self._publisher(String, 'right_arm/named_target'),
            'both_arms/named_target': self._publisher(String, 'both_arms/named_target'),
            'left_arm/joint_command': self._publisher(
                Float64MultiArray, 'left_arm/joint_command'
            ),
            'right_arm/joint_command': self._publisher(
                Float64MultiArray, 'right_arm/joint_command'
            ),
            'both_arms/joint_command': self._publisher(
                Float64MultiArray, 'both_arms/joint_command'
            ),
            'left_arm/pose_command': self._publisher(
                PoseStamped, 'left_arm/pose_command'
            ),
            'right_arm/pose_command': self._publisher(
                PoseStamped, 'right_arm/pose_command'
            ),
            'both_arms/pose_command': self._publisher(
                PoseArray, 'both_arms/pose_command'
            ),
            'left_arm/cartesian_pose_command': self._publisher(
                PoseStamped, 'left_arm/cartesian_pose_command'
            ),
            'right_arm/cartesian_pose_command': self._publisher(
                PoseStamped, 'right_arm/cartesian_pose_command'
            ),
            'left_arm/tcp_pose_command': self._publisher(
                PoseStamped, 'left_arm/tcp_pose_command'
            ),
            'right_arm/tcp_pose_command': self._publisher(
                PoseStamped, 'right_arm/tcp_pose_command'
            ),
            'both_arms/tcp_pose_command': self._publisher(
                PoseArray, 'both_arms/tcp_pose_command'
            ),
            'left_arm/cartesian_tcp_pose_command': self._publisher(
                PoseStamped, 'left_arm/cartesian_tcp_pose_command'
            ),
            'right_arm/cartesian_tcp_pose_command': self._publisher(
                PoseStamped, 'right_arm/cartesian_tcp_pose_command'
            ),
            'left_gripper/command': self._publisher(Float64, 'left_gripper/command'),
            'right_gripper/command': self._publisher(
                Float64, 'right_gripper/command'
            ),
            'both_grippers/command': self._publisher(
                Float64MultiArray, 'both_grippers/command'
            ),
            'grippers/named_command': self._publisher(
                String, 'grippers/named_command'
            ),
            'stop': self._publisher(Empty, 'stop'),
        }
        self._status_subscription = self._node.create_subscription(
            CommanderStatus,
            self._topic('status_structured'),
            self._status_callback,
            10,
        )

        self._executor = SingleThreadedExecutor(context=self._context)
        self._executor.add_node(self._node)
        self._spin_thread = threading.Thread(
            target=self._executor.spin,
            name=f'{resolved_name}_executor',
            daemon=True,
        )
        self._spin_thread.start()

    @property
    def busy(self) -> bool:
        """Whether this client still has an in-flight command."""
        with self._condition:
            return self._pending is not None

    @property
    def last_late_result(self) -> Optional[Result]:
        """Return the eventual result for the most recent timed-out command."""
        with self._condition:
            return self._last_late_result

    def wait_until_ready(self, timeout: Optional[float] = None) -> bool:
        """Wait until the C++ bridge is discoverable on commands and status."""
        timeout_value = self._timeout_or_default(timeout, self._discovery_timeout)
        deadline = time.monotonic() + timeout_value
        representative = self._publishers['left_arm/named_target']
        while time.monotonic() < deadline:
            if self._closed:
                return False
            if (
                representative.get_subscription_count() > 0
                and self._status_subscription.get_publisher_count() > 0
            ):
                return True
            time.sleep(0.05)
        return False

    def move_named(
        self,
        group: Union[ArmGroup, str],
        target_name: str,
        *,
        timeout: Optional[float] = None,
    ) -> Result:
        arm = self._coerce_arm(group)
        if arm is None:
            return self._invalid('group must be left, right, or both')
        if not isinstance(target_name, str) or not target_name.strip():
            return self._invalid('target_name must not be empty')
        command = f'{self._arm_prefix(arm)}/named_target'
        message = String(data=target_name.strip())
        return self._send(command, message, timeout=timeout)

    def move_joints(
        self,
        group: Union[ArmGroup, str],
        joints: Iterable[float],
        *,
        timeout: Optional[float] = None,
    ) -> Result:
        arm = self._coerce_arm(group)
        if arm is None:
            return self._invalid('group must be left, right, or both')
        expected_count = 14 if arm is ArmGroup.BOTH else 7
        try:
            values = self._validated_values(joints, expected_count, 'joints')
        except (TypeError, ValueError) as exc:
            return self._invalid(str(exc))
        command = f'{self._arm_prefix(arm)}/joint_command'
        return self._send(
            command, Float64MultiArray(data=values), timeout=timeout
        )

    def move_pose(
        self,
        group: Union[ArmGroup, str],
        target: PoseLike,
        *,
        cartesian: bool = False,
        timeout: Optional[float] = None,
    ) -> Result:
        arm = self._coerce_arm(group)
        if arm not in (ArmGroup.LEFT, ArmGroup.RIGHT):
            return self._invalid('move_pose group must be left or right')
        try:
            message = self._pose_message(target)
        except (TypeError, ValueError) as exc:
            return self._invalid(str(exc))
        suffix = 'cartesian_pose_command' if cartesian else 'pose_command'
        command = f'{self._arm_prefix(arm)}/{suffix}'
        return self._send(command, message, timeout=timeout)

    def move_both_poses(
        self,
        left_target: PoseLike,
        right_target: PoseLike,
        *,
        timeout: Optional[float] = None,
    ) -> Result:
        return self._move_both_pose_messages(
            'both_arms/pose_command', left_target, right_target, timeout
        )

    def move_tcp_pose(
        self,
        group: Union[ArmGroup, str],
        target: PoseLike,
        *,
        cartesian: bool = False,
        timeout: Optional[float] = None,
    ) -> Result:
        arm = self._coerce_arm(group)
        if arm not in (ArmGroup.LEFT, ArmGroup.RIGHT):
            return self._invalid('move_tcp_pose group must be left or right')
        try:
            message = self._pose_message(target)
        except (TypeError, ValueError) as exc:
            return self._invalid(str(exc))
        suffix = 'cartesian_tcp_pose_command' if cartesian else 'tcp_pose_command'
        command = f'{self._arm_prefix(arm)}/{suffix}'
        return self._send(command, message, timeout=timeout)

    def move_both_tcp_poses(
        self,
        left_target: PoseLike,
        right_target: PoseLike,
        *,
        timeout: Optional[float] = None,
    ) -> Result:
        return self._move_both_pose_messages(
            'both_arms/tcp_pose_command', left_target, right_target, timeout
        )

    def move_gripper(
        self,
        group: Union[GripperGroup, str],
        position: float,
        *,
        timeout: Optional[float] = None,
    ) -> Result:
        gripper = self._coerce_gripper(group)
        if gripper is None:
            return self._invalid('group must be left, right, or both')
        try:
            value = float(position)
        except (TypeError, ValueError):
            return self._invalid('gripper position must be a number')
        if not math.isfinite(value) or not GRIPPER_MINIMUM <= value <= GRIPPER_MAXIMUM:
            return self._invalid(
                f'gripper position must be within [{GRIPPER_MINIMUM}, {GRIPPER_MAXIMUM}] m'
            )
        if gripper is GripperGroup.BOTH:
            return self.move_both_grippers(value, value, timeout=timeout)
        command = f'{gripper.value}_gripper/command'
        return self._send(command, Float64(data=value), timeout=timeout)

    def move_both_grippers(
        self,
        left_position: float,
        right_position: float,
        *,
        timeout: Optional[float] = None,
    ) -> Result:
        try:
            values = [float(left_position), float(right_position)]
        except (TypeError, ValueError):
            return self._invalid('gripper positions must be numbers')
        if any(
            not math.isfinite(value)
            or not GRIPPER_MINIMUM <= value <= GRIPPER_MAXIMUM
            for value in values
        ):
            return self._invalid(
                f'gripper positions must be within [{GRIPPER_MINIMUM}, {GRIPPER_MAXIMUM}] m'
            )
        return self._send(
            'both_grippers/command',
            Float64MultiArray(data=values),
            timeout=timeout,
        )

    def open_gripper(
        self,
        group: Union[GripperGroup, str],
        *,
        timeout: Optional[float] = None,
    ) -> Result:
        return self._named_gripper_command('open', group, timeout)

    def close_gripper(
        self,
        group: Union[GripperGroup, str],
        *,
        timeout: Optional[float] = None,
    ) -> Result:
        return self._named_gripper_command('close', group, timeout)

    def stop(self) -> Result:
        """Request a MoveIt software stop without waiting for another motion."""
        if self._closed:
            return self._not_ready('Commander is closed', 'stop')
        publisher = self._publishers['stop']
        if publisher.get_subscription_count() == 0:
            return self._not_ready('Commander bridge is not available', 'stop')
        publisher.publish(Empty())
        return Result(
            True,
            ErrorCode.SUCCESS,
            'MoveIt stop request published',
            command='stop',
            error_name='SUCCESS',
        )

    def close(self) -> None:
        """Release the private ROS context and executor."""
        if self._closed:
            return
        self._closed = True
        with self._condition:
            self._condition.notify_all()
        self._executor.shutdown(timeout_sec=2.0)
        self._spin_thread.join(timeout=2.0)
        self._node.destroy_node()
        if self._context.ok():
            self._context.shutdown()

    def __enter__(self) -> 'Commander':
        return self

    def __exit__(self, exc_type, exc_value, traceback) -> None:
        del exc_type, exc_value, traceback
        self.close()

    def _move_both_pose_messages(
        self,
        command: str,
        left_target: PoseLike,
        right_target: PoseLike,
        timeout: Optional[float],
    ) -> Result:
        try:
            left = self._pose_message(left_target)
            right = self._pose_message(right_target)
        except (TypeError, ValueError) as exc:
            return self._invalid(str(exc))
        if left.header.frame_id != right.header.frame_id:
            return self._invalid('dual-arm pose targets must use the same frame_id')
        message = PoseArray()
        message.header = left.header
        message.poses = [left.pose, right.pose]
        return self._send(command, message, timeout=timeout)

    def _named_gripper_command(
        self,
        action: str,
        group: Union[GripperGroup, str],
        timeout: Optional[float],
    ) -> Result:
        gripper = self._coerce_gripper(group)
        if gripper is None:
            return self._invalid('group must be left, right, or both')
        named_command = f'{action}_{gripper.value}'
        expected_status = f'grippers/named_command:{named_command}'
        return self._send(
            'grippers/named_command',
            String(data=named_command),
            timeout=timeout,
            expected_status=expected_status,
        )

    def _send(
        self,
        command: str,
        message,
        *,
        timeout: Optional[float],
        expected_status: Optional[str] = None,
    ) -> Result:
        if self._closed:
            return self._not_ready('Commander is closed', command)
        try:
            timeout_value = self._timeout_or_default(timeout, self._default_timeout)
        except ValueError as exc:
            return self._invalid(str(exc), command)

        publisher = self._publishers[command]
        if not self._wait_for_publisher(publisher, self._discovery_timeout):
            return self._not_ready('Commander bridge is not available', command)
        if self._status_subscription.get_publisher_count() == 0:
            return self._not_ready('Commander status publisher is not available', command)

        pending = _PendingCommand(expected_status or command)
        with self._condition:
            if self._pending is not None:
                return Result(
                    False,
                    ErrorCode.BUSY,
                    'another command is still in progress',
                    command=command,
                    error_name='BUSY',
                )
            self._pending = pending

        try:
            publisher.publish(message)
        except Exception as exc:  # rclpy raises several context-specific types.
            with self._condition:
                if self._pending is pending:
                    self._pending = None
            return Result(
                False,
                ErrorCode.INTERNAL_ERROR,
                f'failed to publish command: {exc}',
                command=command,
                error_name='INTERNAL_ERROR',
            )

        deadline = time.monotonic() + timeout_value
        with self._condition:
            while pending.result is None and not self._closed:
                remaining = deadline - time.monotonic()
                if remaining <= 0.0:
                    pending.timed_out = True
                    return Result(
                        False,
                        ErrorCode.TIMEOUT,
                        (
                            f'no result received within {timeout_value:.3f} s; '
                            'the command may still be running'
                        ),
                        command=command,
                        error_name='TIMEOUT',
                    )
                self._condition.wait(remaining)

            if pending.result is not None:
                if self._pending is pending:
                    self._pending = None
                return pending.result

            if self._pending is pending:
                self._pending = None
            return self._not_ready('Commander closed while waiting for result', command)

    def _status_callback(self, message: CommanderStatus) -> None:
        with self._condition:
            pending = self._pending
            if pending is None or message.command != pending.expected_status:
                return
            try:
                code = ErrorCode(message.error_code)
            except ValueError:
                code = ErrorCode.INTERNAL_ERROR
            result = Result(
                bool(message.success),
                code,
                message.message,
                int(message.native_code),
                message.command,
                message.error_name,
            )
            if pending.timed_out:
                self._last_late_result = result
                self._pending = None
            else:
                pending.result = result
            self._condition.notify_all()

    def _publisher(self, message_type, suffix: str):
        return self._node.create_publisher(message_type, self._topic(suffix), 10)

    def _topic(self, suffix: str) -> str:
        return f'{self._namespace}/{suffix}'

    def _wait_for_publisher(self, publisher, timeout: float) -> bool:
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if self._closed:
                return False
            if publisher.get_subscription_count() > 0:
                return True
            time.sleep(0.05)
        return publisher.get_subscription_count() > 0

    @staticmethod
    def _normalize_namespace(namespace: str) -> str:
        if not isinstance(namespace, str) or not namespace.strip(' /'):
            raise ValueError('namespace must not be empty')
        return '/' + namespace.strip(' /')

    @staticmethod
    def _positive_timeout(value: float, label: str) -> float:
        result = float(value)
        if not math.isfinite(result) or result <= 0.0:
            raise ValueError(f'{label} must be a positive finite number')
        return result

    @classmethod
    def _timeout_or_default(cls, value: Optional[float], default: float) -> float:
        return default if value is None else cls._positive_timeout(value, 'timeout')

    @staticmethod
    def _coerce_arm(group: Union[ArmGroup, str]) -> Optional[ArmGroup]:
        try:
            return group if isinstance(group, ArmGroup) else ArmGroup(group)
        except (TypeError, ValueError):
            return None

    @staticmethod
    def _coerce_gripper(
        group: Union[GripperGroup, str]
    ) -> Optional[GripperGroup]:
        try:
            return group if isinstance(group, GripperGroup) else GripperGroup(group)
        except (TypeError, ValueError):
            return None

    @staticmethod
    def _arm_prefix(group: ArmGroup) -> str:
        return 'both_arms' if group is ArmGroup.BOTH else f'{group.value}_arm'

    @staticmethod
    def _validated_values(
        values: Iterable[float], expected_count: int, label: str
    ) -> list[float]:
        try:
            converted = [float(value) for value in values]
        except TypeError as exc:
            raise TypeError(f'{label} must be an iterable of numbers') from exc
        except ValueError as exc:
            raise ValueError(f'{label} must contain only numbers') from exc
        if len(converted) != expected_count:
            raise ValueError(f'{label} requires exactly {expected_count} values')
        if not all(math.isfinite(value) for value in converted):
            raise ValueError(f'{label} values must be finite')
        return converted

    @staticmethod
    def _pose_message(target: PoseLike) -> PoseStamped:
        if isinstance(target, PoseTarget):
            return target.to_message()
        if not isinstance(target, PoseStamped):
            raise TypeError('target must be PoseTarget or geometry_msgs.msg.PoseStamped')
        pose = target.pose
        values = (
            pose.position.x,
            pose.position.y,
            pose.position.z,
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w,
        )
        if not all(math.isfinite(value) for value in values):
            raise ValueError('pose values must be finite')
        if not target.header.frame_id.strip():
            raise ValueError('pose frame_id must not be empty')
        quaternion_norm = math.sqrt(sum(value * value for value in values[3:]))
        if quaternion_norm <= 1e-12:
            raise ValueError('pose quaternion must not be zero')
        return target

    @staticmethod
    def _invalid(message: str, command: str = '') -> Result:
        return Result(
            False,
            ErrorCode.INVALID_ARGUMENT,
            message,
            command=command,
            error_name='INVALID_ARGUMENT',
        )

    @staticmethod
    def _not_ready(message: str, command: str) -> Result:
        return Result(
            False,
            ErrorCode.NOT_READY,
            message,
            command=command,
            error_name='NOT_READY',
        )
