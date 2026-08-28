"""Cooperative Cartesian admittance controller for a dual-arm box grasp."""

from dataclasses import dataclass
import math
from typing import List, Optional, Tuple

from geometry_msgs.msg import TwistStamped, WrenchStamped
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from std_srvs.srv import SetBool, Trigger
from tf2_ros import Buffer, TransformException, TransformListener

from .admittance_math import admittance_step
from .admittance_math import apply_deadband
from .admittance_math import clamp
from .admittance_math import cross
from .admittance_math import rotate_vector


@dataclass
class SensorSample:
    wrench: List[float]
    received_ns: int
    sample_ns: int
    generation: int


class DualArmAdmittanceController(Node):
    """Combine two wrist wrenches into common and internal compliant motion."""

    def __init__(self):
        super().__init__("dual_arm_admittance")

        self.base_frame = self._declare("base_frame", "base_link")
        self.left_sensor_frame = self._declare("left_sensor_frame", "L_hand_link")
        self.right_sensor_frame = self._declare("right_sensor_frame", "R_hand_link")
        self.left_wrench_topic = self._declare("left_wrench_topic", "/left_wrist_ft")
        self.right_wrench_topic = self._declare("right_wrench_topic", "/right_wrist_ft")
        self.reference_topic = self._declare(
            "reference_twist_topic", "/dual_arm_admittance/reference_twist"
        )
        self.left_command_topic = self._declare(
            "left_command_topic", "/left_servo/delta_twist_cmds"
        )
        self.right_command_topic = self._declare(
            "right_command_topic", "/right_servo/delta_twist_cmds"
        )

        self.control_rate = float(self._declare("control_rate_hz", 100.0))
        self.sensor_timeout = float(self._declare("sensor_timeout", 0.10))
        self.max_sensor_skew = float(self._declare("max_sensor_skew_sec", 0.02))
        self.reference_timeout = float(self._declare("reference_timeout", 0.25))
        self.filter_cutoff = float(self._declare("wrench_filter_cutoff_hz", 12.0))
        self.tare_samples_required = int(self._declare("tare_samples", 100))
        self.enable_after_initial_tare = bool(
            self._declare("enable_on_start", False)
        )
        self.enabled = False
        self.input_fault_latched = False

        self.mass = self._vector("mass", [8.0, 8.0, 8.0, 1.5, 1.5, 1.5])
        self.damping = self._vector(
            "damping", [80.0, 80.0, 100.0, 10.0, 10.0, 10.0]
        )
        self.stiffness = self._vector(
            "stiffness", [150.0, 150.0, 200.0, 15.0, 15.0, 15.0]
        )
        self.deadband = self._vector("wrench_deadband", [1.0, 1.0, 1.5, 0.1, 0.1, 0.1])
        self.desired_wrench = self._vector("desired_common_wrench", [0.0] * 6)
        self.desired_internal = self._vector("desired_internal_wrench", [0.0] * 6)
        self.balance_gain = self._vector(
            "internal_balance_gain", [0.001, 0.001, 0.001, 0.003, 0.003, 0.003]
        )
        self.max_balance_velocity = self._vector(
            "max_internal_balance_velocity", [0.005, 0.005, 0.005, 0.01, 0.01, 0.01]
        )
        self.max_common_wrench = self._vector(
            "max_common_wrench", [100.0, 100.0, 100.0, 20.0, 20.0, 20.0]
        )
        self.left_sign = self._vector("left_wrench_sign", [1.0] * 6)
        self.right_sign = self._vector("right_wrench_sign", [1.0] * 6)
        self.max_displacement = self._vector(
            "max_admittance_displacement", [0.03, 0.03, 0.03, 0.15, 0.15, 0.15]
        )
        self.max_admittance_velocity = self._vector(
            "max_admittance_velocity", [0.005, 0.005, 0.005, 0.02, 0.02, 0.02]
        )
        self.max_linear_velocity = float(self._declare("max_linear_velocity", 0.025))
        self.max_angular_velocity = float(self._declare("max_angular_velocity", 0.05))
        self.transform_timeout = float(self._declare("transform_timeout", 0.05))
        self._validate_parameters()

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.left_sample: Optional[SensorSample] = None
        self.right_sample: Optional[SensorSample] = None
        self.left_generation = 0
        self.right_generation = 0
        self.reference = [0.0] * 6
        self.reference_received_ns = 0

        self.left_bias_sum = [0.0] * 6
        self.right_bias_sum = [0.0] * 6
        self.left_bias = [0.0] * 6
        self.right_bias = [0.0] * 6
        self.tare_count = 0
        self.tared = False
        self.last_tare_left_generation = -1
        self.last_tare_right_generation = -1

        self.filtered_common = [0.0] * 6
        self.admittance_position = [0.0] * 6
        self.admittance_velocity = [0.0] * 6
        self.last_update_ns = 0
        self.last_warning_ns = 0

        self.left_command_pub = self.create_publisher(
            TwistStamped, self.left_command_topic, 10
        )
        self.right_command_pub = self.create_publisher(
            TwistStamped, self.right_command_topic, 10
        )
        self.common_wrench_pub = self.create_publisher(
            WrenchStamped, "/dual_arm_admittance/common_wrench", 10
        )
        self.internal_wrench_pub = self.create_publisher(
            WrenchStamped, "/dual_arm_admittance/internal_wrench", 10
        )

        self.create_subscription(
            WrenchStamped,
            self.left_wrench_topic,
            self._left_wrench_callback,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            WrenchStamped,
            self.right_wrench_topic,
            self._right_wrench_callback,
            qos_profile_sensor_data,
        )
        self.create_subscription(TwistStamped, self.reference_topic, self._reference_callback, 10)
        self.create_service(SetBool, "~/enable", self._enable_callback)
        self.create_service(Trigger, "~/tare", self._tare_callback)
        self.create_timer(1.0 / self.control_rate, self._update)

        self.get_logger().info(
            "Dual-arm admittance ready: %s + %s -> %s / %s"
            % (
                self.left_wrench_topic,
                self.right_wrench_topic,
                self.left_command_topic,
                self.right_command_topic,
            )
        )

    def _declare(self, name, default):
        return self.declare_parameter(name, default).value

    def _vector(self, name: str, default: List[float]) -> List[float]:
        values = [float(value) for value in self._declare(name, default)]
        if len(values) != 6:
            raise ValueError(f"parameter '{name}' must contain six values")
        return values

    def _validate_parameters(self):
        scalar_parameters = [
            self.control_rate,
            self.sensor_timeout,
            self.max_sensor_skew,
            self.reference_timeout,
            self.transform_timeout,
            self.filter_cutoff,
            self.max_linear_velocity,
            self.max_angular_velocity,
        ]
        vector_parameters = (
            self.mass
            + self.damping
            + self.stiffness
            + self.deadband
            + self.desired_wrench
            + self.desired_internal
            + self.balance_gain
            + self.max_balance_velocity
            + self.max_common_wrench
            + self.left_sign
            + self.right_sign
            + self.max_displacement
            + self.max_admittance_velocity
        )
        if not all(math.isfinite(value) for value in scalar_parameters + vector_parameters):
            raise ValueError("all numeric parameters must be finite")
        if self.control_rate <= 0.0:
            raise ValueError("control_rate_hz must be positive")
        if (
            self.sensor_timeout <= 0.0
            or self.max_sensor_skew < 0.0
            or self.reference_timeout <= 0.0
            or self.transform_timeout <= 0.0
        ):
            raise ValueError("message timeouts must be positive")
        if self.filter_cutoff < 0.0:
            raise ValueError("wrench_filter_cutoff_hz cannot be negative")
        if self.tare_samples_required < 1:
            raise ValueError("tare_samples must be at least one")
        if any(value <= 0.0 for value in self.mass):
            raise ValueError("all virtual mass values must be positive")
        if any(value < 0.0 for value in self.damping + self.stiffness):
            raise ValueError("damping and stiffness cannot be negative")
        if any(value < 0.0 for value in self.deadband + self.balance_gain):
            raise ValueError("deadband and balance gains cannot be negative")
        if any(
            value <= 0.0
            for value in (
                self.max_displacement
                + self.max_admittance_velocity
                + self.max_balance_velocity
                + self.max_common_wrench
            )
        ):
            raise ValueError("admittance limits must be positive")
        if self.max_linear_velocity <= 0.0 or self.max_angular_velocity <= 0.0:
            raise ValueError("velocity limits must be positive")

    @staticmethod
    def _stamp_ns(message) -> int:
        stamp_ns = int(message.header.stamp.sec) * 1_000_000_000
        stamp_ns += int(message.header.stamp.nanosec)
        return stamp_ns

    @staticmethod
    def _is_finite(values: List[float]) -> bool:
        return all(math.isfinite(value) for value in values)

    def _reject_input(self, message: str):
        if self.enabled and self.tared:
            self.enabled = False
            self.input_fault_latched = True
            self.reference = [0.0] * 6
            self.reference_received_ns = 0
            self.get_logger().error(
                "Admittance disabled by input safety fault; explicit enable is required"
            )
        self._reset_dynamics()
        self._publish_zero()
        self._warn_throttled(message)

    @staticmethod
    def _wrench_to_list(message: WrenchStamped) -> List[float]:
        return [
            message.wrench.force.x,
            message.wrench.force.y,
            message.wrench.force.z,
            message.wrench.torque.x,
            message.wrench.torque.y,
            message.wrench.torque.z,
        ]

    def _left_wrench_callback(self, message: WrenchStamped):
        wrench = self._wrench_to_list(message)
        if not self._is_finite(wrench):
            self.left_sample = None
            self._reject_input("Rejected non-finite left force/torque sample")
            return
        now_ns = self.get_clock().now().nanoseconds
        self.left_generation += 1
        self.left_sample = SensorSample(
            wrench,
            now_ns,
            self._stamp_ns(message) or now_ns,
            self.left_generation,
        )

    def _right_wrench_callback(self, message: WrenchStamped):
        wrench = self._wrench_to_list(message)
        if not self._is_finite(wrench):
            self.right_sample = None
            self._reject_input("Rejected non-finite right force/torque sample")
            return
        now_ns = self.get_clock().now().nanoseconds
        self.right_generation += 1
        self.right_sample = SensorSample(
            wrench,
            now_ns,
            self._stamp_ns(message) or now_ns,
            self.right_generation,
        )

    def _reference_callback(self, message: TwistStamped):
        if message.header.frame_id not in ("", self.base_frame):
            self.reference = [0.0] * 6
            self.reference_received_ns = 0
            self._reject_input(
                f"Ignoring reference in '{message.header.frame_id}'; expected '{self.base_frame}'"
            )
            return
        reference = [
            message.twist.linear.x,
            message.twist.linear.y,
            message.twist.linear.z,
            message.twist.angular.x,
            message.twist.angular.y,
            message.twist.angular.z,
        ]
        if not self._is_finite(reference):
            self.reference = [0.0] * 6
            self.reference_received_ns = 0
            self._reject_input("Rejected non-finite reference twist")
            return
        self.reference = reference
        self.reference_received_ns = self.get_clock().now().nanoseconds

    def _enable_callback(self, request: SetBool.Request, response: SetBool.Response):
        if not request.data:
            self.enabled = False
            self.enable_after_initial_tare = False
            self.input_fault_latched = False
            self.reference = [0.0] * 6
            self.reference_received_ns = 0
            self._reset_dynamics()
            self._publish_zero()
            response.success = True
            response.message = "admittance disabled"
            self.get_logger().info(response.message)
            return response

        now_ns = self.get_clock().now().nanoseconds
        if not self.tared:
            response.success = False
            response.message = "tare is incomplete; admittance remains disabled"
            return response
        if not self._sample_is_fresh(self.left_sample, now_ns) or not self._sample_is_fresh(
            self.right_sample, now_ns
        ):
            response.success = False
            response.message = "force/torque samples are stale; admittance remains disabled"
            return response
        sample_skew = abs(self.left_sample.sample_ns - self.right_sample.sample_ns) * 1e-9
        if sample_skew > self.max_sensor_skew:
            response.success = False
            response.message = "force/torque samples are not synchronized"
            return response

        self._reset_dynamics()
        self.enabled = True
        self.input_fault_latched = False
        response.success = True
        response.message = "admittance enabled"
        self.get_logger().info(response.message)
        return response

    def _tare_callback(self, request: Trigger.Request, response: Trigger.Response):
        del request
        self.enabled = False
        self.enable_after_initial_tare = False
        self.input_fault_latched = False
        self.reference = [0.0] * 6
        self.reference_received_ns = 0
        self._begin_tare()
        self._publish_zero()
        response.success = True
        response.message = f"collecting {self.tare_samples_required} tare samples"
        self.get_logger().info(response.message)
        return response

    def _begin_tare(self):
        self.left_bias_sum = [0.0] * 6
        self.right_bias_sum = [0.0] * 6
        self.tare_count = 0
        self.tared = False
        self.last_tare_left_generation = self.left_generation
        self.last_tare_right_generation = self.right_generation
        self._reset_dynamics()

    def _reset_dynamics(self):
        self.filtered_common = [0.0] * 6
        self.admittance_position = [0.0] * 6
        self.admittance_velocity = [0.0] * 6

    def _warn_throttled(self, message: str):
        now_ns = self.get_clock().now().nanoseconds
        if now_ns - self.last_warning_ns >= 2_000_000_000 or self.last_warning_ns == 0:
            self.get_logger().warning(message)
            self.last_warning_ns = now_ns

    def _sample_is_fresh(self, sample: Optional[SensorSample], now_ns: int) -> bool:
        if sample is None:
            return False
        age = max(0.0, (now_ns - sample.received_ns) * 1e-9)
        return age <= self.sensor_timeout

    def _transform_sensor(
        self, sample: SensorSample, sensor_frame: str, signs: List[float]
    ) -> Tuple[List[float], List[float]]:
        transform = self.tf_buffer.lookup_transform(
            self.base_frame,
            sensor_frame,
            Time(),
            timeout=Duration(seconds=self.transform_timeout),
        )
        rotation = transform.transform.rotation
        quaternion = [rotation.x, rotation.y, rotation.z, rotation.w]
        signed = [value * sign for value, sign in zip(sample.wrench, signs)]
        force = rotate_vector(signed[:3], quaternion)
        torque = rotate_vector(signed[3:], quaternion)
        translation = transform.transform.translation
        position = [translation.x, translation.y, translation.z]
        if not self._is_finite(force + torque + position):
            raise ValueError("force/torque transform contains non-finite values")
        return force + torque, position

    def _collect_tare(
        self,
        left: List[float],
        right: List[float],
        left_generation: int,
        right_generation: int,
    ):
        if (
            left_generation == self.last_tare_left_generation
            or right_generation == self.last_tare_right_generation
        ):
            return
        self.last_tare_left_generation = left_generation
        self.last_tare_right_generation = right_generation
        self.left_bias_sum = [total + value for total, value in zip(self.left_bias_sum, left)]
        self.right_bias_sum = [
            total + value for total, value in zip(self.right_bias_sum, right)
        ]
        self.tare_count += 1
        if self.tare_count >= self.tare_samples_required:
            divisor = float(self.tare_count)
            self.left_bias = [value / divisor for value in self.left_bias_sum]
            self.right_bias = [value / divisor for value in self.right_bias_sum]
            self.tared = True
            self.get_logger().info(
                "Force/torque tare complete: left=%s right=%s"
                % (
                    [round(value, 3) for value in self.left_bias],
                    [round(value, 3) for value in self.right_bias],
                )
            )
            if self.enable_after_initial_tare:
                self.enable_after_initial_tare = False
                self.enabled = True
                self.get_logger().warning(
                    "Initial tare complete; admittance enabled by enable_on_start"
                )

    @staticmethod
    def _subtract_bias(values: List[float], bias: List[float]) -> List[float]:
        return [value - offset for value, offset in zip(values, bias)]

    def _compute_wrenches(
        self,
        left: List[float],
        right: List[float],
        left_position: List[float],
        right_position: List[float],
    ) -> Tuple[List[float], List[float], List[float], List[float]]:
        center = [0.5 * (left_position[i] + right_position[i]) for i in range(3)]
        left_offset = [left_position[i] - center[i] for i in range(3)]
        right_offset = [right_position[i] - center[i] for i in range(3)]
        common_force = [left[i] + right[i] for i in range(3)]
        common_torque = [left[i] + right[i] for i in range(3, 6)]
        left_moment = cross(left_offset, left[:3])
        right_moment = cross(right_offset, right[:3])
        common_torque = [
            common_torque[i] + left_moment[i] + right_moment[i] for i in range(3)
        ]
        internal = [0.5 * (left[i] - right[i]) for i in range(6)]
        return common_force + common_torque, internal, left_offset, right_offset

    def _update_admittance(self, wrench: List[float], dt: float) -> List[float]:
        bounded_wrench = [
            clamp(value, -limit, limit)
            for value, limit in zip(wrench, self.max_common_wrench)
        ]
        if self.filter_cutoff > 0.0:
            alpha = 1.0 - math.exp(-2.0 * math.pi * self.filter_cutoff * dt)
        else:
            alpha = 1.0
        self.filtered_common = [
            old + alpha * (new - old)
            for old, new in zip(self.filtered_common, bounded_wrench)
        ]

        for axis in range(6):
            error = apply_deadband(
                self.filtered_common[axis] - self.desired_wrench[axis],
                self.deadband[axis],
            )
            position, velocity = admittance_step(
                error,
                self.mass[axis],
                self.damping[axis],
                self.stiffness[axis],
                self.admittance_position[axis],
                self.admittance_velocity[axis],
                dt,
            )
            limit = self.max_displacement[axis]
            position = clamp(position, -limit, limit)
            if abs(position) >= limit and position * velocity > 0.0:
                velocity = 0.0
            velocity_limit = self.max_admittance_velocity[axis]
            self.admittance_position[axis] = position
            self.admittance_velocity[axis] = clamp(
                velocity, -velocity_limit, velocity_limit
            )
        return list(self.admittance_velocity)

    def _fresh_reference(self, now_ns: int) -> List[float]:
        if self.reference_received_ns == 0:
            return [0.0] * 6
        age = max(0.0, (now_ns - self.reference_received_ns) * 1e-9)
        return list(self.reference) if age <= self.reference_timeout else [0.0] * 6

    def _publish_wrench(self, publisher, values: List[float], now):
        message = WrenchStamped()
        message.header.stamp = now.to_msg()
        message.header.frame_id = self.base_frame
        message.wrench.force.x, message.wrench.force.y, message.wrench.force.z = values[:3]
        message.wrench.torque.x, message.wrench.torque.y, message.wrench.torque.z = values[3:]
        publisher.publish(message)

    def _make_twist(self, values: List[float], now) -> TwistStamped:
        message = TwistStamped()
        message.header.stamp = now.to_msg()
        message.header.frame_id = self.base_frame
        message.twist.linear.x, message.twist.linear.y, message.twist.linear.z = values[:3]
        message.twist.angular.x, message.twist.angular.y, message.twist.angular.z = values[3:]
        return message

    def _limit_twist(self, values: List[float]) -> List[float]:
        if not self._is_finite(values):
            raise FloatingPointError("command twist contains non-finite values")
        return [
            clamp(value, -self.max_linear_velocity, self.max_linear_velocity)
            if index < 3
            else clamp(value, -self.max_angular_velocity, self.max_angular_velocity)
            for index, value in enumerate(values)
        ]

    def _publish_zero(self):
        now = self.get_clock().now()
        zero = self._make_twist([0.0] * 6, now)
        self.left_command_pub.publish(zero)
        self.right_command_pub.publish(zero)

    def _update(self):
        now = self.get_clock().now()
        now_ns = now.nanoseconds
        if self.last_update_ns == 0:
            dt = 1.0 / self.control_rate
        else:
            dt = clamp((now_ns - self.last_update_ns) * 1e-9, 0.001, 0.05)
        self.last_update_ns = now_ns

        if not self._sample_is_fresh(self.left_sample, now_ns) or not self._sample_is_fresh(
            self.right_sample, now_ns
        ):
            self._reject_input("Waiting for fresh left and right force/torque samples")
            return

        sample_skew = abs(self.left_sample.sample_ns - self.right_sample.sample_ns) * 1e-9
        if sample_skew > self.max_sensor_skew:
            self._reject_input(
                f"Left/right force samples are not synchronized (skew={sample_skew:.4f}s)"
            )
            return

        try:
            left, left_position = self._transform_sensor(
                self.left_sample, self.left_sensor_frame, self.left_sign
            )
            right, right_position = self._transform_sensor(
                self.right_sample, self.right_sensor_frame, self.right_sign
            )
        except (TransformException, ValueError) as error:
            self._reject_input(f"Force/torque transform unavailable: {error}")
            return

        if not self.tared:
            self._collect_tare(
                left,
                right,
                self.left_sample.generation,
                self.right_sample.generation,
            )
            self._publish_zero()
            return

        left = self._subtract_bias(left, self.left_bias)
        right = self._subtract_bias(right, self.right_bias)
        common, internal, left_offset, right_offset = self._compute_wrenches(
            left, right, left_position, right_position
        )
        self._publish_wrench(self.common_wrench_pub, common, now)
        self._publish_wrench(self.internal_wrench_pub, internal, now)

        if not self.enabled:
            self._reset_dynamics()
            self._publish_zero()
            return

        try:
            compliant = self._update_admittance(common, dt)
            reference = self._fresh_reference(now_ns)
            center_twist = [reference[i] + compliant[i] for i in range(6)]
            internal_error = [internal[i] - self.desired_internal[i] for i in range(6)]
            balance = [
                clamp(
                    self.balance_gain[i] * internal_error[i],
                    -self.max_balance_velocity[i],
                    self.max_balance_velocity[i],
                )
                for i in range(6)
            ]

            left_linear = [
                center_twist[i] + cross(center_twist[3:], left_offset)[i] - balance[i]
                for i in range(3)
            ]
            right_linear = [
                center_twist[i] + cross(center_twist[3:], right_offset)[i] + balance[i]
                for i in range(3)
            ]
            left_twist = self._limit_twist(left_linear + [
                center_twist[i] - balance[i] for i in range(3, 6)
            ])
            right_twist = self._limit_twist(right_linear + [
                center_twist[i] + balance[i] for i in range(3, 6)
            ])
        except (ArithmeticError, ValueError) as error:
            self._reject_input(f"Admittance calculation rejected: {error}")
            return
        self.left_command_pub.publish(self._make_twist(left_twist, now))
        self.right_command_pub.publish(self._make_twist(right_twist, now))


def main(args=None):
    rclpy.init(args=args)
    node = DualArmAdmittanceController()
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
