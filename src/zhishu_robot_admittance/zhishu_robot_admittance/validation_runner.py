"""Run the reference box-carry admittance acceptance trajectory."""

from builtin_interfaces.msg import Duration
from dataclasses import dataclass
import json
import math
from pathlib import Path
import re
import subprocess
import threading
import time

from geometry_msgs.msg import TwistStamped, WrenchStamped
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import JointState
from std_srvs.srv import SetBool, Trigger
from tf2_ros import Buffer, TransformException, TransformListener
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


@dataclass(frozen=True)
class BoxPose:
    sample_time: float
    x: float
    y: float
    z: float
    qx: float
    qy: float
    qz: float
    qw: float


class GazeboPoseTracker:
    """Read the actual carry_box pose from Gazebo Transport."""

    def __init__(self):
        self._lock = threading.Lock()
        self._latest = None
        self._history = []
        self._process = subprocess.Popen(
            [
                "gz",
                "topic",
                "-e",
                "-t",
                "/world/admittance_box/dynamic_pose/info",
            ],
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
            bufsize=1,
        )
        self._thread = threading.Thread(target=self._read, daemon=True)
        self._thread.start()

    def _read(self):
        depth = 0
        section = None
        name = None
        values = {}
        for raw_line in self._process.stdout:
            line = raw_line.strip()
            if line == "pose {":
                depth = 1
                section = None
                name = None
                values = {}
                continue
            if depth == 0:
                continue
            if line == "position {":
                section = "position"
            elif line == "orientation {":
                section = "orientation"
            else:
                name_match = re.fullmatch(r'name: "([^"]+)"', line)
                value_match = re.fullmatch(
                    r"([xyzw]): ([-+0-9.eE]+)", line
                )
                if name_match:
                    name = name_match.group(1)
                elif value_match and section:
                    values[(section, value_match.group(1))] = float(
                        value_match.group(2)
                    )

            depth += line.count("{") - line.count("}")
            if line == "}" and depth == 1:
                section = None
            if depth == 0 and name == "carry_box":
                pose = BoxPose(
                    time.monotonic(),
                    values.get(("position", "x"), 0.0),
                    values.get(("position", "y"), 0.0),
                    values.get(("position", "z"), 0.0),
                    values.get(("orientation", "x"), 0.0),
                    values.get(("orientation", "y"), 0.0),
                    values.get(("orientation", "z"), 0.0),
                    values.get(("orientation", "w"), 1.0),
                )
                with self._lock:
                    self._latest = pose
                    self._history.append(pose)

    def wait_for_pose(self, timeout=5.0):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            pose = self.latest()
            if pose is not None:
                return pose
            time.sleep(0.02)
        raise RuntimeError("Timed out waiting for the Gazebo carry_box pose")

    def latest(self):
        with self._lock:
            return self._latest

    def history_since(self, sample_time):
        with self._lock:
            return [pose for pose in self._history if pose.sample_time >= sample_time]

    def close(self):
        self._process.terminate()
        try:
            self._process.wait(timeout=2.0)
        except subprocess.TimeoutExpired:
            self._process.kill()


class ValidationRunner(Node):
    """Publish and validate a 200 mm straight-line carry trajectory."""

    def __init__(self):
        super().__init__("admittance_validation_runner")
        self.publisher = self.create_publisher(
            TwistStamped, "/dual_arm_admittance/reference_twist", 10
        )
        self.body_publisher = self.create_publisher(
            JointTrajectory, "/body_posture_controller/joint_trajectory", 10
        )
        self.declare_parameter("linear_distance_m", 0.200)
        self.declare_parameter("linear_speed_mps", 0.020)
        self.declare_parameter("arm_linear_distance_m", 0.040)
        self.declare_parameter("linear_axis", "z")
        # A vertical straight line avoids the horizontal singularities and the
        # support-table friction of the original lift-then-forward profile.
        self.declare_parameter("linear_direction", 1.0)
        self.declare_parameter("cross_track_feedback_gain", 0.0)
        self.declare_parameter("cross_track_feedback_max_speed_mps", 0.020)
        self.declare_parameter("output_path", "")
        self.tare_client = self.create_client(Trigger, "/dual_arm_admittance/tare")
        self.enable_client = self.create_client(SetBool, "/dual_arm_admittance/enable")
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.sensor_counts = {"left": 0, "right": 0}
        self.sensor_invalid = False
        self.sensor_stamps = {"left": None, "right": None}
        self.max_sensor_skew = 0.0
        self.command_history = {"left": [], "right": []}
        self.sim_time = None
        self.current_waist_pitch = None
        self.create_subscription(
            Clock, "/clock", self._clock_callback, qos_profile_sensor_data
        )
        self.create_subscription(
            JointState,
            "/joint_states",
            self._joint_states_callback,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            WrenchStamped,
            "/left_wrist_ft",
            lambda message: self._wrench("left", message),
            qos_profile_sensor_data,
        )
        self.create_subscription(
            WrenchStamped,
            "/right_wrist_ft",
            lambda message: self._wrench("right", message),
            qos_profile_sensor_data,
        )
        self.create_subscription(
            TwistStamped,
            "/left_servo/delta_twist_cmds",
            lambda message: self._command("left", message),
            10,
        )
        self.create_subscription(
            TwistStamped,
            "/right_servo/delta_twist_cmds",
            lambda message: self._command("right", message),
            10,
        )
        self.initial_tcp_distance = None
        self.max_tcp_distance_change = 0.0

    def _wrench(self, side, message):
        values = (
            message.wrench.force.x,
            message.wrench.force.y,
            message.wrench.force.z,
            message.wrench.torque.x,
            message.wrench.torque.y,
            message.wrench.torque.z,
        )
        self.sensor_invalid |= not all(math.isfinite(value) for value in values)
        self.sensor_counts[side] += 1
        self.sensor_stamps[side] = message.header.stamp.sec + (
            1e-9 * message.header.stamp.nanosec
        )
        if all(stamp is not None for stamp in self.sensor_stamps.values()):
            skew = abs(self.sensor_stamps["left"] - self.sensor_stamps["right"])
            self.max_sensor_skew = max(self.max_sensor_skew, skew)

    def _command(self, side, message):
        stamp = message.header.stamp.sec + 1e-9 * message.header.stamp.nanosec
        self.command_history[side].append(
            (
                stamp,
                message.twist.linear.x,
                message.twist.linear.y,
                message.twist.linear.z,
            )
        )

    def _clock_callback(self, message):
        self.sim_time = message.clock.sec + 1e-9 * message.clock.nanosec

    def _joint_states_callback(self, message):
        if "waist_pitch_joint" in message.name:
            index = message.name.index("waist_pitch_joint")
            if index < len(message.position):
                value = float(message.position[index])
                if math.isfinite(value):
                    self.current_waist_pitch = value

    def wait_ready(self, timeout=30.0):
        for client in (self.tare_client, self.enable_client):
            if not client.wait_for_service(timeout_sec=timeout):
                raise RuntimeError(f"Service {client.srv_name} is unavailable")
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
            if (
                min(self.sensor_counts.values()) >= 20
                and self.sim_time is not None
                and self.current_waist_pitch is not None
            ):
                return
        raise RuntimeError("Force/torque sensors did not provide 20 samples")

    def call_tare(self):
        future = self.tare_client.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        if not future.done() or not future.result().success:
            raise RuntimeError("Admittance tare request failed")

    def set_enabled(self, enabled):
        request = SetBool.Request()
        request.data = enabled
        future = self.enable_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        if not future.done() or not future.result().success:
            state = "enable" if enabled else "disable"
            raise RuntimeError(f"Admittance {state} request failed")

    def _sample_tcp_distance(self):
        try:
            left = self.tf_buffer.lookup_transform(
                "base_link", "left_gripper_tcp", Time()
            )
            right = self.tf_buffer.lookup_transform(
                "base_link", "right_gripper_tcp", Time()
            )
        except TransformException:
            return
        a = left.transform.translation
        b = right.transform.translation
        distance = math.sqrt((a.x - b.x) ** 2 + (a.y - b.y) ** 2 + (a.z - b.z) ** 2)
        if self.initial_tcp_distance is None:
            self.initial_tcp_distance = distance
        self.max_tcp_distance_change = max(
            self.max_tcp_distance_change, abs(distance - self.initial_tcp_distance)
        )

    def publish_profile(
        self,
        duration,
        axis=None,
        peak=0.0,
        ramp=0.5,
        pose_tracker=None,
        line_start=None,
        cross_track_gain=0.0,
        cross_track_max_speed=0.020,
    ):
        if self.sim_time is None:
            raise RuntimeError("Simulation clock is unavailable")
        start = self.sim_time
        next_tick = start
        wall_deadline = time.monotonic() + max(10.0, 5.0 * duration)
        while True:
            rclpy.spin_once(self, timeout_sec=0.005)
            now = self.sim_time
            elapsed = now - start
            if elapsed >= duration:
                break
            if time.monotonic() >= wall_deadline:
                raise RuntimeError("Simulation clock stopped while publishing profile")
            if now < next_tick:
                continue
            if axis is None:
                speed = 0.0
            elif elapsed < ramp:
                speed = peak * elapsed / ramp
            elif elapsed > duration - ramp:
                speed = peak * max(0.0, duration - elapsed) / ramp
            else:
                speed = peak
            message = TwistStamped()
            message.header.stamp.sec = int(now)
            message.header.stamp.nanosec = int((now - int(now)) * 1e9)
            message.header.frame_id = "base_link"
            setattr(message.twist.linear, axis or "x", speed)
            if (
                cross_track_gain > 0.0
                and pose_tracker is not None
                and line_start is not None
            ):
                pose = pose_tracker.latest()
                if pose is None or time.monotonic() - pose.sample_time > 0.5:
                    raise RuntimeError("Gazebo box pose feedback is stale")
                for other in "xyz":
                    if other == axis:
                        continue
                    correction = cross_track_gain * (
                        axis_value(line_start, other) - axis_value(pose, other)
                    )
                    correction = max(
                        -cross_track_max_speed,
                        min(cross_track_max_speed, correction),
                    )
                    setattr(message.twist.linear, other, correction)
            self.publisher.publish(message)
            self._sample_tcp_distance()
            next_tick += 0.01

    def publish_waist_profile(self, duration, distance, ramp=0.5):
        if self.current_waist_pitch is None:
            raise RuntimeError("Current waist_pitch_joint state is unavailable")
        waist_pitch = self.current_waist_pitch
        deadline = time.monotonic() + 5.0
        while self.body_publisher.get_subscription_count() == 0:
            if time.monotonic() >= deadline:
                raise RuntimeError("Body posture controller is unavailable")
            rclpy.spin_once(self, timeout_sec=0.05)

        peak = distance / (duration - ramp)
        trajectory = JointTrajectory()
        trajectory.joint_names = [
            "waist_up_down_joint",
            "waist_pitch_joint",
            "head_yaw_joint",
            "head_pitch_joint",
        ]
        sample_period = 0.05
        samples = math.ceil(duration / sample_period)
        for index in range(1, samples + 1):
            elapsed = min(index * sample_period, duration)
            point = JointTrajectoryPoint()
            point.positions = [
                trapezoid_position(elapsed, duration, peak, ramp),
                waist_pitch,
                0.0,
                0.0,
            ]
            point.time_from_start = Duration(
                sec=int(elapsed),
                nanosec=int((elapsed - int(elapsed)) * 1e9),
            )
            trajectory.points.append(point)
        self.body_publisher.publish(trajectory)


def displacement(a, b):
    return math.sqrt((b.x - a.x) ** 2 + (b.y - a.y) ** 2 + (b.z - a.z) ** 2)


def axis_value(pose, axis):
    return getattr(pose, axis)


def cross_track_distance(start, pose, axis):
    return math.sqrt(
        sum(
            (axis_value(pose, other) - axis_value(start, other)) ** 2
            for other in "xyz"
            if other != axis
        )
    )


def trapezoid_position(elapsed, duration, peak, ramp):
    elapsed = max(0.0, min(elapsed, duration))
    total = peak * (duration - ramp)
    if elapsed < ramp:
        return 0.5 * peak * elapsed * elapsed / ramp
    if elapsed <= duration - ramp:
        return peak * (elapsed - 0.5 * ramp)
    remaining = duration - elapsed
    return total - 0.5 * peak * remaining * remaining / ramp


def integrate_commands(history):
    samples = history
    result = [0.0, 0.0, 0.0]
    for previous, current in zip(samples, samples[1:]):
        dt = current[0] - previous[0]
        if dt <= 0.0 or dt > 0.05:
            continue
        for axis in range(3):
            result[axis] += 0.5 * (previous[axis + 1] + current[axis + 1]) * dt
    return result


def quaternion_to_rpy(pose):
    x, y, z, w = pose.qx, pose.qy, pose.qz, pose.qw
    roll = math.atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))
    pitch = math.asin(max(-1.0, min(1.0, 2.0 * (w * y - z * x))))
    yaw = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
    return roll, pitch, yaw


def main(args=None):
    rclpy.init(args=args)
    runner = ValidationRunner()
    tracker = GazeboPoseTracker()
    enabled = False
    try:
        runner.wait_ready()
        tracker.wait_for_pose()
        runner.call_tare()
        runner.publish_profile(2.0)
        runner.set_enabled(True)
        enabled = True
        runner.publish_profile(0.5)

        start = tracker.latest()
        trajectory_start_time = start.sample_time
        line_start_time = start.sample_time
        runner.command_history = {"left": [], "right": []}
        requested_distance = abs(
            float(runner.get_parameter("linear_distance_m").value)
        )
        requested_speed = abs(
            float(runner.get_parameter("linear_speed_mps").value)
        )
        arm_distance = abs(
            float(runner.get_parameter("arm_linear_distance_m").value)
        )
        linear_axis = str(runner.get_parameter("linear_axis").value).lower()
        if linear_axis not in "xyz" or len(linear_axis) != 1:
            raise RuntimeError("linear_axis must be one of: x, y, z")
        direction = (
            -1.0
            if float(runner.get_parameter("linear_direction").value) < 0.0
            else 1.0
        )
        if requested_distance <= 0.0 or requested_speed <= 0.0:
            raise RuntimeError("linear_distance_m and linear_speed_mps must be positive")
        cross_track_gain = float(runner.get_parameter("cross_track_feedback_gain").value)
        cross_track_max_speed = float(
            runner.get_parameter("cross_track_feedback_max_speed_mps").value
        )
        if not math.isfinite(cross_track_gain) or cross_track_gain < 0.0:
            raise RuntimeError(
                "cross_track_feedback_gain must be finite and nonnegative"
            )
        if not math.isfinite(cross_track_max_speed) or cross_track_max_speed <= 0.0:
            raise RuntimeError(
                "cross_track_feedback_max_speed_mps must be finite and positive"
            )
        if arm_distance >= requested_distance:
            raise RuntimeError("arm_linear_distance_m must be below linear_distance_m")
        waist_distance = requested_distance - arm_distance
        if waist_distance > 0.3:
            raise RuntimeError("Requested waist contribution exceeds its 0.3 m limit")
        line_ramp = 0.5
        line_duration = requested_distance / requested_speed + line_ramp
        arm_peak = arm_distance / (line_duration - line_ramp)
        runner.publish_waist_profile(
            line_duration, direction * waist_distance, ramp=line_ramp
        )
        runner.publish_profile(
            line_duration,
            axis=linear_axis,
            peak=direction * arm_peak,
            ramp=line_ramp,
            pose_tracker=tracker,
            line_start=start,
            cross_track_gain=cross_track_gain,
            cross_track_max_speed=cross_track_max_speed,
        )
        line_end = tracker.latest()
        runner.publish_profile(0.5)
        stopped_05 = tracker.latest()
        runner.publish_profile(2.5)
        final = tracker.latest()
        runner.set_enabled(False)
        enabled = False

        history = tracker.history_since(trajectory_start_time)
        line_history = tracker.history_since(line_start_time)
        initial_rpy = quaternion_to_rpy(start)
        max_angles = [0.0, 0.0, 0.0]
        for pose in history:
            rpy = quaternion_to_rpy(pose)
            for index in range(3):
                delta = math.atan2(
                    math.sin(rpy[index] - initial_rpy[index]),
                    math.cos(rpy[index] - initial_rpy[index]),
                )
                max_angles[index] = max(max_angles[index], abs(delta))

        # The stop window is exactly 0.5 s in simulation time. Wall time is
        # invalid here because Gazebo may run below real time on this host.
        stop_interval = 0.5
        metrics = {
            "box_start_xyz_m": [start.x, start.y, start.z],
            "box_final_xyz_m": [final.x, final.y, final.z],
            "linear_axis": linear_axis,
            "requested_linear_distance_m": requested_distance,
            "commanded_arm_distance_m": arm_distance,
            "commanded_waist_distance_m": waist_distance,
            "cross_track_feedback_gain": cross_track_gain,
            "cross_track_feedback_max_speed_mps": cross_track_max_speed,
            "linear_axis_displacement_m": direction
            * (axis_value(final, linear_axis) - axis_value(start, linear_axis)),
            "linear_distance_m": displacement(start, final),
            "max_linear_cross_track_error_m": max(
                (
                    cross_track_distance(start, pose, linear_axis)
                    for pose in line_history
                ),
                default=0.0,
            ),
            "max_tcp_distance_change_m": runner.max_tcp_distance_change,
            "max_box_rpy_change_deg": [math.degrees(value) for value in max_angles],
            "stop_speed_mps": displacement(line_end, stopped_05) / stop_interval,
            "final_hold_drift_m": displacement(stopped_05, final),
            "left_sensor_samples": runner.sensor_counts["left"],
            "right_sensor_samples": runner.sensor_counts["right"],
            "max_observed_sensor_skew_s": runner.max_sensor_skew,
            "sensor_nonfinite": runner.sensor_invalid,
            "gazebo_pose_samples": len(history),
            "left_servo_command_integral_xyz_m": integrate_commands(
                runner.command_history["left"]
            ),
            "right_servo_command_integral_xyz_m": integrate_commands(
                runner.command_history["right"]
            ),
            "left_servo_command_samples": len(runner.command_history["left"]),
            "right_servo_command_samples": len(runner.command_history["right"]),
        }
        checks = {
            "straight_line_200_plus_minus_10_mm": 0.190
            <= metrics["linear_axis_displacement_m"]
            <= 0.210,
            "linear_cross_track_error_at_most_5_mm": metrics[
                "max_linear_cross_track_error_m"
            ]
            <= 0.005,
            "tcp_distance_change_at_most_5_mm": metrics[
                "max_tcp_distance_change_m"
            ] <= 0.005,
            "box_rpy_change_at_most_3_deg": max(
                metrics["max_box_rpy_change_deg"]
            ) <= 3.0,
            "stop_speed_below_2_mm_s": metrics["stop_speed_mps"] <= 0.002,
            "final_hold_drift_at_most_3_mm": metrics["final_hold_drift_m"] <= 0.003,
            "sensor_data_finite": not metrics["sensor_nonfinite"],
        }
        report = {"passed": all(checks.values()), "checks": checks, "metrics": metrics}
        serialized = json.dumps(report, ensure_ascii=False, indent=2)
        output_path = str(runner.get_parameter("output_path").value)
        if output_path:
            Path(output_path).write_text(serialized + "\n", encoding="utf-8")
        print(serialized)
        return 0 if report["passed"] else 2
    finally:
        if enabled:
            try:
                runner.set_enabled(False)
            except RuntimeError:
                pass
        tracker.close()
        runner.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
