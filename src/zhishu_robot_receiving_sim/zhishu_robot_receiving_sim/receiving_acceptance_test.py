"""Black-box acceptance test for one complete receiving simulation run."""

import json
import math
import sys
import time
from typing import Dict, List, Optional, Tuple

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import Trigger

from .flow import is_subsequence


EXPECTED_ORDER = [
    "les_receive_qc",
    "les_dispatch",
    "agv_deliver",
    "robot_pick",
    "part_barcode_scan",
    "les_not_pushed",
    "part_barcode_scan",
    "rack_sku_scan",
    "robot_place",
    "les_complete",
    "agv_return",
    "receipt_complete",
]


class ReceivingAcceptanceTest(Node):
    def __init__(self) -> None:
        super().__init__("receiving_acceptance_test")
        self.declare_parameter("timeout_seconds", 300.0)
        self.declare_parameter("terminal_settle_seconds", 1.0)
        self.declare_parameter("expect_barcode_retry", True)
        self.declare_parameter("require_agv_odometry", True)
        self.declare_parameter("minimum_agv_displacement_m", 0.8)
        self.declare_parameter("maximum_agv_return_error_m", 0.30)
        self.declare_parameter("require_commander_result", True)
        self.declare_parameter("minimum_commander_results", 7)
        self.declare_parameter("require_box_pose", True)

        self._reset_client = self.create_client(Trigger, "/receiving_sim/reset")
        self._start_client = self.create_client(Trigger, "/receiving_sim/start")
        self._active_run_id = ""
        self._events: List[Dict] = []
        self._terminal: Optional[Dict] = None
        self._commander_results: List[str] = []
        self._odom_origin: Optional[Tuple[float, float]] = None
        self._maximum_agv_displacement = 0.0
        self._current_agv_displacement = 0.0

        self.create_subscription(String, "/receiving_sim/events", self._event, 100)
        self.create_subscription(String, "/receiving_sim/state", self._state, 10)
        self.create_subscription(
            String, "/zhishu_robot_commander/status", self._commander, 20
        )
        self.create_subscription(
            Odometry, "/receiving_agv/odometry", self._odometry, 20
        )

    def execute(self) -> Tuple[bool, List[str]]:
        timeout = float(self.get_parameter("timeout_seconds").value)
        if timeout <= 0.0 or timeout > 1800.0:
            return False, ["timeout_seconds must be in (0, 1800]"]
        deadline = time.monotonic() + timeout
        reset = self._call(self._reset_client, deadline)
        if reset is None or not reset.success:
            message = "no response" if reset is None else reset.message
            return False, [f"reset failed: {message}"]
        start = self._call(self._start_client, deadline)
        if start is None or not start.success:
            message = "no response" if start is None else start.message
            return False, [f"start failed: {message}"]
        self._active_run_id = start.message
        self.get_logger().info(f"Acceptance run started: {self._active_run_id}")

        while rclpy.ok() and self._terminal is None and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
        if self._terminal is None:
            return False, ["workflow did not reach a terminal state before timeout"]
        settle_seconds = float(self.get_parameter("terminal_settle_seconds").value)
        if settle_seconds < 0.0 or settle_seconds > 10.0:
            return False, ["terminal_settle_seconds must be in [0, 10]"]
        # State and odometry use separate ROS topics, so the terminal state can
        # arrive before the final bridged Gazebo odometry sample.  Drain that
        # short tail before checking the AGV return error.
        settle_deadline = min(deadline, time.monotonic() + settle_seconds)
        while rclpy.ok() and time.monotonic() < settle_deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
        return self._evaluate()

    def _call(self, client, deadline: float):
        while not client.wait_for_service(timeout_sec=0.2):
            if time.monotonic() >= deadline:
                return None
        future = client.call_async(Trigger.Request())
        while rclpy.ok() and not future.done() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
        return future.result() if future.done() else None

    def _evaluate(self) -> Tuple[bool, List[str]]:
        failures = []
        if self._terminal.get("state") != "receipt_complete":
            failures.append(
                f"terminal state was {self._terminal.get('state')}: "
                f"{self._terminal.get('details')}"
            )

        observed = []
        for event in self._events:
            state = event.get("state")
            if not observed or observed[-1] != state:
                observed.append(state)
        expected = list(EXPECTED_ORDER)
        if not bool(self.get_parameter("expect_barcode_retry").value):
            expected = [state for state in expected if state != "les_not_pushed"]
            first_scan = expected.index("part_barcode_scan")
            if expected[first_scan + 1] == "part_barcode_scan":
                expected.pop(first_scan + 1)
        if not is_subsequence(expected, observed):
            failures.append(f"state order mismatch; observed={observed}")

        if bool(self.get_parameter("expect_barcode_retry").value):
            retried = any(
                event.get("state") == "part_barcode_scan"
                and event.get("status") == "RETRY"
                for event in self._events
            )
            if not retried:
                failures.append("expected barcode RETRY event was not observed")

        if bool(self.get_parameter("require_agv_odometry").value):
            minimum = float(
                self.get_parameter("minimum_agv_displacement_m").value
            )
            if self._maximum_agv_displacement < minimum:
                failures.append(
                    "AGV odometry displacement "
                    f"{self._maximum_agv_displacement:.3f} m is below {minimum:.3f} m"
                )
            maximum_return_error = float(
                self.get_parameter("maximum_agv_return_error_m").value
            )
            if self._current_agv_displacement > maximum_return_error:
                failures.append(
                    "AGV did not return near its start; final displacement "
                    f"{self._current_agv_displacement:.3f} m exceeds "
                    f"{maximum_return_error:.3f} m"
                )

        if bool(self.get_parameter("require_commander_result").value):
            minimum_results = int(
                self.get_parameter("minimum_commander_results").value
            )
            successful_results = sum(
                result.startswith("SUCCESS:") for result in self._commander_results
            )
            if successful_results < minimum_results:
                failures.append(
                    f"only {successful_results}/{minimum_results} required Commander "
                    "commands succeeded"
                )
            failed_results = [
                result for result in self._commander_results
                if result.startswith("FAILED:")
            ]
            if failed_results:
                failures.append(f"Commander reported failures: {failed_results}")

        if bool(self.get_parameter("require_box_pose").value):
            box_moved = any(
                event.get("state") == "robot_place"
                and event.get("status") == "SUCCESS"
                and event.get("details", {}).get("box_moved_in_gazebo") is True
                for event in self._events
            )
            if not box_moved:
                failures.append("Gazebo box placement was not confirmed")

        summary = [
            f"run_id={self._active_run_id}",
            f"events={len(self._events)}",
            f"agv_max_displacement_m={self._maximum_agv_displacement:.3f}",
            f"agv_return_error_m={self._current_agv_displacement:.3f}",
            f"commander_results={len(self._commander_results)}",
        ]
        return not failures, summary + failures

    def _event(self, message: String) -> None:
        event = self._decode(message.data)
        if event is not None and event.get("run_id") == self._active_run_id:
            self._events.append(event)

    def _state(self, message: String) -> None:
        state = self._decode(message.data)
        if state is None or state.get("run_id") != self._active_run_id:
            return
        if state.get("state") in ("receipt_complete", "failed"):
            self._terminal = state

    def _commander(self, message: String) -> None:
        if self._active_run_id:
            self._commander_results.append(message.data)

    def _odometry(self, message: Odometry) -> None:
        position = (
            message.pose.pose.position.x,
            message.pose.pose.position.y,
        )
        if self._odom_origin is None:
            self._odom_origin = position
            return
        displacement = math.hypot(
            position[0] - self._odom_origin[0],
            position[1] - self._odom_origin[1],
        )
        self._current_agv_displacement = displacement
        self._maximum_agv_displacement = max(self._maximum_agv_displacement, displacement)

    def _decode(self, text: str) -> Optional[Dict]:
        try:
            value = json.loads(text)
        except json.JSONDecodeError:
            return None
        return value if isinstance(value, dict) else None


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ReceivingAcceptanceTest()
    success = False
    messages = ["acceptance test stopped before evaluation"]
    try:
        try:
            success, messages = node.execute()
        except Exception as error:  # noqa: BLE001 - test process boundary
            messages = [f"acceptance test raised {type(error).__name__}: {error}"]
        for message in messages:
            if success:
                node.get_logger().info(message)
            else:
                node.get_logger().error(message)
    finally:
        node.destroy_node()
        rclpy.shutdown()
    sys.exit(0 if success else 1)
