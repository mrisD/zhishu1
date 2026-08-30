"""Coordinate the simulated LES, scanner, Gazebo AGV and MoveIt robot."""

import json
import math
import threading
import time
import uuid
from typing import Any, Dict, Optional, Tuple

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from ros_gz_interfaces.msg import Entity
from ros_gz_interfaces.srv import SetEntityPose
from std_msgs.msg import String
from std_srvs.srv import Trigger

from .flow import FlowState, ReceivingFlow, STATE_LABELS


class ReceivingOrchestrator(Node):
    def __init__(self) -> None:
        super().__init__("receiving_orchestrator")
        self.declare_parameter("auto_start", True)
        self.declare_parameter("startup_delay_seconds", 12.0)
        self.declare_parameter("max_barcode_attempts", 3)
        self.declare_parameter("service_timeout_seconds", 10.0)
        self.declare_parameter("commander_timeout_seconds", 60.0)
        self.declare_parameter("simulate_agv_motion", True)
        self.declare_parameter("agv_speed_mps", 0.30)
        self.declare_parameter("agv_travel_distance_m", 1.35)
        self.declare_parameter("agv_motion_timeout_seconds", 15.0)
        self.declare_parameter("simulate_robot_motion", True)
        self.declare_parameter("require_robot_motion_success", False)
        self.declare_parameter("move_box_in_gazebo", True)
        self.declare_parameter("gazebo_world", "receiving_demo")
        self.declare_parameter("box_entity", "received_part_box")
        self.declare_parameter("box_source_pose", [0.55, -0.85, 0.24])
        self.declare_parameter("box_destination_pose", [1.05, 0.0, 0.50])
        self.declare_parameter("agv_entity", "receiving_agv")
        self.declare_parameter("agv_home_pose", [2.40, 0.0, 0.0])

        maximum_attempts = int(self.get_parameter("max_barcode_attempts").value)
        self._flow = ReceivingFlow(maximum_attempts)
        self._run_lock = threading.Lock()
        self._cancel = threading.Event()
        self._worker: Optional[threading.Thread] = None
        self._run_id = ""
        self._event_sequence = 0

        state_qos = QoSProfile(depth=1)
        state_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        state_qos.reliability = ReliabilityPolicy.RELIABLE
        self._state_publisher = self.create_publisher(
            String, "/receiving_sim/state", state_qos
        )
        self._event_publisher = self.create_publisher(
            String, "/receiving_sim/events", 100
        )
        self._agv_publisher = self.create_publisher(
            Twist, "/receiving_agv/cmd_vel", 10
        )
        self._arm_publisher = self.create_publisher(
            String, "/zhishu_robot_commander/both_arms/named_target", 10
        )
        self._gripper_publisher = self.create_publisher(
            String, "/zhishu_robot_commander/grippers/named_command", 10
        )

        self._odom_lock = threading.Lock()
        self._latest_odom: Optional[Tuple[float, float]] = None
        self.create_subscription(
            Odometry, "/receiving_agv/odometry", self._odom_callback, 20
        )

        self._commander_condition = threading.Condition()
        self._commander_sequence = 0
        self._commander_status = ""
        self.create_subscription(
            String,
            "/zhishu_robot_commander/status",
            self._commander_status_callback,
            20,
        )

        # rclpy.node.Node owns ``_clients`` as an internal list.  Keep the
        # workflow's named clients separate so create_client() can register
        # each client with the node on ROS 2 Jazzy.
        self._service_clients = {
            "les_receive": self.create_client(
                Trigger, "/les_mock/receive_and_inspect"
            ),
            "les_dispatch": self.create_client(Trigger, "/les_mock/dispatch_task"),
            "les_not_pushed": self.create_client(
                Trigger, "/les_mock/feedback_not_pushed"
            ),
            "les_complete": self.create_client(Trigger, "/les_mock/complete_task"),
            "les_agv_return": self.create_client(
                Trigger, "/les_mock/notify_agv_return"
            ),
            "scan_part": self.create_client(
                Trigger, "/scanner_mock/scan_part_box"
            ),
            "scan_rack": self.create_client(Trigger, "/scanner_mock/scan_rack_sku"),
            "scan_reset": self.create_client(Trigger, "/scanner_mock/reset"),
        }
        world = str(self.get_parameter("gazebo_world").value)
        self._set_pose_client = self.create_client(
            SetEntityPose, f"/world/{world}/set_pose"
        )

        self.create_service(Trigger, "/receiving_sim/start", self._start_service)
        self.create_service(Trigger, "/receiving_sim/reset", self._reset_service)
        self.create_service(Trigger, "/receiving_sim/stop", self._stop_service)
        self._publish_state("IDLE", {})

        if bool(self.get_parameter("auto_start").value):
            delay = float(self.get_parameter("startup_delay_seconds").value)
            if delay < 0.0 or delay > 300.0:
                raise ValueError("startup_delay_seconds must be in [0, 300]")
            self._auto_start_timer = self.create_timer(delay, self._auto_start)
        else:
            self._auto_start_timer = None
        self.get_logger().info("Receiving workflow orchestrator is ready")

    def _auto_start(self) -> None:
        if self._auto_start_timer is not None:
            self._auto_start_timer.cancel()
        self._start_run()

    def _start_service(self, _request, response):
        started = self._start_run()
        response.success = started
        response.message = self._run_id if started else "a receiving run is already active"
        return response

    def _reset_service(self, _request, response):
        with self._run_lock:
            if self._worker is not None and self._worker.is_alive():
                response.success = False
                response.message = "stop the active receiving run before reset"
                return response
            self._flow.reset()
            self._run_id = ""
            self._event_sequence = 0
            self._publish_state("IDLE", {})
        response.success = True
        response.message = "receiving flow reset"
        return response

    def _stop_service(self, _request, response):
        self._cancel.set()
        self._stop_agv()
        response.success = True
        response.message = "stop requested"
        return response

    def _start_run(self) -> bool:
        with self._run_lock:
            if self._worker is not None and self._worker.is_alive():
                return False
            self._flow.reset()
            self._cancel.clear()
            self._run_id = f"receiving-{uuid.uuid4().hex[:8]}"
            self._event_sequence = 0
            self._worker = threading.Thread(
                target=self._run, name="receiving-workflow", daemon=True
            )
            self._worker.start()
            return True

    def _run(self) -> None:
        try:
            self._call_trigger("scan_reset")
            self._reset_gazebo_entities()

            self._enter_next()
            self._run_trigger_step("les_receive")

            self._enter_next()
            self._run_trigger_step("les_dispatch")

            self._enter_next()
            self._run_action_step(lambda: self._drive_agv(inbound=True))

            self._enter_next()
            self._run_action_step(self._robot_pick)

            self._enter_next()
            while self._flow.state == FlowState.PART_BARCODE_SCAN:
                payload = self._run_trigger_step("scan_part", complete=False)
                matched = bool(payload.get("matched", False))
                if matched:
                    self._publish_state("SUCCESS", payload)
                    self._flow.barcode_result(True)
                    break
                self._publish_state("RETRY", payload)
                self._flow.barcode_result(False)
                if self._flow.state == FlowState.FAILED:
                    raise RuntimeError(self._flow.failure_reason)
                self._run_trigger_step("les_not_pushed")
                self._enter_next()

            rack_scan = self._run_trigger_step("scan_rack", complete=False)
            if not bool(rack_scan.get("matched", False)):
                raise RuntimeError("rack SKU barcode did not match")
            self._publish_state("SUCCESS", rack_scan)

            self._enter_next()
            self._run_action_step(self._robot_place)

            self._enter_next()
            self._run_trigger_step("les_complete")

            self._enter_next()
            self._publish_state("RUNNING", {"phase": "notify_agv"})
            notification = self._call_trigger("les_agv_return")
            self._drive_agv(inbound=False)
            self._publish_state("SUCCESS", notification)

            self._enter_next()
            self._publish_state("SUCCESS", {"result": "receipt_completed"})
            self.get_logger().info("Receiving workflow completed successfully")
        except Exception as error:  # noqa: BLE001 - terminal workflow boundary
            self._stop_agv()
            reason = str(error) or type(error).__name__
            if self._flow.state != FlowState.FAILED:
                self._flow.fail(reason)
            self._publish_state("FAILED", {"reason": reason})
            self.get_logger().error(f"Receiving workflow failed: {reason}")

    def _enter_next(self) -> None:
        self._check_cancelled()
        self._flow.advance()

    def _run_trigger_step(
        self, client_name: str, complete: bool = True
    ) -> Dict[str, Any]:
        self._publish_state("RUNNING", {"service": client_name})
        payload = self._call_trigger(client_name)
        if complete:
            self._publish_state("SUCCESS", payload)
        return payload

    def _run_action_step(self, action) -> None:
        self._publish_state("RUNNING", {})
        details = action() or {}
        self._publish_state("SUCCESS", details)

    def _call_trigger(self, client_name: str) -> Dict[str, Any]:
        client = self._service_clients[client_name]
        timeout = float(self.get_parameter("service_timeout_seconds").value)
        deadline = time.monotonic() + timeout
        while not client.wait_for_service(timeout_sec=0.2):
            self._check_cancelled()
            if time.monotonic() >= deadline:
                raise TimeoutError(f"service {client.srv_name} is unavailable")
        future = client.call_async(Trigger.Request())
        while not future.done():
            self._check_cancelled()
            if time.monotonic() >= deadline:
                raise TimeoutError(f"service {client.srv_name} timed out")
            time.sleep(0.02)
        response = future.result()
        if response is None or not response.success:
            message = "no response" if response is None else response.message
            raise RuntimeError(f"service {client.srv_name} failed: {message}")
        try:
            payload = json.loads(response.message) if response.message else {}
        except json.JSONDecodeError as error:
            raise RuntimeError(
                f"service {client.srv_name} returned invalid JSON"
            ) from error
        if not isinstance(payload, dict):
            raise RuntimeError(f"service {client.srv_name} must return a JSON object")
        return payload

    def _robot_pick(self) -> Dict[str, Any]:
        if not bool(self.get_parameter("simulate_robot_motion").value):
            return {"motion": "skipped by parameter"}
        commands = [
            ("arm", "both_arms_crate_pre"),
            ("gripper", "open_both"),
            ("gripper", "close_both"),
            ("arm", "both_arms_pose1"),
        ]
        results = self._run_robot_commands(commands)
        return {"motion": "pick simulated", "commands": results}

    def _robot_place(self) -> Dict[str, Any]:
        if not bool(self.get_parameter("simulate_robot_motion").value):
            box_moved = self._set_box_pose(destination=True)
            return {
                "motion": "skipped by parameter",
                "box_moved_in_gazebo": box_moved,
            }
        placement_commands = [
            ("arm", "both_arms_crate_safe"),
            ("gripper", "open_both"),
        ]
        results = self._run_robot_commands(placement_commands)
        box_moved = self._set_box_pose(destination=True)
        results.extend(self._run_robot_commands([("arm", "both_arms_home")]))
        return {
            "motion": "place simulated",
            "commands": results,
            "box_moved_in_gazebo": box_moved,
        }

    def _run_robot_commands(self, commands) -> list:
        require_success = bool(
            self.get_parameter("require_robot_motion_success").value
        )
        results = []
        for command_type, value in commands:
            success, status = self._send_robot_command(command_type, value)
            results.append({"command": value, "success": success, "status": status})
            if not success and require_success:
                raise RuntimeError(f"robot command {value} failed: {status}")
            if not success:
                self.get_logger().warning(
                    f"Robot motion {value} was unavailable; continuing virtual flow"
                )
        return results

    def _send_robot_command(self, command_type: str, value: str) -> Tuple[bool, str]:
        publisher = self._arm_publisher if command_type == "arm" else (
            self._gripper_publisher
        )
        expected = "~/both_arms/named_target" if command_type == "arm" else (
            f"grippers/named_command:{value}"
        )
        timeout = float(self.get_parameter("commander_timeout_seconds").value)
        deadline = time.monotonic() + timeout
        while publisher.get_subscription_count() == 0:
            self._check_cancelled()
            if time.monotonic() >= deadline:
                return False, "Commander subscriber unavailable"
            time.sleep(0.1)
        with self._commander_condition:
            baseline = self._commander_sequence
        message = String()
        message.data = value
        publisher.publish(message)
        with self._commander_condition:
            while self._commander_sequence <= baseline:
                self._check_cancelled()
                remaining = deadline - time.monotonic()
                if remaining <= 0.0:
                    return False, "Commander status timeout"
                self._commander_condition.wait(timeout=min(remaining, 0.2))
            status = self._commander_status
        if expected not in status:
            return False, f"unexpected Commander status: {status}"
        return status.startswith("SUCCESS:"), status

    def _drive_agv(self, inbound: bool) -> Dict[str, Any]:
        if not bool(self.get_parameter("simulate_agv_motion").value):
            return {"agv_motion": "skipped by parameter"}
        speed = float(self.get_parameter("agv_speed_mps").value)
        distance = float(self.get_parameter("agv_travel_distance_m").value)
        timeout = float(self.get_parameter("agv_motion_timeout_seconds").value)
        if speed <= 0.0 or speed > 1.5 or distance <= 0.0 or distance > 10.0:
            raise ValueError("AGV speed or travel distance is outside its safe range")
        with self._odom_lock:
            start = self._latest_odom
        direction = 1.0 if inbound else -1.0
        command = Twist()
        command.linear.x = direction * speed
        started = time.monotonic()
        expected_duration = distance / speed
        odometry_used = start is not None
        travelled = 0.0
        try:
            while True:
                self._check_cancelled()
                self._agv_publisher.publish(command)
                with self._odom_lock:
                    current = self._latest_odom
                if start is not None and current is not None:
                    travelled = math.hypot(current[0] - start[0], current[1] - start[1])
                    if travelled >= distance:
                        break
                elif time.monotonic() - started >= expected_duration:
                    break
                if time.monotonic() - started >= timeout:
                    raise TimeoutError("Gazebo AGV did not reach its travel target")
                time.sleep(0.05)
        finally:
            self._stop_agv()
        return {
            "direction": "to_workstation" if inbound else "to_storage",
            "target_distance_m": distance,
            "measured_distance_m": round(travelled, 3),
            "odometry_used": odometry_used,
        }

    def _stop_agv(self) -> None:
        self._agv_publisher.publish(Twist())

    def _reset_gazebo_entities(self) -> None:
        if not bool(self.get_parameter("move_box_in_gazebo").value):
            return
        self._set_entity_pose(
            str(self.get_parameter("agv_entity").value),
            list(self.get_parameter("agv_home_pose").value),
            yaw=math.pi,
            required=False,
        )
        self._set_entity_pose(
            str(self.get_parameter("box_entity").value),
            list(self.get_parameter("box_source_pose").value),
            required=False,
        )

    def _set_box_pose(self, destination: bool) -> bool:
        if not bool(self.get_parameter("move_box_in_gazebo").value):
            return False
        parameter = "box_destination_pose" if destination else "box_source_pose"
        return self._set_entity_pose(
            str(self.get_parameter("box_entity").value),
            list(self.get_parameter(parameter).value),
            required=False,
        )

    def _set_entity_pose(
        self, name: str, xyz: list, yaw: float = 0.0, required: bool = False
    ) -> bool:
        if len(xyz) != 3:
            raise ValueError("entity pose parameter must contain x, y and z")
        timeout = float(self.get_parameter("service_timeout_seconds").value)
        if not self._set_pose_client.wait_for_service(timeout_sec=min(timeout, 3.0)):
            if required:
                raise TimeoutError("Gazebo set_pose service is unavailable")
            self.get_logger().warning(
                "Gazebo set_pose service unavailable; continuing without object teleport"
            )
            return False
        request = SetEntityPose.Request()
        request.entity.name = name
        request.entity.type = Entity.MODEL
        request.pose.position.x = float(xyz[0])
        request.pose.position.y = float(xyz[1])
        request.pose.position.z = float(xyz[2])
        request.pose.orientation.z = math.sin(yaw / 2.0)
        request.pose.orientation.w = math.cos(yaw / 2.0)
        future = self._set_pose_client.call_async(request)
        deadline = time.monotonic() + timeout
        while not future.done():
            self._check_cancelled()
            if time.monotonic() >= deadline:
                if required:
                    raise TimeoutError(f"Gazebo set_pose timed out for {name}")
                return False
            time.sleep(0.02)
        response = future.result()
        success = response is not None and response.success
        if not success and required:
            raise RuntimeError(f"Gazebo rejected set_pose for {name}")
        return success

    def _odom_callback(self, message: Odometry) -> None:
        with self._odom_lock:
            self._latest_odom = (
                message.pose.pose.position.x,
                message.pose.pose.position.y,
            )

    def _commander_status_callback(self, message: String) -> None:
        with self._commander_condition:
            self._commander_sequence += 1
            self._commander_status = message.data
            self._commander_condition.notify_all()

    def _publish_state(self, status: str, details: Dict[str, Any]) -> None:
        self._event_sequence += 1
        snapshot = self._flow.snapshot()
        event = {
            "run_id": self._run_id,
            "sequence": self._event_sequence,
            "state": snapshot.state.value,
            "label": snapshot.label,
            "status": status,
            "barcode_attempt": snapshot.barcode_attempt,
            "max_barcode_attempts": snapshot.max_barcode_attempts,
            "details": details,
            "stamp_seconds": round(self.get_clock().now().nanoseconds / 1e9, 3),
        }
        message = String()
        message.data = json.dumps(event, ensure_ascii=False, separators=(",", ":"))
        self._state_publisher.publish(message)
        self._event_publisher.publish(message)
        self.get_logger().info(
            f"[{status}] {STATE_LABELS[snapshot.state]} ({snapshot.state.value})"
        )

    def _check_cancelled(self) -> None:
        if self._cancel.is_set() or not rclpy.ok():
            raise RuntimeError("receiving run was cancelled")

    def stop(self) -> None:
        self._cancel.set()
        self._stop_agv()
        if self._worker is not None and self._worker.is_alive():
            self._worker.join(timeout=5.0)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ReceivingOrchestrator()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        node.stop()
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()
