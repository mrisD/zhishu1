"""Close both simulated grippers and attach the crate as an MTC grasp."""

import re
import subprocess

from action_msgs.msg import GoalStatus
from builtin_interfaces.msg import Duration as DurationMsg
from control_msgs.action import FollowJointTrajectory
import rclpy
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectoryPoint


WORLD_NAME = "admittance_box"
ROBOT_MODEL_NAME = "zhishu_robot"


def find_model_entity_id(pose_output: str, model_name: str) -> int | None:
    """Extract a model entity id from Gazebo's pose/info text output."""
    pattern = rf'pose\s*\{{\s*name:\s*"{re.escape(model_name)}"\s*id:\s*(\d+)'
    match = re.search(pattern, pose_output, flags=re.DOTALL)
    return int(match.group(1)) if match else None


def build_attachment_request(entity_id: int) -> str:
    """Build the EntityPlugin request for the MTC-style grasp attachment."""
    inner_xml = (
        "<parent_link>L_hand_link</parent_link>"
        "<child_model>carry_box</child_model>"
        "<child_link>box_link</child_link>"
        "<detach_topic>/admittance_box/detach</detach_topic>"
        "<attach_topic>/admittance_box/attach</attach_topic>"
        "<output_topic>/admittance_box/attached</output_topic>"
    )
    return (
        f"entity: {{id: {entity_id}, type: MODEL}}, plugins: [{{"
        'name: "gz::sim::systems::DetachableJoint", '
        'filename: "gz-sim-detachable-joint-system", '
        f'innerxml: "{inner_xml}"'
        "}]"
    )


class GraspCoordinator(Node):
    """Send time-aligned gripper trajectories and secure the grasped crate."""

    def __init__(self):
        super().__init__("admittance_grasp_coordinator")
        self.declare_parameter("gripper_position", 0.055)
        self.declare_parameter("motion_duration", 4.0)
        self.declare_parameter("start_delay", 0.5)
        self.declare_parameter("attach_box", True)
        self.left_client = ActionClient(
            self,
            FollowJointTrajectory,
            "/left_gripper_controller/follow_joint_trajectory",
        )
        self.right_client = ActionClient(
            self,
            FollowJointTrajectory,
            "/right_gripper_controller/follow_joint_trajectory",
        )

    @staticmethod
    def _duration_message(seconds: float) -> DurationMsg:
        return Duration(seconds=seconds).to_msg()

    def _goal(self, joint_name: str, position: float, duration: float, start_time):
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.header.stamp = start_time
        goal.trajectory.joint_names = [joint_name]
        point = JointTrajectoryPoint()
        point.positions = [position]
        point.time_from_start = self._duration_message(duration)
        goal.trajectory.points = [point]
        return goal

    def _model_entity_id(self) -> int | None:
        command = [
            "gz",
            "topic",
            "-e",
            "-t",
            f"/world/{WORLD_NAME}/pose/info",
        ]
        process = subprocess.Popen(
            command,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
        )
        try:
            stdout, stderr = process.communicate(timeout=2.0)
        except subprocess.TimeoutExpired:
            process.terminate()
            stdout, stderr = process.communicate(timeout=2.0)

        entity_id = find_model_entity_id(stdout, ROBOT_MODEL_NAME)
        if entity_id is None:
            self.get_logger().error(
                "Could not find the robot model entity in Gazebo pose/info: "
                + stderr.strip()
            )
        return entity_id

    def _attach_box(self) -> bool:
        entity_id = self._model_entity_id()
        if entity_id is None:
            return False

        command = [
            "gz",
            "service",
            "-s",
            f"/world/{WORLD_NAME}/entity/system/add",
            "--reqtype",
            "gz.msgs.EntityPlugin_V",
            "--reptype",
            "gz.msgs.Boolean",
            "--timeout",
            "20000",
            "--req",
            build_attachment_request(entity_id),
        ]
        try:
            result = subprocess.run(
                command,
                capture_output=True,
                text=True,
                timeout=25.0,
                check=False,
            )
        except (OSError, subprocess.TimeoutExpired) as error:
            self.get_logger().error(f"Failed to invoke Gazebo attachment service: {error}")
            return False

        if result.returncode != 0 or "data: true" not in result.stdout:
            details = (result.stderr or result.stdout).strip()
            self.get_logger().error(f"Gazebo rejected the box attachment: {details}")
            return False

        self.get_logger().info(
            "Attached carry_box to L_hand_link using a Gazebo detachable joint"
        )
        return True

    def run(self) -> bool:
        position = float(self.get_parameter("gripper_position").value)
        duration = float(self.get_parameter("motion_duration").value)
        start_delay = float(self.get_parameter("start_delay").value)
        attach_box = bool(self.get_parameter("attach_box").value)
        if not 0.0 <= position <= 0.06:
            self.get_logger().error("gripper_position must be within [0.0, 0.06] m")
            return False
        if duration <= 0.0 or start_delay < 0.0:
            self.get_logger().error(
                "motion_duration must be positive and start_delay non-negative"
            )
            return False

        if not self.left_client.wait_for_server(timeout_sec=30.0):
            self.get_logger().error("left gripper action server is unavailable")
            return False
        if not self.right_client.wait_for_server(timeout_sec=30.0):
            self.get_logger().error("right gripper action server is unavailable")
            return False

        # A use_sim_time clock starts at zero until the first /clock sample arrives.
        # Building a stamped trajectory before that makes it stale immediately.
        for _ in range(50):
            if self.get_clock().now().nanoseconds > 0:
                break
            rclpy.spin_once(self, timeout_sec=0.1)
        if self.get_clock().now().nanoseconds <= 0:
            self.get_logger().error("simulation clock did not start")
            return False

        start_time = (self.get_clock().now() + Duration(seconds=start_delay)).to_msg()
        left_future = self.left_client.send_goal_async(
            self._goal(
                "left_gripper_left_finger_joint", position, duration, start_time
            )
        )
        right_future = self.right_client.send_goal_async(
            self._goal("gripper_left_finger_joint", position, duration, start_time)
        )
        timeout = start_delay + duration + 15.0
        rclpy.spin_until_future_complete(self, left_future, timeout_sec=timeout)
        rclpy.spin_until_future_complete(self, right_future, timeout_sec=timeout)
        if not left_future.done() or not right_future.done():
            self.get_logger().error("timed out while sending synchronized grasp goals")
            return False

        goal_handles = [left_future.result(), right_future.result()]
        if not all(handle.accepted for handle in goal_handles):
            self.get_logger().error("one or both gripper controllers rejected the goal")
            return False

        result_futures = [handle.get_result_async() for handle in goal_handles]
        for future in result_futures:
            rclpy.spin_until_future_complete(self, future, timeout_sec=timeout)
        if not all(future.done() for future in result_futures):
            self.get_logger().error("synchronized grasp timed out")
            return False

        results = [future.result() for future in result_futures]
        if not all(
            result.status == GoalStatus.STATUS_SUCCEEDED
            and result.result.error_code == FollowJointTrajectory.Result.SUCCESSFUL
            for result in results
        ):
            self.get_logger().error("one or both synchronized grasp trajectories failed")
            return False

        self.get_logger().info(
            f"Both grippers reached {position:.3f} m synchronously in {duration:.1f} s"
        )
        return not attach_box or self._attach_box()


def main(args=None):
    rclpy.init(args=args)
    node = GraspCoordinator()
    try:
        return 0 if node.run() else 1
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
