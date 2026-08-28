"""Activate the admittance-demo controllers and fail if the switch is rejected."""

from controller_manager_msgs.srv import SwitchController
import rclpy
from rclpy.node import Node


CONTROLLERS = [
    "joint_state_broadcaster",
    "body_posture_controller",
    "left_arm_controller",
    "right_arm_controller",
    "left_gripper_controller",
    "right_gripper_controller",
]


def main(args=None):
    rclpy.init(args=args)
    node = Node("admittance_controller_activator")
    exit_code = 1
    try:
        client = node.create_client(
            SwitchController, "/controller_manager/switch_controller"
        )
        if not client.wait_for_service(timeout_sec=60.0):
            node.get_logger().error("controller_manager switch service is unavailable")
            return exit_code

        request = SwitchController.Request()
        request.activate_controllers = CONTROLLERS
        request.deactivate_controllers = []
        request.strictness = 2  # controller_manager_msgs/SwitchController STRICT
        request.activate_asap = True
        request.timeout.sec = 10

        future = client.call_async(request)
        rclpy.spin_until_future_complete(node, future, timeout_sec=20.0)
        if not future.done():
            node.get_logger().error("controller activation timed out")
            return exit_code
        if future.exception() is not None:
            node.get_logger().error(f"controller activation failed: {future.exception()}")
            return exit_code
        if not future.result().ok:
            node.get_logger().error("controller_manager rejected controller activation")
            return exit_code

        node.get_logger().info("All admittance-demo controllers are active")
        exit_code = 0
        return exit_code
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
