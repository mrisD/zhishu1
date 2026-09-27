#!/usr/bin/env python3
"""Mirror reconstructed RGB-D voxels into the MoveIt planning scene."""

import rclpy
from geometry_msgs.msg import Pose
from moveit_msgs.msg import CollisionObject, PlanningScene
from moveit_msgs.srv import ApplyPlanningScene
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from shape_msgs.msg import SolidPrimitive


class PlanningSceneObstaclesNode(Node):
    def __init__(self):
        super().__init__("planning_scene_obstacles")
        self.declare_parameter("input_topic", "/bottle_grasp/reconstructed_obstacles")
        self.declare_parameter("collision_object_id", "rgbd_reconstructed_obstacles")
        self.declare_parameter("voxel_size", 0.03)
        self.declare_parameter("padding", 0.01)
        self.declare_parameter("maximum_collision_voxels", 2500)
        self._client = self.create_client(ApplyPlanningScene, "/apply_planning_scene")
        self._request_in_flight = False
        self._last_wait_warning_ns = 0
        self.create_subscription(
            PointCloud2,
            self.get_parameter("input_topic").value,
            self._on_cloud,
            qos_profile_sensor_data,
        )
        self.get_logger().info("PlanningScene RGB-D voxel fallback ready")

    def _on_cloud(self, message):
        if self._request_in_flight:
            return
        if not self._client.service_is_ready():
            now_ns = self.get_clock().now().nanoseconds
            if now_ns - self._last_wait_warning_ns > int(2e9):
                self.get_logger().warning("Waiting for /apply_planning_scene")
                self._last_wait_warning_ns = now_ns
            return

        points = point_cloud2.read_points_numpy(
            message, field_names=["x", "y", "z"], skip_nans=True
        ).reshape((-1, 3))
        limit = int(self.get_parameter("maximum_collision_voxels").value)
        if len(points) > limit:
            step = max(1, (len(points) + limit - 1) // limit)
            points = points[::step][:limit]

        collision = CollisionObject()
        collision.header = message.header
        collision.id = self.get_parameter("collision_object_id").value
        if len(points):
            side = (
                float(self.get_parameter("voxel_size").value)
                + 2.0 * float(self.get_parameter("padding").value)
            )
            primitive = SolidPrimitive()
            primitive.type = SolidPrimitive.BOX
            primitive.dimensions = [side, side, side]
            collision.primitives = [primitive for _ in range(len(points))]
            collision.primitive_poses = []
            for x, y, z in points:
                pose = Pose()
                pose.position.x = float(x)
                pose.position.y = float(y)
                pose.position.z = float(z)
                pose.orientation.w = 1.0
                collision.primitive_poses.append(pose)
            collision.operation = CollisionObject.ADD
        else:
            collision.operation = CollisionObject.REMOVE

        scene = PlanningScene()
        scene.is_diff = True
        scene.robot_state.is_diff = True
        scene.world.collision_objects = [collision]
        request = ApplyPlanningScene.Request()
        request.scene = scene
        self._request_in_flight = True
        future = self._client.call_async(request)
        future.add_done_callback(self._on_applied)

    def _on_applied(self, future):
        self._request_in_flight = False
        try:
            if not future.result().success:
                self.get_logger().warning("MoveIt rejected reconstructed obstacle update")
        except Exception as error:  # service failure should not stop perception
            self.get_logger().warning(f"PlanningScene update failed: {error}")


def main(args=None):
    rclpy.init(args=args)
    node = PlanningSceneObstaclesNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
        except KeyboardInterrupt:
            pass


if __name__ == "__main__":
    main()
