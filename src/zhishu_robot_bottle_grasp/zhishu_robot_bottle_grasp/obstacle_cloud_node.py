#!/usr/bin/env python3
"""Crop Gemini Pro points into a collision-only cloud for MoveIt OctoMap."""

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header
from tf2_ros import Buffer, TransformException, TransformListener

from .geometry import quaternion_matrix, transform_points
from .voxel_map import TemporalVoxelMap


class ObstacleCloudNode(Node):
    def __init__(self):
        super().__init__("obstacle_cloud")
        defaults = {
            "input_topic": "/camera_head/depth_registered/points",
            "output_topic": "/bottle_grasp/obstacle_points",
            "model_topic": "/bottle_grasp/reconstructed_obstacles",
            "bottle_pose_topic": "/bottle_grasp/grasp_pose",
            "target_frame": "base_link",
            "publish_period": 0.25,
            "point_stride": 6,
            "min_x": 0.12,
            "max_x": 1.10,
            "min_y": -0.62,
            "max_y": 0.62,
            "min_z": 1.145,
            "max_z": 1.85,
            "bottle_exclusion_radius": 0.11,
            "bottle_exclusion_half_height": 0.25,
            "voxel_size": 0.03,
            "minimum_voxel_hits": 2,
            "voxel_decay_seconds": 2.0,
            "maximum_voxels": 50000,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)

        self._tf_buffer = Buffer(cache_time=Duration(seconds=5.0))
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self._latest_bottle = None
        self._last_publish_ns = -1
        self._voxel_map = TemporalVoxelMap(
            voxel_size=self.get_parameter("voxel_size").value,
            minimum_hits=self.get_parameter("minimum_voxel_hits").value,
            decay_seconds=self.get_parameter("voxel_decay_seconds").value,
            max_voxels=self.get_parameter("maximum_voxels").value,
        )
        self._publisher = self.create_publisher(
            PointCloud2, self.get_parameter("output_topic").value, qos_profile_sensor_data
        )
        self._model_publisher = self.create_publisher(
            PointCloud2, self.get_parameter("model_topic").value, qos_profile_sensor_data
        )
        self.create_subscription(
            PoseStamped,
            self.get_parameter("bottle_pose_topic").value,
            self._on_bottle,
            10,
        )
        self.create_subscription(
            PointCloud2,
            self.get_parameter("input_topic").value,
            self._on_cloud,
            qos_profile_sensor_data,
        )
        self.get_logger().info(
            "Temporal voxel reconstruction ready; support plane and tracked bottle are excluded"
        )

    def _on_bottle(self, message):
        self._latest_bottle = np.array(
            [message.pose.position.x, message.pose.position.y, message.pose.position.z],
            dtype=np.float64,
        )

    def _on_cloud(self, message):
        stamp = Time.from_msg(message.header.stamp)
        period_ns = int(float(self.get_parameter("publish_period").value) * 1e9)
        if self._last_publish_ns >= 0 and stamp.nanoseconds - self._last_publish_ns < period_ns:
            return
        self._last_publish_ns = stamp.nanoseconds

        try:
            transform = self._tf_buffer.lookup_transform(
                self.get_parameter("target_frame").value,
                message.header.frame_id,
                stamp,
                timeout=Duration(seconds=0.08),
            )
        except TransformException:
            # Gazebo may publish the render stamp a few milliseconds ahead of
            # joint-state TF.  The head camera is stationary during grasping,
            # so the latest transform is the correct bounded fallback.
            try:
                transform = self._tf_buffer.lookup_transform(
                    self.get_parameter("target_frame").value,
                    message.header.frame_id,
                    Time(),
                    timeout=Duration(seconds=0.08),
                )
            except TransformException as error:
                self.get_logger().warning(
                    f"Point-cloud TF unavailable: {error}", throttle_duration_sec=2.0
                )
                return

        points = point_cloud2.read_points_numpy(
            message, field_names=["x", "y", "z"], skip_nans=True
        ).reshape(-1, 3)
        stride = max(1, int(self.get_parameter("point_stride").value))
        points = points[::stride].astype(np.float64, copy=False)
        points = points[np.isfinite(points).all(axis=1)]
        if not len(points):
            return
        translation = np.array([
            transform.transform.translation.x,
            transform.transform.translation.y,
            transform.transform.translation.z,
        ])
        quaternion = np.array([
            transform.transform.rotation.x,
            transform.transform.rotation.y,
            transform.transform.rotation.z,
            transform.transform.rotation.w,
        ])
        base_points = transform_points(points, translation, quaternion)
        keep = (
            (base_points[:, 0] >= self.get_parameter("min_x").value)
            & (base_points[:, 0] <= self.get_parameter("max_x").value)
            & (base_points[:, 1] >= self.get_parameter("min_y").value)
            & (base_points[:, 1] <= self.get_parameter("max_y").value)
            & (base_points[:, 2] >= self.get_parameter("min_z").value)
            & (base_points[:, 2] <= self.get_parameter("max_z").value)
        )
        if self._latest_bottle is not None:
            delta = base_points - self._latest_bottle
            radius = float(self.get_parameter("bottle_exclusion_radius").value)
            half_height = float(self.get_parameter("bottle_exclusion_half_height").value)
            bottle = (
                np.hypot(delta[:, 0], delta[:, 1]) < radius
            ) & (np.abs(delta[:, 2]) < half_height)
            keep &= ~bottle

        stamp_seconds = stamp.nanoseconds * 1e-9
        reconstructed_base = self._voxel_map.update(base_points[keep], stamp_seconds)

        # MoveIt's PointCloudOctomapUpdater uses the message frame as its ray
        # origin.  Convert the fused base-frame voxels back into the current
        # optical frame so free-space clearing remains geometrically correct.
        rotation = quaternion_matrix(*quaternion)
        reconstructed_optical = (reconstructed_base - translation) @ rotation

        header = Header()
        header.stamp = message.header.stamp
        header.frame_id = message.header.frame_id
        output = point_cloud2.create_cloud_xyz32(
            header, reconstructed_optical.astype(np.float32)
        )
        self._publisher.publish(output)

        model_header = Header()
        model_header.stamp = message.header.stamp
        model_header.frame_id = self.get_parameter("target_frame").value
        model = point_cloud2.create_cloud_xyz32(
            model_header, reconstructed_base.astype(np.float32)
        )
        self._model_publisher.publish(model)


def main(args=None):
    rclpy.init(args=args)
    node = ObstacleCloudNode()
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
