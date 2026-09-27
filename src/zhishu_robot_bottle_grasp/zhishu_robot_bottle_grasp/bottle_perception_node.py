#!/usr/bin/env python3
"""Fuse YOLO instance masks with aligned Gemini Pro depth."""

import math
from threading import Lock

import cv2
import message_filters
import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import PoseStamped, TwistStamped
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import Header
from tf2_ros import Buffer, TransformException, TransformListener
from vision_msgs.msg import Detection3D, Detection3DArray, ObjectHypothesisWithPose
from visualization_msgs.msg import Marker

from .geometry import (
    CameraIntrinsics,
    depth_to_meters,
    estimate_bottle_geometry,
    project_masked_depth,
    transform_points,
)
from .tracker import AlphaBetaTracker
from .yolo_backend import ColorBottleBackend, YoloSegmentationBackend


class BottlePerceptionNode(Node):
    def __init__(self):
        super().__init__("bottle_perception")
        self._declare_parameters()
        self._bridge = CvBridge()
        self._camera_info = None
        self._camera_info_lock = Lock()
        self._tf_buffer = Buffer(cache_time=Duration(seconds=5.0))
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self._tracker = AlphaBetaTracker(
            self.get_parameter("tracker_alpha").value,
            self.get_parameter("tracker_beta").value,
            self.get_parameter("tracker_max_gap").value,
        )
        self._backend = self._make_backend()

        image_topic = self.get_parameter("image_topic").value
        depth_topic = self.get_parameter("depth_topic").value
        info_topic = self.get_parameter("camera_info_topic").value
        self.create_subscription(CameraInfo, info_topic, self._on_camera_info, qos_profile_sensor_data)
        self._image_sub = message_filters.Subscriber(
            self, Image, image_topic, qos_profile=qos_profile_sensor_data
        )
        self._depth_sub = message_filters.Subscriber(
            self, Image, depth_topic, qos_profile=qos_profile_sensor_data
        )
        queue_size = int(self.get_parameter("sync_queue_size").value)
        slop = float(self.get_parameter("sync_slop").value)
        self._sync = message_filters.ApproximateTimeSynchronizer(
            [self._image_sub, self._depth_sub], queue_size, slop
        )
        self._sync.registerCallback(self._on_rgbd)

        self._detection_pub = self.create_publisher(
            Detection3DArray, self.get_parameter("detection_topic").value, 10
        )
        self._grasp_pub = self.create_publisher(
            PoseStamped, self.get_parameter("grasp_pose_topic").value, 10
        )
        self._predicted_pub = self.create_publisher(
            PoseStamped, self.get_parameter("predicted_pose_topic").value, 10
        )
        self._twist_pub = self.create_publisher(
            TwistStamped, self.get_parameter("tracked_twist_topic").value, 10
        )
        self._marker_pub = self.create_publisher(Marker, "~/marker", 10)
        self._debug_pub = self.create_publisher(Image, "~/debug_image", qos_profile_sensor_data)
        self._last_dimension_warning = None
        self.get_logger().info(
            f"RGB-D bottle perception ready: backend={self.get_parameter('detector_backend').value}, "
            f"image={image_topic}, depth={depth_topic}"
        )

    def _declare_parameters(self):
        defaults = {
            "image_topic": "/camera_head/color/image_raw",
            "depth_topic": "/camera_head/depth/image_raw",
            "camera_info_topic": "/camera_head/color/camera_info",
            "target_frame": "base_link",
            "detector_backend": "yolo",
            "yolo_weights": "yolo26n-seg.pt",
            "yolo_device": "cpu",
            "confidence": 0.35,
            "minimum_mask_area": 300,
            "minimum_points": 80,
            "point_stride": 2,
            "minimum_depth": 0.10,
            "maximum_depth": 4.0,
            "mask_erode_pixels": 2,
            "table_ring_pixels": 28,
            "sync_queue_size": 8,
            "sync_slop": 0.07,
            "tracker_alpha": 0.65,
            "tracker_beta": 0.12,
            "tracker_max_gap": 0.8,
            "prediction_horizon": 0.45,
            "minimum_track_observations": 3,
            "detection_topic": "/bottle_grasp/detections_3d",
            "grasp_pose_topic": "/bottle_grasp/grasp_pose",
            "predicted_pose_topic": "/bottle_grasp/predicted_grasp_pose",
            "tracked_twist_topic": "/bottle_grasp/tracked_twist",
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)

    def _make_backend(self):
        name = str(self.get_parameter("detector_backend").value).lower()
        if name == "yolo":
            return YoloSegmentationBackend(
                self.get_parameter("yolo_weights").value,
                self.get_parameter("confidence").value,
                self.get_parameter("yolo_device").value,
            )
        if name == "color":
            self.get_logger().warning(
                "Using diagnostic color segmentation; this mode is only for camera/geometry tests"
            )
            return ColorBottleBackend(self.get_parameter("minimum_mask_area").value)
        raise ValueError(f"unsupported detector_backend: {name}")

    def _on_camera_info(self, message):
        with self._camera_info_lock:
            self._camera_info = message

    def _on_rgbd(self, image_message, depth_message):
        with self._camera_info_lock:
            camera_info = self._camera_info
        if camera_info is None:
            self.get_logger().warning("Waiting for color CameraInfo", throttle_duration_sec=3.0)
            return
        try:
            bgr = self._bridge.imgmsg_to_cv2(image_message, desired_encoding="bgr8")
            depth = self._bridge.imgmsg_to_cv2(depth_message, desired_encoding="passthrough")
            depth_m = depth_to_meters(depth, depth_message.encoding)
        except Exception as error:
            self.get_logger().error(f"Image conversion failed: {error}")
            return

        if depth_m.shape[:2] != bgr.shape[:2]:
            dimensions = (bgr.shape[1], bgr.shape[0])
            depth_m = cv2.resize(depth_m, dimensions, interpolation=cv2.INTER_NEAREST)
            current = (depth_message.width, depth_message.height, image_message.width, image_message.height)
            if current != self._last_dimension_warning:
                self.get_logger().warning(
                    "Depth and color dimensions differ; nearest-neighbour resize applied. "
                    "Verify Gemini Pro depth_registration/D2C is enabled: "
                    f"depth={current[:2]}, color={current[2:]}"
                )
                self._last_dimension_warning = current

        try:
            segmentations = self._backend.infer(bgr)
        except Exception as error:
            self.get_logger().error(f"Detector inference failed: {error}", throttle_duration_sec=2.0)
            return
        if not segmentations:
            self._publish_debug(image_message, bgr, None, "no bottle")
            return
        segmentation = max(segmentations, key=lambda item: item.score)
        if int(np.count_nonzero(segmentation.mask)) < self.get_parameter("minimum_mask_area").value:
            return

        try:
            transform = self._tf_buffer.lookup_transform(
                self.get_parameter("target_frame").value,
                image_message.header.frame_id,
                Time.from_msg(image_message.header.stamp),
                timeout=Duration(seconds=0.12),
            )
        except TransformException as error:
            self.get_logger().warning(f"Camera TF unavailable: {error}", throttle_duration_sec=2.0)
            return

        intrinsics = CameraIntrinsics(
            float(camera_info.k[0]), float(camera_info.k[4]),
            float(camera_info.k[2]), float(camera_info.k[5])
        )
        mask = segmentation.mask.astype(np.uint8)
        erosion = int(self.get_parameter("mask_erode_pixels").value)
        if erosion > 0:
            kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (2 * erosion + 1,) * 2)
            object_mask = cv2.erode(mask, kernel).astype(bool)
        else:
            object_mask = mask.astype(bool)
        ring_pixels = int(self.get_parameter("table_ring_pixels").value)
        ring_kernel = cv2.getStructuringElement(
            cv2.MORPH_ELLIPSE, (2 * ring_pixels + 1, 2 * ring_pixels + 1)
        )
        ring_mask = cv2.dilate(mask, ring_kernel).astype(bool) & ~cv2.dilate(
            mask, np.ones((7, 7), np.uint8)
        ).astype(bool)

        kwargs = {
            "stride": int(self.get_parameter("point_stride").value),
            "min_depth": float(self.get_parameter("minimum_depth").value),
            "max_depth": float(self.get_parameter("maximum_depth").value),
        }
        object_camera = project_masked_depth(depth_m, object_mask, intrinsics, **kwargs)
        table_camera = project_masked_depth(depth_m, ring_mask, intrinsics, **kwargs)
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
        object_base = transform_points(object_camera, translation, quaternion)
        table_base = transform_points(table_camera, translation, quaternion)
        try:
            estimate = estimate_bottle_geometry(
                object_base,
                table_base,
                minimum_points=int(self.get_parameter("minimum_points").value),
            )
        except ValueError as error:
            self.get_logger().warning(f"3D bottle estimate rejected: {error}", throttle_duration_sec=1.0)
            return

        stamp_seconds = Time.from_msg(image_message.header.stamp).nanoseconds * 1e-9
        track = self._tracker.update(estimate.grasp_point, stamp_seconds)
        horizon = float(self.get_parameter("prediction_horizon").value)
        predicted = self._tracker.predict(stamp_seconds + horizon)
        frame = self.get_parameter("target_frame").value
        self._publish_outputs(image_message, frame, segmentation, estimate, track, predicted)
        self._publish_debug(
            image_message,
            bgr,
            segmentation,
            f"{estimate.size[0]*1000:.0f}x{estimate.size[1]*1000:.0f}x{estimate.size[2]*1000:.0f} mm",
        )

    @staticmethod
    def _yaw_quaternion(yaw):
        return 0.0, 0.0, math.sin(0.5 * yaw), math.cos(0.5 * yaw)

    def _pose(self, header, position, yaw):
        message = PoseStamped()
        message.header = header
        message.pose.position.x = float(position[0])
        message.pose.position.y = float(position[1])
        message.pose.position.z = float(position[2])
        qx, qy, qz, qw = self._yaw_quaternion(yaw)
        message.pose.orientation.x = qx
        message.pose.orientation.y = qy
        message.pose.orientation.z = qz
        message.pose.orientation.w = qw
        return message

    def _publish_outputs(self, image_message, frame, segmentation, estimate, track, predicted):
        header = Header()
        header.stamp = image_message.header.stamp
        header.frame_id = frame
        grasp_pose = self._pose(header, track.position, estimate.yaw)
        predicted_pose = self._pose(header, predicted, estimate.yaw)
        self._grasp_pub.publish(grasp_pose)
        if track.observations >= int(self.get_parameter("minimum_track_observations").value):
            self._predicted_pub.publish(predicted_pose)

        twist = TwistStamped()
        twist.header = header
        twist.twist.linear.x, twist.twist.linear.y, twist.twist.linear.z = map(float, track.velocity)
        self._twist_pub.publish(twist)

        detection_array = Detection3DArray()
        detection_array.header = header
        detection = Detection3D()
        detection.header = header
        detection.id = "bottle-0"
        detection.bbox.center = self._pose(header, estimate.center, estimate.yaw).pose
        detection.bbox.size.x, detection.bbox.size.y, detection.bbox.size.z = map(float, estimate.size)
        hypothesis = ObjectHypothesisWithPose()
        hypothesis.hypothesis.class_id = "bottle"
        hypothesis.hypothesis.score = float(segmentation.score)
        hypothesis.pose.pose = detection.bbox.center
        detection.results.append(hypothesis)
        detection_array.detections.append(detection)
        self._detection_pub.publish(detection_array)

        marker = Marker()
        marker.header = header
        marker.ns = "bottle_grasp"
        marker.id = 0
        marker.type = Marker.CUBE
        marker.action = Marker.ADD
        marker.pose = detection.bbox.center
        marker.scale = detection.bbox.size
        marker.color.r, marker.color.g, marker.color.b, marker.color.a = 0.1, 0.8, 0.2, 0.45
        marker.lifetime = Duration(seconds=0.25).to_msg()
        self._marker_pub.publish(marker)

    def _publish_debug(self, source, image, segmentation, text):
        debug = image.copy()
        if segmentation is not None:
            overlay = np.zeros_like(debug)
            overlay[segmentation.mask] = (0, 180, 0)
            debug = cv2.addWeighted(debug, 1.0, overlay, 0.35, 0.0)
            x1, y1, x2, y2 = np.rint(segmentation.xyxy).astype(int)
            cv2.rectangle(debug, (x1, y1), (x2, y2), (0, 255, 0), 2)
        cv2.putText(debug, text, (12, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (20, 240, 240), 2)
        message = self._bridge.cv2_to_imgmsg(debug, encoding="bgr8")
        message.header = source.header
        self._debug_pub.publish(message)


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = BottlePerceptionNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            if node is not None:
                node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
        except KeyboardInterrupt:
            pass


if __name__ == "__main__":
    main()
