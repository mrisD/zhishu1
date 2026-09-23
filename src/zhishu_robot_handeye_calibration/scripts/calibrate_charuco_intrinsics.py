#!/usr/bin/env python3
"""Interactively calibrate Gemini Pro RGB intrinsics from the hand-eye board."""

import sys
from datetime import datetime, timezone
from pathlib import Path

import cv2
from cv_bridge import CvBridge
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
import yaml


class CharucoIntrinsicCalibration(Node):
    """Collect ChArUco observations and write ROS CameraInfo calibration."""

    def __init__(self):
        """Configure the board detector, GUI, subscriber, and output path."""
        super().__init__("gemini_pro_charuco_intrinsic_calibration")
        self.image_topic = self.declare_parameter(
            "image_topic", "/camera_head/color/image_raw"
        ).value
        default_output = Path.home().joinpath(
            ".ros", "camera_info", "gemini_pro_rgb_640x480.yaml"
        )
        self.output_path = Path(
            self.declare_parameter("output_path", str(default_output)).value
        ).expanduser()
        self.camera_name = self.declare_parameter(
            "camera_name", "camera_head_color"
        ).value
        self.minimum_samples = self.declare_parameter("minimum_samples", 20).value
        self.minimum_corners = self.declare_parameter("minimum_corners", 12).value
        self.maximum_rms = self.declare_parameter("maximum_rms", 0.8).value

        self.bridge = CvBridge()
        self.dictionary = cv2.aruco.getPredefinedDictionary(
            cv2.aruco.DICT_4X4_50
        )
        self.board = cv2.aruco.CharucoBoard_create(
            7, 5, 0.030, 0.022, self.dictionary
        )
        self.detector_parameters = cv2.aruco.DetectorParameters_create()
        self.sample_corners = []
        self.sample_ids = []
        self.image_size = None
        self.current_corners = None
        self.current_ids = None
        self.done = False
        self.calibration = None
        self.window_name = "Gemini Pro ChArUco intrinsics"
        cv2.namedWindow(self.window_name, cv2.WINDOW_NORMAL)
        self.create_subscription(
            Image, self.image_topic, self._on_image, qos_profile_sensor_data
        )
        self.get_logger().info(
            "Controls: SPACE=capture, R=remove last, C=calibrate, "
            "S=calibrate and save, Q=quit"
        )
        self.get_logger().info(
            "Use 30-50 sharp views covering the center, edges and corners with "
            "different distances and board tilts."
        )

    def _on_image(self, message):
        image = self.bridge.imgmsg_to_cv2(message, desired_encoding="bgr8")
        size = (image.shape[1], image.shape[0])
        if self.image_size is None:
            self.image_size = size
            self.get_logger().info(f"receiving {size[0]}x{size[1]} RGB images")
        elif size != self.image_size:
            self.get_logger().error(
                f"image size changed from {self.image_size} to {size}; "
                "restart calibration with a fixed resolution"
            )
            return

        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        marker_corners, marker_ids, _ = cv2.aruco.detectMarkers(
            gray, self.dictionary, parameters=self.detector_parameters
        )
        self.current_corners = None
        self.current_ids = None
        marker_count = 0 if marker_ids is None else len(marker_ids)
        corner_count = 0
        if marker_ids is not None:
            cv2.aruco.drawDetectedMarkers(image, marker_corners, marker_ids)
            _, corners, ids = cv2.aruco.interpolateCornersCharuco(
                marker_corners, marker_ids, gray, self.board
            )
            if ids is not None:
                self.current_corners = corners
                self.current_ids = ids
                corner_count = len(ids)
                cv2.aruco.drawDetectedCornersCharuco(image, corners, ids)

        valid = corner_count >= self.minimum_corners
        status_color = (0, 200, 0) if valid else (0, 0, 255)
        lines = [
            f"samples: {len(self.sample_corners)} (minimum {self.minimum_samples})",
            f"detected: {marker_count} markers, {corner_count} ChArUco corners",
            "SPACE capture | R remove | C calibrate | S save | Q quit",
        ]
        if self.calibration is not None:
            lines.append(f"last RMS: {self.calibration['rms']:.4f} px")
        for index, text in enumerate(lines):
            cv2.putText(
                image,
                text,
                (15, 30 + index * 28),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.65,
                status_color if index == 1 else (255, 255, 255),
                2,
                cv2.LINE_AA,
            )

        cv2.imshow(self.window_name, image)
        key = cv2.waitKey(1) & 0xFF
        if key == ord(" "):
            self._capture_current()
        elif key in (ord("r"), ord("R")):
            self._remove_last()
        elif key in (ord("c"), ord("C")):
            self._calibrate(save=False)
        elif key in (ord("s"), ord("S")):
            self._calibrate(save=True)
        elif key in (ord("q"), ord("Q"), 27):
            self.done = True

    def _capture_current(self):
        if self.current_ids is None or len(self.current_ids) < self.minimum_corners:
            self.get_logger().warning(
                f"sample rejected: need at least {self.minimum_corners} ChArUco "
                "corners in the current image"
            )
            return
        self.sample_corners.append(self.current_corners.copy())
        self.sample_ids.append(self.current_ids.copy())
        self.calibration = None
        self.get_logger().info(
            f"captured sample {len(self.sample_corners)} with "
            f"{len(self.current_ids)} corners"
        )

    def _remove_last(self):
        if not self.sample_corners:
            self.get_logger().warning("there is no sample to remove")
            return
        self.sample_corners.pop()
        self.sample_ids.pop()
        self.calibration = None
        self.get_logger().info(f"removed sample; {len(self.sample_corners)} remain")

    def _calibrate(self, save):
        if len(self.sample_corners) < self.minimum_samples:
            self.get_logger().error(
                f"need at least {self.minimum_samples} samples; currently have "
                f"{len(self.sample_corners)}"
            )
            return

        result = cv2.aruco.calibrateCameraCharucoExtended(
            self.sample_corners,
            self.sample_ids,
            self.board,
            self.image_size,
            None,
            None,
        )
        rms, matrix, distortion = result[:3]
        per_view_errors = np.asarray(result[7], dtype=np.float64).reshape(-1)
        self.calibration = {
            "rms": float(rms),
            "matrix": matrix,
            "distortion": distortion.reshape(-1),
            "per_view_errors": per_view_errors,
        }
        worst = float(np.max(per_view_errors)) if per_view_errors.size else float("nan")
        self.get_logger().info(
            f"calibration complete: RMS={rms:.4f} px, "
            f"worst view={worst:.4f} px, samples={len(self.sample_corners)}"
        )
        if rms > self.maximum_rms:
            self.get_logger().error(
                f"RMS {rms:.4f} exceeds maximum_rms={self.maximum_rms:.4f}; "
                "remove weak views and capture more diverse sharp images"
            )
            if save:
                self.get_logger().error("calibration was not saved")
            return
        if rms > 0.5:
            self.get_logger().warning(
                "RMS is above the preferred 0.5 px target; the file is usable, "
                "but collecting stronger samples is recommended"
            )
        if save:
            self._save()

    def _save(self):
        matrix = self.calibration["matrix"]
        distortion = self.calibration["distortion"]
        projection = np.zeros((3, 4), dtype=np.float64)
        projection[:, :3] = matrix
        camera_info = {
            "image_width": int(self.image_size[0]),
            "image_height": int(self.image_size[1]),
            "camera_name": self.camera_name,
            "camera_matrix": {
                "rows": 3,
                "cols": 3,
                "data": [float(value) for value in matrix.reshape(-1)],
            },
            "distortion_model": "plumb_bob",
            "distortion_coefficients": {
                "rows": 1,
                "cols": int(distortion.size),
                "data": [float(value) for value in distortion],
            },
            "rectification_matrix": {
                "rows": 3,
                "cols": 3,
                "data": [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0],
            },
            "projection_matrix": {
                "rows": 3,
                "cols": 4,
                "data": [float(value) for value in projection.reshape(-1)],
            },
        }
        report = {
            "calibration_date": datetime.now(timezone.utc).isoformat(),
            "calibration_rms_error": float(self.calibration["rms"]),
            "calibration_samples": len(self.sample_corners),
            "per_view_errors": [
                float(value) for value in self.calibration["per_view_errors"]
            ],
            "charuco": {
                "dictionary": "DICT_4X4_50",
                "squares_x": 7,
                "squares_y": 5,
                "square_length": 0.030,
                "marker_length": 0.022,
            },
        }
        self.output_path.parent.mkdir(parents=True, exist_ok=True)
        with self.output_path.open("w", encoding="utf-8") as stream:
            yaml.safe_dump(camera_info, stream, sort_keys=False, allow_unicode=True)
        report_path = self.output_path.with_name(
            f"{self.output_path.stem}.report{self.output_path.suffix}"
        )
        with report_path.open("w", encoding="utf-8") as stream:
            yaml.safe_dump(report, stream, sort_keys=False, allow_unicode=True)
        self.get_logger().info(
            f"saved CameraInfo YAML to {self.output_path} and quality report "
            f"to {report_path}"
        )


def main():
    """Run the interactive calibration node until the user exits."""
    rclpy.init()
    node = CharucoIntrinsicCalibration()
    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
