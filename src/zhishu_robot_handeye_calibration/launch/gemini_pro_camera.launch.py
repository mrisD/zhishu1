#!/usr/bin/env python3
"""Launch the legacy Gemini Pro through the official Orbbec ROS 2 driver."""

from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def _default_camera_info_url():
    path = Path.home() / ".ros" / "camera_info" / "gemini_pro_rgb_640x480.yaml"
    return path.as_uri()


def generate_launch_description():
    """Run RGB, registered depth, D2C, and point clouds in one SDK pipeline."""
    camera_name = LaunchConfiguration("camera_name")
    frame_id = LaunchConfiguration("frame_id")

    arguments = [
        DeclareLaunchArgument("camera_name", default_value="camera_head"),
        DeclareLaunchArgument(
            "frame_id",
            default_value="head_camera_optical_frame",
            description=(
                "Optical frame used by RGB, registered depth, and point clouds. "
                "The robot model owns this TF, so the camera driver does not publish TF."
            ),
        ),
        DeclareLaunchArgument("camera_info_url", default_value=_default_camera_info_url()),
        DeclareLaunchArgument("color_width", default_value="640"),
        DeclareLaunchArgument("color_height", default_value="480"),
        DeclareLaunchArgument("color_fps", default_value="30"),
        DeclareLaunchArgument("color_format", default_value="MJPG"),
        DeclareLaunchArgument("depth_width", default_value="640"),
        DeclareLaunchArgument("depth_height", default_value="400"),
        DeclareLaunchArgument("depth_fps", default_value="30"),
        DeclareLaunchArgument("depth_format", default_value="Y11"),
        DeclareLaunchArgument("enable_depth", default_value="true"),
        DeclareLaunchArgument("enable_d2c", default_value="true"),
        DeclareLaunchArgument("enable_point_cloud", default_value="true"),
        DeclareLaunchArgument("enable_colored_point_cloud", default_value="true"),
        DeclareLaunchArgument(
            "ordered_pc",
            default_value="true",
            description=(
                "Keep organized 640x480 point clouds. The vendor driver's unordered "
                "mode leaves row_step inconsistent after filtering invalid points."
            ),
        ),
        DeclareLaunchArgument(
            "publish_tf",
            default_value="false",
            description="Avoid duplicate TF publishers for the robot-owned camera frame.",
        ),
        DeclareLaunchArgument("log_level", default_value="info"),
    ]

    parameters = {
        "camera_name": camera_name,
        "vendor_id": "0x2bc5",
        "product_id": "0x0614",
        "enable_color": True,
        "color_width": LaunchConfiguration("color_width"),
        "color_height": LaunchConfiguration("color_height"),
        "color_fps": LaunchConfiguration("color_fps"),
        "color_format": LaunchConfiguration("color_format"),
        "color_info_url": LaunchConfiguration("camera_info_url"),
        "enable_color_auto_exposure": True,
        "enable_color_auto_white_balance": True,
        "enable_depth": LaunchConfiguration("enable_depth"),
        "depth_width": LaunchConfiguration("depth_width"),
        "depth_height": LaunchConfiguration("depth_height"),
        "depth_fps": LaunchConfiguration("depth_fps"),
        "depth_format": LaunchConfiguration("depth_format"),
        "enable_ir": False,
        "depth_registration": LaunchConfiguration("enable_d2c"),
        "align_mode": "HW",
        "enable_depth_scale": True,
        "enable_d2c_viewer": LaunchConfiguration("enable_d2c"),
        "enable_point_cloud": LaunchConfiguration("enable_point_cloud"),
        "enable_colored_point_cloud": LaunchConfiguration(
            "enable_colored_point_cloud"
        ),
        "ordered_pc": LaunchConfiguration("ordered_pc"),
        "enable_publish_extrinsic": True,
        "publish_tf": LaunchConfiguration("publish_tf"),
        "tf_publish_rate": 0.0,
        "color_optical_frame_id": frame_id,
        "depth_optical_frame_id": frame_id,
        "cloud_frame_id": frame_id,
        "log_level": LaunchConfiguration("log_level"),
    }

    camera = ComposableNode(
        package="orbbec_camera",
        plugin="orbbec_camera::OBCameraNodeDriver",
        name="gemini_pro",
        namespace=camera_name,
        parameters=[parameters],
    )
    container = ComposableNodeContainer(
        name="gemini_pro_container",
        namespace="",
        package="rclcpp_components",
        executable="component_container",
        composable_node_descriptions=[camera],
        output="screen",
    )

    return LaunchDescription(
        arguments
        + [
            LogInfo(
                msg=[
                    "Gemini Pro official driver: /",
                    camera_name,
                    "/color, registered depth, D2C, and point clouds",
                ]
            ),
            container,
        ]
    )
