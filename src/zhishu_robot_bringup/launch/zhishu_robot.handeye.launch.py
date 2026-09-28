#!/usr/bin/env python3
"""Start real-robot MoveIt bringup with the left hand-eye target installed."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    bringup_share = get_package_share_directory("zhishu_robot_bringup")
    handeye_share = get_package_share_directory("zhishu_robot_handeye_calibration")
    camera_share = handeye_share

    left_arm_use_fake_hardware = LaunchConfiguration("left_arm_use_fake_hardware")
    right_arm_use_fake_hardware = LaunchConfiguration("right_arm_use_fake_hardware")
    torso_head_use_fake_hardware = LaunchConfiguration("torso_head_use_fake_hardware")
    enable_left_arm_hardware = LaunchConfiguration("enable_left_arm_hardware")
    enable_right_arm_hardware = LaunchConfiguration("enable_right_arm_hardware")
    enable_torso_head_hardware = LaunchConfiguration("enable_torso_head_hardware")
    enable_motors_on_activate = LaunchConfiguration("enable_motors_on_activate")
    torso_head_enable_motors_on_activate = LaunchConfiguration(
        "torso_head_enable_motors_on_activate"
    )
    use_rviz = LaunchConfiguration("use_rviz")
    enable_commander = LaunchConfiguration("enable_commander")
    start_camera = LaunchConfiguration("start_camera")
    run_camera_checks = LaunchConfiguration("run_camera_checks")
    start_pose_sequence = LaunchConfiguration("start_pose_sequence")
    pose_sequence_delay = LaunchConfiguration("pose_sequence_delay")

    robot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_share, "launch", "zhishu_robot.moveit.launch.py")
        ),
        launch_arguments={
            "left_arm_use_fake_hardware": left_arm_use_fake_hardware,
            "right_arm_use_fake_hardware": right_arm_use_fake_hardware,
            "torso_head_use_fake_hardware": torso_head_use_fake_hardware,
            "enable_left_arm_hardware": enable_left_arm_hardware,
            "enable_right_arm_hardware": enable_right_arm_hardware,
            "enable_torso_head_hardware": enable_torso_head_hardware,
            "enable_motors_on_activate": enable_motors_on_activate,
            "torso_head_enable_motors_on_activate": (
                torso_head_enable_motors_on_activate
            ),
            "use_rviz": use_rviz,
            "rviz_config_file": os.path.join(
                handeye_share, "rviz", "handeye.rviz"
            ),
            "enable_commander": enable_commander,
            "handeye_target_enabled": "true",
            "initial_positions_file": os.path.join(
                handeye_share, "config", "initial_positions.yaml"
            ),
        }.items(),
    )

    camera = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(camera_share, "launch", "gemini_pro_camera.launch.py")
        ),
        launch_arguments={
            "camera_name": "camera_head",
            "frame_id": "head_camera_optical_frame",
            "enable_depth": "true",
            "enable_d2c": "true",
            "enable_point_cloud": "true",
            "enable_colored_point_cloud": "true",
            "publish_tf": "false",
        }.items(),
        condition=IfCondition(start_camera),
    )

    camera_info_check = Node(
        package="zhishu_robot_handeye_calibration",
        executable="check_camera_info.py",
        name="gemini_pro_camera_info_check",
        output="screen",
        condition=IfCondition(run_camera_checks),
    )
    camera_target_check = Node(
        package="zhishu_robot_handeye_calibration",
        executable="check_camera_target.py",
        name="handeye_camera_target_check",
        output="screen",
        condition=IfCondition(run_camera_checks),
    )

    # This node is intentionally opt-in: it waits for Enter before each move,
    # so a real robot never moves merely because the launch file was started.
    pose_sequence = Node(
        package="zhishu_robot_handeye_calibration",
        executable="handeye_pose_sequence",
        name="handeye_pose_sequence",
        output="screen",
        emulate_tty=True,
        condition=IfCondition(start_pose_sequence),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "left_arm_use_fake_hardware",
                default_value="false",
                description="Left arm mode: true=mock, false=EtherCAT",
            ),
            DeclareLaunchArgument(
                "right_arm_use_fake_hardware",
                default_value="false",
                description="Right arm mode: true=mock, false=EtherCAT",
            ),
            DeclareLaunchArgument(
                "torso_head_use_fake_hardware",
                default_value="false",
                description="Torso/head mode: true=mock, false=EtherCAT",
            ),
            DeclareLaunchArgument("enable_left_arm_hardware", default_value="true"),
            DeclareLaunchArgument("enable_right_arm_hardware", default_value="false"),
            DeclareLaunchArgument("enable_torso_head_hardware", default_value="true"),
            DeclareLaunchArgument(
                "enable_motors_on_activate",
                default_value="true",
                description=(
                    "Enable real arm drives on activation; set false for "
                    "communication-only checks before motion is verified"
                ),
            ),
            DeclareLaunchArgument(
                "torso_head_enable_motors_on_activate", default_value="true"
            ),
            DeclareLaunchArgument("use_rviz", default_value="true"),
            DeclareLaunchArgument("enable_commander", default_value="false"),
            DeclareLaunchArgument(
                "start_camera",
                default_value="true",
                description="Start the official Gemini Pro RGB/depth/D2C/point-cloud driver",
            ),
            DeclareLaunchArgument(
                "run_camera_checks",
                default_value="true",
                description="Run automatic RGB/CameraInfo and target-frame checks",
            ),
            DeclareLaunchArgument(
                "start_pose_sequence",
                default_value="false",
                description=(
                    "Start the interactive pose sequence; it asks for Enter before "
                    "each motion and is disabled by default for safety"
                ),
            ),
            DeclareLaunchArgument(
                "pose_sequence_delay",
                default_value="15.0",
                description="Seconds to wait before starting the interactive sequence",
            ),
            robot,
            camera,
            TimerAction(
                period=8.0,
                actions=[camera_info_check, camera_target_check],
            ),
            TimerAction(
                period=pose_sequence_delay,
                actions=[pose_sequence],
            )
        ]
    )
