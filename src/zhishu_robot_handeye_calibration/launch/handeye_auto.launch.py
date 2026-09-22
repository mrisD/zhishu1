#!/usr/bin/env python3
"""Run the complete Eye-to-Hand simulation, calibration, and validation."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    ExecuteProcess,
    IncludeLaunchDescription,
    RegisterEventHandler,
    SetLaunchConfiguration,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable, LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_share = get_package_share_directory("zhishu_robot_handeye_calibration")
    simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(package_share, "launch", "handeye_gazebo.launch.py")
        ),
        launch_arguments={
            "headless": LaunchConfiguration("headless"),
            "use_rviz": LaunchConfiguration("use_rviz"),
        }.items(),
    )
    calibration = Node(
        package="zhishu_robot_handeye_calibration",
        executable="handeye_auto_calibration",
        name="handeye_auto_calibration",
        output="screen",
        parameters=[
            {"use_sim_time": True},
            {
                "output_file": LaunchConfiguration("output_file"),
                "image_directory": LaunchConfiguration("image_directory"),
                "moveit_samples_file": LaunchConfiguration("moveit_samples_file"),
                "truth_sensor_frame": LaunchConfiguration("truth_sensor_frame"),
                "translation_tolerance": LaunchConfiguration("translation_tolerance"),
                "rotation_tolerance": LaunchConfiguration("rotation_tolerance"),
                "minimum_markers": LaunchConfiguration("minimum_markers"),
                "sample_attempts_per_pose": LaunchConfiguration("sample_attempts_per_pose"),
                "reprojection_translation_tolerance": LaunchConfiguration(
                    "reprojection_translation_tolerance"
                ),
                "reprojection_rotation_tolerance": LaunchConfiguration(
                    "reprojection_rotation_tolerance"
                ),
            },
        ],
    )
    pause_gazebo = ExecuteProcess(
        cmd=[
            "gz",
            "service",
            "-s",
            "/world/handeye_empty/control",
            "--reqtype",
            "gz.msgs.WorldControl",
            "--reptype",
            "gz.msgs.Boolean",
            "--timeout",
            "5000",
            "--req",
            "pause: true",
        ],
        name="pause_gazebo_cleanly",
        output="screen",
    )
    remove_robot = ExecuteProcess(
        cmd=[
            "gz",
            "service",
            "-s",
            "/world/handeye_empty/remove",
            "--reqtype",
            "gz.msgs.Entity",
            "--reptype",
            "gz.msgs.Boolean",
            "--timeout",
            "5000",
            "--req",
            'name: "zhishu_robot", type: MODEL',
        ],
        name="unload_gazebo_robot",
        output="screen",
    )
    stop_when_complete = RegisterEventHandler(
        OnProcessExit(
            target_action=calibration,
            on_exit=[pause_gazebo],
        )
    )
    unload_after_pause = RegisterEventHandler(
        OnProcessExit(
            target_action=pause_gazebo,
            on_exit=[TimerAction(period=1.0, actions=[remove_robot])],
        )
    )
    shutdown_after_cleanup = RegisterEventHandler(
        OnProcessExit(
            target_action=remove_robot,
            on_exit=[
                TimerAction(
                    period=1.0,
                    actions=[
                        EmitEvent(
                            event=Shutdown(
                                reason="automatic hand-eye calibration finished"
                            )
                        )
                    ],
                )
            ],
        )
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "output_file", default_value="/tmp/handeye_calibration_result.yaml"
            ),
            DeclareLaunchArgument(
                "image_directory", default_value="/tmp/handeye_calibration_samples"
            ),
            DeclareLaunchArgument(
                "moveit_samples_file", default_value="/tmp/handeye_moveit_samples.yaml"
            ),
            DeclareLaunchArgument(
                "truth_sensor_frame", default_value="head_camera_optical_truth"
            ),
            DeclareLaunchArgument("headless", default_value="true"),
            DeclareLaunchArgument("use_rviz", default_value="false"),
            DeclareLaunchArgument("translation_tolerance", default_value="0.03"),
            DeclareLaunchArgument("rotation_tolerance", default_value="0.05"),
            DeclareLaunchArgument("minimum_markers", default_value="3"),
            DeclareLaunchArgument("sample_attempts_per_pose", default_value="3"),
            DeclareLaunchArgument(
                "reprojection_translation_tolerance", default_value="0.02"
            ),
            DeclareLaunchArgument(
                "reprojection_rotation_tolerance", default_value="0.03"
            ),
            # MoveGroup may need more than the launch default of five seconds
            # to tear down its planning pipelines and controller clients.
            SetLaunchConfiguration("sigterm_timeout", "20"),
            SetLaunchConfiguration("sigkill_timeout", "10"),
            # Gazebo Transport is independent from DDS.  Give each ROS domain
            # its own partition so cleanup services cannot hit another server.
            SetEnvironmentVariable(
                "GZ_PARTITION",
                [
                    "zhishu_handeye_",
                    EnvironmentVariable("ROS_DOMAIN_ID", default_value="0"),
                ],
            ),
            simulation,
            stop_when_complete,
            unload_after_pause,
            shutdown_after_cleanup,
            TimerAction(period=12.0, actions=[calibration]),
        ]
    )
