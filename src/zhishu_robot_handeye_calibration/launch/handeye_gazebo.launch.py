#!/usr/bin/env python3
"""Launch Zhishu Eye-to-Hand calibration in Gazebo Harmonic."""

import os
import sys

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    AppendEnvironmentVariable,
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    headless = LaunchConfiguration("headless")
    use_rviz = LaunchConfiguration("use_rviz")
    hide_head_pitch_visual = LaunchConfiguration("hide_head_pitch_visual")

    package_share = get_package_share_directory("zhishu_robot_handeye_calibration")
    gazebo_share = get_package_share_directory("zhishu_robot_gazebo")

    calibration_resources = AppendEnvironmentVariable(
        "GZ_SIM_RESOURCE_PATH", os.path.dirname(package_share)
    )

    simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                gazebo_share,
                "launch",
                "zhishu_robot.gazebo_moveit.launch.py",
            )
        ),
        launch_arguments={
            "headless": headless,
            "use_rviz": "false",
            "world_file": "handeye_empty.sdf",
            "use_camera": "false",
            "use_head_camera": "true",
            "hide_head_pitch_visual": hide_head_pitch_visual,
            "handeye_target_enabled": "true",
            "software_rendering": "true",
            "initial_positions_file": os.path.join(
                package_share, "config", "initial_positions.yaml"
            ),
        }.items(),
    )

    moveit_config = (
        MoveItConfigsBuilder(
            "zhishu_robot", package_name="zhishu_robot_moveit_config"
        )
        .to_moveit_configs()
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="handeye_rviz",
        output="screen",
        arguments=["-d", os.path.join(package_share, "rviz", "handeye.rviz")],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
            moveit_config.joint_limits,
            {"use_sim_time": True},
        ],
        condition=IfCondition(use_rviz),
    )

    # Invoke the interpreter explicitly. This also works when files were copied
    # from a Windows checkout where the POSIX executable bit is unavailable.
    head_guard = ExecuteProcess(
        cmd=[
            sys.executable,
            os.path.join(
                package_share,
                "..",
                "..",
                "lib",
                "zhishu_robot_handeye_calibration",
                "head_lock_guard.py",
            ),
            "--ros-args",
            "-p",
            "use_sim_time:=true",
            "-p",
            "head_yaw_target:=0.5",
            "-p",
            "head_pitch_target:=0.15",
            "-p",
            "tolerance:=0.002",
        ],
        output="screen",
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("headless", default_value="false"),
            DeclareLaunchArgument("use_rviz", default_value="true"),
            DeclareLaunchArgument(
                "hide_head_pitch_visual",
                default_value="true",
                description="Hide only head_pitch_link visual geometry",
            ),
            calibration_resources,
            simulation,
            TimerAction(period=7.0, actions=[head_guard, rviz]),
        ]
    )
