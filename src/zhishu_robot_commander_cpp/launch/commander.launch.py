#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    use_fake_hardware = LaunchConfiguration("use_fake_hardware")
    enable_motors_on_activate = LaunchConfiguration("enable_motors_on_activate")
    package_share = get_package_share_directory("zhishu_robot_commander_cpp")
    commander_yaml = os.path.join(package_share, "config", "commander.yaml")

    moveit_config = (
        MoveItConfigsBuilder(
            "zhishu_robot", package_name="zhishu_robot_moveit_config"
        )
        .robot_description(
            file_path="config/zhishu_robot.urdf.xacro",
            mappings={
                "use_gazebo": "false",
                "use_fake_hardware": use_fake_hardware,
                "enable_motors_on_activate": enable_motors_on_activate,
            },
        )
        .robot_description_semantic(file_path="config/zhishu_robot.srdf")
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .joint_limits(file_path="config/joint_limits.yaml")
        .to_moveit_configs()
    )

    commander = Node(
        package="zhishu_robot_commander_cpp",
        executable="commander_node",
        name="zhishu_robot_commander",
        output="screen",
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.joint_limits,
            commander_yaml,
            {"use_sim_time": use_sim_time},
            {"commander.coordinate_frames_file": LaunchConfiguration("coordinate_frames_file")},
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument("coordinate_frames_file", default_value=""),
            DeclareLaunchArgument(
                "use_fake_hardware",
                default_value="false",
                description="Match the running bringup hardware mode",
            ),
            DeclareLaunchArgument(
                "enable_motors_on_activate",
                default_value="true",
                description="Match the running bringup robot description",
            ),
            commander,
        ]
    )
