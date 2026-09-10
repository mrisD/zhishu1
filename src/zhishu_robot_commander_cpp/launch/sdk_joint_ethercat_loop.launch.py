#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    loop_count = LaunchConfiguration("loop_count")
    interval_sec = LaunchConfiguration("interval_sec")
    use_sim_time = LaunchConfiguration("use_sim_time")
    use_fake_hardware = LaunchConfiguration("use_fake_hardware")
    enable_motors_on_activate = LaunchConfiguration("enable_motors_on_activate")

    package_share = get_package_share_directory("zhishu_robot_commander_cpp")
    commander_yaml = os.path.join(package_share, "config", "commander.yaml")
    joint_loop_yaml = os.path.join(
        package_share, "config", "joint_ethercat_loop.yaml"
    )
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

    loop_node = Node(
        package="zhishu_robot_commander_cpp",
        executable="sdk_joint_ethercat_loop",
        name="zhishu_robot_joint_ethercat_loop",
        output="screen",
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.joint_limits,
            commander_yaml,
            joint_loop_yaml,
            {
                "loop_count": loop_count,
                "interval_sec": interval_sec,
                "use_sim_time": use_sim_time,
            },
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "loop_count",
                default_value="0",
                description="Number of all-zero -> configured pose -> all-zero cycles; 0 loops forever",
            ),
            DeclareLaunchArgument(
                "interval_sec",
                default_value="1.5",
                description="Pause in seconds between joint motions",
            ),
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument("use_fake_hardware", default_value="false"),
            DeclareLaunchArgument("enable_motors_on_activate", default_value="true"),
            loop_node,
        ]
    )
