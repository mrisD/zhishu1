#!/usr/bin/env python3
"""Launch the complete LES receiving workflow in Gazebo and MoveIt."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    package_share = get_package_share_directory("zhishu_robot_receiving_sim")
    gazebo_share = get_package_share_directory("zhishu_robot_gazebo")
    commander_share = get_package_share_directory("zhishu_robot_commander_cpp")

    headless = LaunchConfiguration("headless")
    use_rviz = LaunchConfiguration("use_rviz")
    auto_start = LaunchConfiguration("auto_start")
    startup_delay = LaunchConfiguration("startup_delay_seconds")
    mismatch_attempts = LaunchConfiguration("part_mismatch_attempts")
    quality_pass = LaunchConfiguration("quality_pass")
    simulate_robot = LaunchConfiguration("simulate_robot_motion")
    require_robot = LaunchConfiguration("require_robot_motion_success")
    simulate_agv = LaunchConfiguration("simulate_agv_motion")
    move_box = LaunchConfiguration("move_box_in_gazebo")

    gazebo_moveit = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                gazebo_share, "launch", "zhishu_robot.gazebo_moveit.launch.py"
            )
        ),
        launch_arguments={
            "headless": headless,
            "use_rviz": use_rviz,
            "world_file": "receiving_demo.sdf",
        }.items(),
    )

    receiving_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="receiving_sim_bridge",
        output="screen",
        parameters=[
            {
                "config_file": os.path.join(
                    package_share, "config", "receiving_bridge.yaml"
                )
            }
        ],
    )

    commander = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(commander_share, "launch", "commander.launch.py")
        ),
        launch_arguments={
            "use_sim_time": "true",
            "use_fake_hardware": "false",
            "enable_motors_on_activate": "true",
        }.items(),
    )

    les_mock = Node(
        package="zhishu_robot_receiving_sim",
        executable="les_mock",
        output="screen",
        parameters=[
            {"use_sim_time": True},
            {"quality_pass": ParameterValue(quality_pass, value_type=bool)},
        ],
    )
    scanner_mock = Node(
        package="zhishu_robot_receiving_sim",
        executable="scanner_mock",
        output="screen",
        parameters=[
            {"use_sim_time": True},
            {
                "part_mismatch_attempts": ParameterValue(
                    mismatch_attempts, value_type=int
                )
            },
        ],
    )
    orchestrator = Node(
        package="zhishu_robot_receiving_sim",
        executable="receiving_orchestrator",
        output="screen",
        parameters=[
            {"use_sim_time": True},
            {"auto_start": ParameterValue(auto_start, value_type=bool)},
            {
                "startup_delay_seconds": ParameterValue(
                    startup_delay, value_type=float
                )
            },
            {
                "simulate_robot_motion": ParameterValue(
                    simulate_robot, value_type=bool
                )
            },
            {
                "require_robot_motion_success": ParameterValue(
                    require_robot, value_type=bool
                )
            },
            {
                "simulate_agv_motion": ParameterValue(
                    simulate_agv, value_type=bool
                )
            },
            {
                "move_box_in_gazebo": ParameterValue(move_box, value_type=bool)
            },
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("headless", default_value="false"),
            DeclareLaunchArgument("use_rviz", default_value="true"),
            DeclareLaunchArgument("auto_start", default_value="true"),
            DeclareLaunchArgument("startup_delay_seconds", default_value="12.0"),
            DeclareLaunchArgument("part_mismatch_attempts", default_value="1"),
            DeclareLaunchArgument("quality_pass", default_value="true"),
            DeclareLaunchArgument("simulate_robot_motion", default_value="true"),
            DeclareLaunchArgument(
                "require_robot_motion_success",
                default_value="false",
                description="Fail the workflow instead of using the virtual fallback",
            ),
            DeclareLaunchArgument("simulate_agv_motion", default_value="true"),
            DeclareLaunchArgument("move_box_in_gazebo", default_value="true"),
            gazebo_moveit,
            receiving_bridge,
            les_mock,
            scanner_mock,
            # MoveIt starts after Gazebo in the included launch. Start Commander
            # afterwards so it discovers the existing action servers.
            TimerAction(period=8.0, actions=[commander]),
            orchestrator,
        ]
    )
