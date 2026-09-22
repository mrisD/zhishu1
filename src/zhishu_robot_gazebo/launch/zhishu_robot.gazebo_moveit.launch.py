#!/usr/bin/env python3
"""Start Gazebo, MoveIt move_group, and RViz against one controller stack."""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    headless = LaunchConfiguration("headless")
    use_rviz = LaunchConfiguration("use_rviz")
    world_file = LaunchConfiguration("world_file")
    use_camera = LaunchConfiguration("use_camera")
    use_head_camera = LaunchConfiguration("use_head_camera")
    hide_head_pitch_visual = LaunchConfiguration("hide_head_pitch_visual")
    handeye_target_enabled = LaunchConfiguration("handeye_target_enabled")
    managed_move_group_shutdown = LaunchConfiguration("managed_move_group_shutdown")
    managed_gazebo_shutdown = LaunchConfiguration("managed_gazebo_shutdown")

    gazebo_share = FindPackageShare("zhishu_robot_gazebo").find(
        "zhishu_robot_gazebo"
    )
    moveit_share = FindPackageShare("zhishu_robot_moveit_config").find(
        "zhishu_robot_moveit_config"
    )
    move_group_common_params = os.path.join(
        moveit_share, "config", "move_group_common.yaml"
    )

    moveit_config = (
        MoveItConfigsBuilder(
            "zhishu_robot", package_name="zhishu_robot_moveit_config"
        )
        .robot_description(
            mappings={
                "handeye_target_enabled": handeye_target_enabled,
                "use_head_camera": use_head_camera,
                "hide_head_pitch_visual": hide_head_pitch_visual,
            }
        )
        .to_moveit_configs()
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                gazebo_share, "launch", "zhishu_robot.gazebo.launch.py"
            )
        ),
        launch_arguments={
            "headless": headless,
            "world_file": world_file,
            "load_controllers": "true",
            "use_gazebo": "true",
            "use_robot_state_pub": "true",
            "use_sim_time": "true",
            "use_camera": use_camera,
            "use_head_camera": use_head_camera,
            "hide_head_pitch_visual": hide_head_pitch_visual,
            "handeye_target_enabled": handeye_target_enabled,
            "managed_gazebo_shutdown": managed_gazebo_shutdown,
        }.items(),
    )

    move_group = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            move_group_common_params,
            {"use_sim_time": True},
        ],
        condition=UnlessCondition(managed_move_group_shutdown),
    )

    # MoveIt 2.12.x on Jazzy has a known executor-destructor crash on both
    # SIGINT and SIGTERM.  This optional supervisor preserves all runtime
    # failures but bypasses only that broken shutdown path.
    managed_move_group = Node(
        package="zhishu_robot_gazebo",
        executable="managed_process.py",
        name="move_group",
        output="screen",
        arguments=[
            "--managed-target",
            "/opt/ros/jazzy/lib/moveit_ros_move_group/move_group",
        ],
        parameters=[
            moveit_config.to_dict(),
            move_group_common_params,
            {"use_sim_time": True},
        ],
        condition=IfCondition(managed_move_group_shutdown),
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        output="screen",
        arguments=[
            "-d",
            os.path.join(moveit_share, "config", "moveit.rviz"),
        ],
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

    # Gazebo creates the robot and its controller manager asynchronously.
    # Starting MoveIt shortly afterwards avoids action-server discovery races
    # without creating a second fake ros2_control stack.
    start_visualization = TimerAction(
        period=4.0,
        actions=[move_group, managed_move_group, rviz],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "headless",
                default_value="false",
                description="Run Gazebo without its GUI",
            ),
            DeclareLaunchArgument(
                "use_rviz",
                default_value="true",
                description="Start RViz with the MoveIt configuration",
            ),
            DeclareLaunchArgument(
                "world_file",
                default_value="pick_and_place_demo.world",
                description="Gazebo world file",
            ),
            DeclareLaunchArgument(
                "use_camera",
                default_value="false",
                description="Start the legacy RGBD image bridge",
            ),
            DeclareLaunchArgument(
                "use_head_camera",
                default_value="false",
                description="Enable the simulated head RGB camera",
            ),
            DeclareLaunchArgument(
                "hide_head_pitch_visual",
                default_value="false",
                description="Hide head_pitch_link visual geometry only",
            ),
            DeclareLaunchArgument(
                "handeye_target_enabled",
                default_value="false",
                description="Replace the rendered left gripper with the flange-mounted hand-eye target",
            ),
            DeclareLaunchArgument(
                "managed_move_group_shutdown",
                default_value="false",
                description="Work around the MoveIt Jazzy shutdown crash",
            ),
            DeclareLaunchArgument(
                "managed_gazebo_shutdown",
                default_value="false",
                description="Work around known Gazebo teardown crashes",
            ),
            gazebo,
            start_visualization,
        ]
    )
