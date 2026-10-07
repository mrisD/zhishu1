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
    bottle_grasp_sim_enabled = LaunchConfiguration("bottle_grasp_sim_enabled")
    mobile_base_sim_enabled = LaunchConfiguration("mobile_base_sim_enabled")
    initial_positions_file = LaunchConfiguration("initial_positions_file")
    managed_move_group_shutdown = LaunchConfiguration("managed_move_group_shutdown")
    managed_gazebo_shutdown = LaunchConfiguration("managed_gazebo_shutdown")
    use_left_arm_controller = LaunchConfiguration("use_left_arm_controller")
    x = LaunchConfiguration("x")
    y = LaunchConfiguration("y")
    z = LaunchConfiguration("z")
    roll = LaunchConfiguration("roll")
    pitch = LaunchConfiguration("pitch")
    yaw = LaunchConfiguration("yaw")
    allow_renaming = LaunchConfiguration("allow_renaming")

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
                "bottle_grasp_sim_enabled": bottle_grasp_sim_enabled,
                "mobile_base_sim_enabled": mobile_base_sim_enabled,
                "mobile_base_sim_mass": LaunchConfiguration("mobile_base_sim_mass"),
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
            "bottle_grasp_sim_enabled": bottle_grasp_sim_enabled,
            "mobile_base_sim_enabled": mobile_base_sim_enabled,
            "mobile_base_sim_mass": LaunchConfiguration("mobile_base_sim_mass"),
            "initial_positions_file": initial_positions_file,
            "managed_gazebo_shutdown": managed_gazebo_shutdown,
            "use_left_arm_controller": use_left_arm_controller,
            "x": x,
            "y": y,
            "z": z,
            "roll": roll,
            "pitch": pitch,
            "yaw": yaw,
            "allow_renaming": allow_renaming,
        }.items(),
    )

    move_group = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            move_group_common_params,
            {
                "use_sim_time": True,
                "trajectory_execution.allowed_execution_duration_scaling": 3.0,
                "trajectory_execution.allowed_goal_duration_margin": 10.0,
                # Required by MoveIt Task Constructor task.execute().  Loading
                # the capability is harmless for normal RViz planning and avoids
                # starting a second move_group for pallet-transfer simulation.
                "capabilities": "move_group/ExecuteTaskSolutionCapability",
            },
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
            {
                "use_sim_time": True,
                "trajectory_execution.allowed_execution_duration_scaling": 3.0,
                "trajectory_execution.allowed_goal_duration_margin": 10.0,
                "capabilities": "move_group/ExecuteTaskSolutionCapability",
            },
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
                "bottle_grasp_sim_enabled",
                default_value="false",
                description="Enable simulation-only bottle attach support",
            ),
            DeclareLaunchArgument(
                "mobile_base_sim_enabled",
                default_value="false",
                description="Enable Gazebo DiffDrive for the robot base",
            ),
            DeclareLaunchArgument("mobile_base_sim_mass", default_value="11.9793854739169"),
            DeclareLaunchArgument(
                "initial_positions_file",
                default_value=os.path.join(
                    moveit_share, "config", "initial_positions.yaml"
                ),
                description="Absolute path to the ros2_control initial joint positions YAML",
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
            DeclareLaunchArgument(
                "use_left_arm_controller",
                default_value="false",
                description=(
                    "Use left_arm_controller instead of dual_arm_controller "
                    "for left-arm-only execution"
                ),
            ),
            DeclareLaunchArgument("x", default_value="0.0"),
            DeclareLaunchArgument("y", default_value="0.0"),
            DeclareLaunchArgument("z", default_value="0.0858"),
            DeclareLaunchArgument("roll", default_value="0.0"),
            DeclareLaunchArgument("pitch", default_value="0.0"),
            DeclareLaunchArgument("yaw", default_value="0.0"),
            DeclareLaunchArgument(
                "allow_renaming",
                default_value="true",
                description="Pass model-name collision policy to Gazebo spawn",
            ),
            gazebo,
            start_visualization,
        ]
    )
