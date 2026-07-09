#!/usr/bin/env python3
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import RegisterEventHandler, TimerAction
from launch.event_handlers import OnProcessExit, OnProcessStart
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    package_name = "zhishu_robot_moveit_config"
    package_share = get_package_share_directory(package_name)
    controllers_file = os.path.join(package_share, "config", "ros2_controllers.yaml")

    moveit_config = (
        MoveItConfigsBuilder("zhishu_robot", package_name=package_name)
        .robot_description(file_path="config/zhishu_robot.urdf.xacro")
        .robot_description_semantic(file_path="config/zhishu_robot.srdf")
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .joint_limits(file_path="config/joint_limits.yaml")
        .planning_pipelines(pipelines=["ompl"])
        .to_moveit_configs()
    )

    static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="world_to_base",
        arguments=["0", "0", "0", "0", "0", "0", "world", "base_footprint"],
        output="screen",
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[moveit_config.robot_description],
        output="screen",
    )

    ros2_control = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[moveit_config.robot_description, controllers_file],
        output="screen",
    )

    joint_state_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
        output="screen",
    )

    dual_arm_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["dual_arm_controller", "--controller-manager", "/controller_manager"],
        output="screen",
    )

    gripper_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["right_gripper_controller", "--controller-manager", "/controller_manager"],
        output="screen",
    )

    move_group = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        parameters=[
            moveit_config.to_dict(),
            {"allow_trajectory_execution": True},
            {"moveit_manage_controllers": True},
        ],
        output="screen",
    )

    demo = Node(
        package="zhishu_robot_system_tests",
        executable="dual_arm_mock_demo",
        parameters=[
            moveit_config.to_dict(),
            {"target": "both_arms_pose1"},
            {"return_home": True},
            {"velocity_scaling": 0.1},
            {"acceleration_scaling": 0.1},
        ],
        output="screen",
    )

    return LaunchDescription([
        static_tf,
        robot_state_publisher,
        ros2_control,
        joint_state_spawner,
        RegisterEventHandler(
            OnProcessExit(
                target_action=joint_state_spawner,
                on_exit=[dual_arm_spawner],
            )
        ),
        RegisterEventHandler(
            OnProcessExit(
                target_action=dual_arm_spawner,
                on_exit=[gripper_spawner],
            )
        ),
        RegisterEventHandler(
            OnProcessExit(
                target_action=gripper_spawner,
                on_exit=[move_group],
            )
        ),
        RegisterEventHandler(
            OnProcessStart(
                target_action=move_group,
                on_start=[TimerAction(period=5.0, actions=[demo])],
            )
        ),
    ])