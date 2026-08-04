#!/usr/bin/env python3
"""Load the current zhishu_robot controller set into an existing manager."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def spawner(controller_name, controller_manager, timeout, params_file):
    return Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            controller_name,
            "--controller-manager",
            controller_manager,
            "--controller-manager-timeout",
            timeout,
            "--param-file",
            params_file,
        ],
        output="screen",
    )


def generate_launch_description():
    controller_manager = LaunchConfiguration("controller_manager")
    timeout = LaunchConfiguration("controller_manager_timeout")
    params_file = LaunchConfiguration("controller_params_file")

    joint_states = spawner("joint_state_broadcaster", controller_manager, timeout, params_file)
    body_posture = spawner("body_posture_controller", controller_manager, timeout, params_file)
    dual_arm = spawner("dual_arm_controller", controller_manager, timeout, params_file)
    left_gripper = spawner("left_gripper_controller", controller_manager, timeout, params_file)
    right_gripper = spawner("right_gripper_controller", controller_manager, timeout, params_file)

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "controller_manager",
                default_value="/controller_manager",
                description="Controller manager node path",
            ),
            DeclareLaunchArgument(
                "controller_manager_timeout",
                default_value="60",
                description="Seconds to wait for the controller manager",
            ),
            DeclareLaunchArgument(
                "controller_params_file",
                default_value=PathJoinSubstitution(
                    [FindPackageShare("zhishu_robot_moveit_config"), "config", "ros2_controllers.yaml"]
                ),
                description="Controller parameter YAML passed to every Jazzy controller spawner",
            ),
            joint_states,
            RegisterEventHandler(OnProcessExit(target_action=joint_states, on_exit=[body_posture])),
            RegisterEventHandler(OnProcessExit(target_action=body_posture, on_exit=[dual_arm])),
            RegisterEventHandler(OnProcessExit(target_action=dual_arm, on_exit=[left_gripper])),
            RegisterEventHandler(OnProcessExit(target_action=left_gripper, on_exit=[right_gripper])),
        ]
    )
