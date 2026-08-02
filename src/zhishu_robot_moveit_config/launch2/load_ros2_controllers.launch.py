#!/usr/bin/env python3
"""Load the current zhishu_robot controller set into an existing manager."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def spawner(controller_name, controller_manager, timeout):
    return Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            controller_name,
            "--controller-manager",
            controller_manager,
            "--controller-manager-timeout",
            timeout,
        ],
        output="screen",
    )

def generate_launch_description():
    controller_manager = LaunchConfiguration("controller_manager")
    timeout = LaunchConfiguration("controller_manager_timeout")

    joint_states = spawner(
        "joint_state_broadcaster", controller_manager, timeout
    )
    body_posture = spawner(
        "body_posture_controller", controller_manager, timeout
    )
    dual_arm = spawner("dual_arm_controller", controller_manager, timeout)
    left_gripper = spawner(
        "left_gripper_controller", controller_manager, timeout
    )
    right_gripper = spawner(
        "right_gripper_controller", controller_manager, timeout
    )

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
            joint_states,
            RegisterEventHandler(
                OnProcessExit(
                    target_action=joint_states,
                    on_exit=[body_posture],
                )
            ),
            RegisterEventHandler(
                OnProcessExit(
                    target_action=body_posture,
                    on_exit=[dual_arm],
                )
            ),
            RegisterEventHandler(
                OnProcessExit(
                    target_action=dual_arm,
                    on_exit=[left_gripper],
                )
            ),
            RegisterEventHandler(
                OnProcessExit(
                    target_action=left_gripper,
                    on_exit=[right_gripper],
                )
            ),
        ]
    )
