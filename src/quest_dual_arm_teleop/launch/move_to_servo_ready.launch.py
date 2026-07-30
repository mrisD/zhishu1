#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    moveit_package = "zhishu_robot_moveit_config"

    moveit_config = (
        MoveItConfigsBuilder("zhishu_robot", package_name=moveit_package)
        .robot_description(file_path="config/zhishu_robot.urdf.xacro")
        .robot_description_semantic(file_path="config/zhishu_robot.srdf")
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .joint_limits(file_path="config/joint_limits.yaml")
        .to_moveit_configs()
    )

    move_to_ready = Node(
        package="quest_dual_arm_teleop",
        executable="move_to_servo_ready",
        name="move_to_servo_ready",
        output="screen",
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.joint_limits,
            {"use_sim_time": False},
        ],
    )

    return LaunchDescription([move_to_ready])
