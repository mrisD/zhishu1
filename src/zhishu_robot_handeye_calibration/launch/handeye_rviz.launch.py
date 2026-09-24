#!/usr/bin/env python3
"""Start only the hand-eye RViz client for a remote real-robot bringup."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    handeye_share = get_package_share_directory(
        "zhishu_robot_handeye_calibration"
    )

    rviz_config_file = LaunchConfiguration("rviz_config_file")
    use_sim_time = LaunchConfiguration("use_sim_time")

    # The robot descriptions still come from the RK3588.  These local-only
    # planning parameters populate the RViz pipeline/planner selectors so a
    # valid planner_id is sent with every remote MoveGroup request.
    moveit_config = (
        MoveItConfigsBuilder(
            "zhishu_robot", package_name="zhishu_robot_moveit_config"
        )
        .robot_description_kinematics(
            file_path="config/kinematics.yaml"
        )
        .joint_limits(file_path="config/joint_limits.yaml")
        .planning_pipelines(
            default_planning_pipeline="ompl",
            pipelines=["ompl", "pilz_industrial_motion_planner"],
        )
        .to_moveit_configs()
    )

    # robot_description, robot_description_semantic, joint states, planning
    # scene, and MoveIt actions come from the RK3588.  Do not pass a second
    # local robot model: it could silently diverge from the remote machine.
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="handeye_rviz",
        output="screen",
        arguments=["-d", rviz_config_file],
        parameters=[
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
            moveit_config.joint_limits,
            {
                "use_sim_time": ParameterValue(
                    use_sim_time, value_type=bool
                )
            }
        ],
        additional_env={
            "LC_NUMERIC": "C",
            "QT_QPA_PLATFORM": "xcb",
            "RVIZ_DISABLE_LIBINPUT": "1",
        },
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "rviz_config_file",
                default_value=os.path.join(
                    handeye_share, "rviz", "handeye.rviz"
                ),
                description="Hand-eye RViz configuration file",
            ),
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="false",
                description=(
                    "Use simulation time; keep false for the RK3588 real robot"
                ),
            ),
            rviz,
        ]
    )
