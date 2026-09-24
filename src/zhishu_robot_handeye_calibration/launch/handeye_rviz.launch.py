#!/usr/bin/env python3
"""Start only the hand-eye RViz client for a remote real-robot bringup."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    handeye_share = get_package_share_directory(
        "zhishu_robot_handeye_calibration"
    )

    rviz_config_file = LaunchConfiguration("rviz_config_file")
    use_sim_time = LaunchConfiguration("use_sim_time")

    # robot_description, robot_description_semantic, joint states, planning
    # scene, MoveIt actions, and camera images all come from the RK3588.  Do
    # not generate a second local model: it could silently diverge from the
    # hardware launch arguments used by the remote machine.
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="handeye_rviz",
        output="screen",
        arguments=["-d", rviz_config_file],
        parameters=[
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
