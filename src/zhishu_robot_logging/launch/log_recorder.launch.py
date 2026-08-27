#!/usr/bin/env python3

from datetime import datetime
import os
from pathlib import Path
import re

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    OpaqueFunction,
    SetLaunchConfiguration,
    SetEnvironmentVariable,
)
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackagePrefix


def _default_log_root():
    """Return <workspace>/zhishu_robot_logs beside colcon's log dir."""
    package_share = Path(
        get_package_share_directory("zhishu_robot_logging")
    ).resolve()
    for parent in package_share.parents:
        if parent.name == "install":
            return parent.parent / "zhishu_robot_logs"
    return Path.cwd() / "zhishu_robot_logs"


def _configure_native_log_directory(context):
    configured_root = LaunchConfiguration("log_root").perform(context).strip()
    root = (
        Path(configured_root).expanduser()
        if configured_root
        else _default_log_root()
    )
    session_name = LaunchConfiguration("log_session").perform(context).strip()
    if session_name in {".", ".."} or not re.fullmatch(
        r"[A-Za-z0-9_.-]+", session_name
    ):
        raise ValueError(
            "log_session must contain only letters, digits, '.', '_' or '-'"
        )
    native_directory = (
        root / datetime.now().strftime("%Y-%m-%d") / session_name / "native"
    )
    native_directory.mkdir(parents=True, exist_ok=True)
    return [
        SetLaunchConfiguration("log_root", str(root)),
        SetEnvironmentVariable("ROS_LOG_DIR", str(native_directory)),
    ]


def generate_launch_description():
    session_started_at = datetime.now().strftime("%H-%M-%S")
    return LaunchDescription([
        DeclareLaunchArgument(
            "log_root",
            default_value=os.environ.get(
                "ZHISHU_ROBOT_LOG_DIR", str(_default_log_root())
            ),
            description=(
                "Root directory; defaults to <workspace>/zhishu_robot_logs "
                "beside build, install, and log"
            ),
        ),
        DeclareLaunchArgument(
            "log_session",
            default_value=session_started_at,
            description="Per-launch directory name; defaults to start time HH-MM-SS",
        ),
        DeclareLaunchArgument(
            "max_file_size_mb",
            default_value="10",
            description="Rotate the active log after this many MiB",
        ),
        DeclareLaunchArgument(
            "min_severity",
            default_value="10",
            description="Minimum rosgraph severity: DEBUG=10 INFO=20 WARN=30 ERROR=40 FATAL=50",
        ),
        DeclareLaunchArgument(
            "enable_kernel_ethercat_log",
            default_value="true",
            description="Record new filtered kernel EtherCAT messages",
        ),
        OpaqueFunction(function=_configure_native_log_directory),
        Node(
            package="zhishu_robot_logging",
            executable="zhishu_robot_log_recorder",
            name="zhishu_robot_log_recorder",
            output="screen",
            parameters=[{
                "log_root": ParameterValue(
                    LaunchConfiguration("log_root"), value_type=str
                ),
                "session_name": ParameterValue(
                    LaunchConfiguration("log_session"), value_type=str
                ),
                "max_file_size_mb": ParameterValue(
                    LaunchConfiguration("max_file_size_mb"), value_type=int
                ),
                "min_severity": ParameterValue(
                    LaunchConfiguration("min_severity"), value_type=int
                ),
            }],
        ),
        ExecuteProcess(
            cmd=[
                PathJoinSubstitution([
                    FindPackagePrefix("zhishu_robot_logging"),
                    "lib",
                    "zhishu_robot_logging",
                    "kernel_ethercat_logger",
                ]),
                "--log-root", LaunchConfiguration("log_root"),
                "--session-name", LaunchConfiguration("log_session"),
            ],
            output="screen",
            condition=IfCondition(LaunchConfiguration("enable_kernel_ethercat_log")),
        ),
    ])
