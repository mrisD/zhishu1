#!/usr/bin/env python3
"""Start Gazebo, two MoveIt Servo instances, and cooperative admittance."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, ExecuteProcess, LogInfo
from launch.actions import IncludeLaunchDescription, RegisterEventHandler, TimerAction
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from moveit_configs_utils import MoveItConfigsBuilder
import yaml


def load_yaml(package_name, relative_path):
    path = os.path.join(get_package_share_directory(package_name), relative_path)
    with open(path, "r", encoding="utf-8") as stream:
        return yaml.safe_load(stream)


def continue_if_success(next_action, description):
    """Continue a startup chain only when the previous process succeeded."""

    def on_exit(event, context):
        del context
        if event.returncode == 0:
            return next_action if isinstance(next_action, list) else [next_action]
        reason = f"{description} failed with exit code {event.returncode}"
        return [LogInfo(msg=f"ERROR: {reason}"), EmitEvent(event=Shutdown(reason=reason))]

    return on_exit


def generate_launch_description():
    package_name = "zhishu_robot_admittance"
    moveit_package = "zhishu_robot_moveit_config"
    gazebo_package = "zhishu_robot_gazebo"

    package_share = get_package_share_directory(package_name)
    moveit_share = get_package_share_directory(moveit_package)
    gazebo_share = get_package_share_directory(gazebo_package)

    headless = LaunchConfiguration("headless")
    use_rviz = LaunchConfiguration("use_rviz")
    enable_on_start = LaunchConfiguration("enable_on_start")
    force_x11 = LaunchConfiguration("force_x11")
    render_engine_gui = LaunchConfiguration("render_engine_gui")
    software_rendering = LaunchConfiguration("software_rendering")
    initial_positions = os.path.join(
        package_share, "config", "box_admittance_initial_positions.yaml"
    )

    moveit_config = (
        MoveItConfigsBuilder("zhishu_robot", package_name=moveit_package)
        .robot_description(
            file_path="config/zhishu_robot.urdf.xacro",
            mappings={
                "use_gazebo": "true",
                "use_fake_hardware": "false",
                "initial_positions_file": initial_positions,
            },
        )
        .robot_description_semantic(file_path="config/zhishu_robot.srdf")
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .joint_limits(file_path="config/joint_limits.yaml")
        .planning_pipelines(pipelines=["ompl", "pilz_industrial_motion_planner"])
        .to_moveit_configs()
    )
    moveit_params = moveit_config.to_dict()
    controllers_yaml = os.path.join(moveit_share, "config", "ros2_controllers.yaml")
    admittance_yaml = os.path.join(package_share, "config", "admittance.yaml")

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_share, "launch", "zhishu_robot.gazebo.launch.py")
        ),
        launch_arguments={
            "headless": headless,
            "force_x11": force_x11,
            "render_engine_gui": render_engine_gui,
            "software_rendering": software_rendering,
            "paused": "true",
            "world_file": "dual_arm_box_admittance.sdf",
            "initial_positions_file": initial_positions,
            "load_controllers": "false",
            "use_gazebo": "true",
            "use_robot_state_pub": "true",
            "use_sim_time": "true",
        }.items(),
    )

    def controller_spawner(controller_name):
        return Node(
            package="controller_manager",
            executable="spawner",
            output="screen",
            arguments=[
                controller_name,
                "--controller-manager",
                "/controller_manager",
                "--controller-manager-timeout",
                "60",
                "--param-file",
                controllers_yaml,
                "--inactive",
            ],
        )

    joint_state_spawner = controller_spawner("joint_state_broadcaster")
    body_posture_spawner = controller_spawner("body_posture_controller")
    left_arm_spawner = controller_spawner("left_arm_controller")
    right_arm_spawner = controller_spawner("right_arm_controller")
    left_gripper_spawner = controller_spawner("left_gripper_controller")
    right_gripper_spawner = controller_spawner("right_gripper_controller")

    move_group = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[moveit_params, {"use_sim_time": True}],
    )

    left_servo_params = {
        "moveit_servo": load_yaml("quest_dual_arm_teleop", "config/left_servo.yaml")
    }
    right_servo_params = {
        "moveit_servo": load_yaml("quest_dual_arm_teleop", "config/right_servo.yaml")
    }
    servo_common_parameters = [
        moveit_config.robot_description,
        moveit_config.robot_description_semantic,
        moveit_config.robot_description_kinematics,
        moveit_config.joint_limits,
        {"use_sim_time": True},
    ]
    left_servo = Node(
        package="moveit_servo",
        executable="servo_node",
        name="left_servo",
        output="screen",
        parameters=[left_servo_params] + servo_common_parameters,
    )
    right_servo = Node(
        package="moveit_servo",
        executable="servo_node",
        name="right_servo",
        output="screen",
        parameters=[right_servo_params] + servo_common_parameters,
    )

    admittance = Node(
        package=package_name,
        executable="dual_arm_admittance_controller",
        name="dual_arm_admittance",
        output="screen",
        parameters=[
            admittance_yaml,
            {"use_sim_time": True},
            {"enable_on_start": ParameterValue(enable_on_start, value_type=bool)},
        ],
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        output="screen",
        arguments=["-d", os.path.join(moveit_share, "config", "moveit.rviz")],
        parameters=[moveit_params, {"use_sim_time": True}],
        condition=IfCondition(use_rviz),
    )

    switch_servo_to_twist = TimerAction(
        period=2.0,
        actions=[
            ExecuteProcess(
                cmd=[
                    "ros2",
                    "service",
                    "call",
                    "/left_servo/switch_command_type",
                    "moveit_msgs/srv/ServoCommandType",
                    "{command_type: 1}",
                ],
                output="screen",
            ),
            ExecuteProcess(
                cmd=[
                    "ros2",
                    "service",
                    "call",
                    "/right_servo/switch_command_type",
                    "moveit_msgs/srv/ServoCommandType",
                    "{command_type: 1}",
                ],
                output="screen",
            ),
        ],
    )
    start_admittance = TimerAction(period=2.5, actions=[admittance])

    unpause_gazebo = ExecuteProcess(
        cmd=[
            "gz",
            "service",
            "-s",
            "/world/admittance_box/control",
            "--reqtype",
            "gz.msgs.WorldControl",
            "--reptype",
            "gz.msgs.Boolean",
            "--timeout",
            "5000",
            "--req",
            "pause: false",
        ],
        output="screen",
    )
    activate_controllers = Node(
        package=package_name,
        executable="activate_admittance_controllers",
        output="screen",
    )
    after_joint_state = RegisterEventHandler(
        OnProcessExit(
            target_action=joint_state_spawner,
            on_exit=continue_if_success(
                body_posture_spawner, "joint_state_broadcaster load"
            ),
        )
    )
    after_body_posture = RegisterEventHandler(
        OnProcessExit(
            target_action=body_posture_spawner,
            on_exit=continue_if_success(left_arm_spawner, "body posture controller load"),
        )
    )
    after_left_arm = RegisterEventHandler(
        OnProcessExit(
            target_action=left_arm_spawner,
            on_exit=continue_if_success(right_arm_spawner, "left arm controller load"),
        )
    )
    after_right_arm = RegisterEventHandler(
        OnProcessExit(
            target_action=right_arm_spawner,
            on_exit=continue_if_success(left_gripper_spawner, "right arm controller load"),
        )
    )
    after_left_gripper = RegisterEventHandler(
        OnProcessExit(
            target_action=left_gripper_spawner,
            on_exit=continue_if_success(
                right_gripper_spawner, "left gripper controller load"
            ),
        )
    )
    unpause_after_loading = RegisterEventHandler(
        OnProcessExit(
            target_action=right_gripper_spawner,
            on_exit=continue_if_success(
                unpause_gazebo, "right gripper controller load"
            ),
        )
    )
    activate_after_unpause = RegisterEventHandler(
        OnProcessExit(
            target_action=unpause_gazebo,
            on_exit=continue_if_success(activate_controllers, "Gazebo unpause"),
        )
    )
    start_nodes_after_activation = RegisterEventHandler(
        OnProcessExit(
            target_action=activate_controllers,
            on_exit=continue_if_success(
                [
                    move_group,
                    left_servo,
                    right_servo,
                    rviz,
                    switch_servo_to_twist,
                    start_admittance,
                ],
                "controller activation",
            ),
        )
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "headless", default_value="false", description="Run Gazebo without GUI"
            ),
            DeclareLaunchArgument(
                "use_rviz", default_value="true", description="Start RViz"
            ),
            DeclareLaunchArgument(
                "force_x11",
                default_value="true",
                description="Use the X11 Qt backend to avoid VMware/Wayland flicker",
            ),
            DeclareLaunchArgument(
                "render_engine_gui",
                default_value="ogre",
                description="Gazebo GUI engine; ogre is the VMware-compatible default",
            ),
            DeclareLaunchArgument(
                "software_rendering",
                default_value="true",
                description="Use Mesa software OpenGL for a stable virtual-machine GUI",
            ),
            DeclareLaunchArgument(
                "enable_on_start",
                default_value="false",
                description="Enable compliant motion immediately after automatic tare",
            ),
            gazebo,
            after_joint_state,
            after_body_posture,
            after_left_arm,
            after_right_arm,
            after_left_gripper,
            unpause_after_loading,
            activate_after_unpause,
            start_nodes_after_activation,
            joint_state_spawner,
        ]
    )
