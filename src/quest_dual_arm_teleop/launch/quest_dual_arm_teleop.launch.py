#!/usr/bin/env python3
import os
import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, RegisterEventHandler, TimerAction
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def load_yaml(package_name: str, relative_path: str):
    path = os.path.join(get_package_share_directory(package_name), relative_path)
    with open(path, "r", encoding="utf-8") as stream:
        return yaml.safe_load(stream)


def generate_launch_description():
    package_name = "quest_dual_arm_teleop"
    moveit_package = "zhishu_robot_moveit_config"

    package_share = get_package_share_directory(package_name)
    moveit_share = get_package_share_directory(moveit_package)

    launch_rviz = LaunchConfiguration("launch_rviz")
    tcp_port = LaunchConfiguration("tcp_port")
    use_fake_hardware = LaunchConfiguration("use_fake_hardware")

    moveit_config = (
        MoveItConfigsBuilder("zhishu_robot", package_name=moveit_package)
        .robot_description(
            file_path="config/zhishu_robot.urdf.xacro",
            mappings={
                "use_gazebo": "false",
                "use_fake_hardware": use_fake_hardware,
            },
        )
        .robot_description_semantic(file_path="config/zhishu_robot.srdf")
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .joint_limits(file_path="config/joint_limits.yaml")
        .planning_pipelines(pipelines=["ompl", "pilz_industrial_motion_planner"])
        .to_moveit_configs()
    )
    moveit_all_params = moveit_config.to_dict()

    controllers_yaml = os.path.join(moveit_share, "config", "ros2_controllers.yaml")
    rviz_config = os.path.join(moveit_share, "config", "moveit.rviz")
    teleop_config = os.path.join(package_share, "config", "quest_teleop.yaml")
    left_controller_params = os.path.join(package_share, "config", "left_arm_controller.yaml")
    right_controller_params = os.path.join(package_share, "config", "right_arm_controller.yaml")
    moveit_teleop_controllers = load_yaml(package_name, "config/moveit_controllers_teleop.yaml")

    left_servo_params = {"moveit_servo": load_yaml(package_name, "config/left_servo.yaml")}
    right_servo_params = {"moveit_servo": load_yaml(package_name, "config/right_servo.yaml")}

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[moveit_config.robot_description, {"use_sim_time": False}],
    )

    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        output="screen",
        parameters=[controllers_yaml, {"use_sim_time": False}],
        additional_env={"LC_NUMERIC": "C"},
    )

    joint_state_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "-c", "/controller_manager",
            "--param-file", controllers_yaml,
        ],
        output="screen",
    )

    left_arm_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "left_arm_controller",
            "-c", "/controller_manager",
            "--param-file", left_controller_params,
        ],
        output="screen",
    )

    right_arm_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "right_arm_controller",
            "-c", "/controller_manager",
            "--param-file", right_controller_params,
        ],
        output="screen",
    )

    move_group = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        # name="move_group",
        output="screen",
        parameters=[
            moveit_all_params,
            moveit_teleop_controllers,
            {"use_sim_time": False},
            {"allow_trajectory_execution": True},
            {"moveit_manage_controllers": False},
        ],
    )

    left_servo = Node(
        package="moveit_servo",
        executable="servo_node",
        name="left_servo",
        output="screen",
        parameters=[
            left_servo_params,
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            # Deliberately do NOT pass robot_description_kinematics to Servo.
            # move_group/RViz still receive KDL through moveit_all_params above.
            # Without an IK plugin on the Servo node, MoveIt Servo uses its
            # Jacobian pseudo-inverse path for Cartesian Twist commands.
            moveit_config.joint_limits,
        ],
    )

    right_servo = Node(
        package="moveit_servo",
        executable="servo_node",
        name="right_servo",
        output="screen",
        parameters=[
            right_servo_params,
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            # Deliberately do NOT pass robot_description_kinematics to Servo.
            # move_group/RViz still receive KDL through moveit_all_params above.
            # Without an IK plugin on the Servo node, MoveIt Servo uses its
            # Jacobian pseudo-inverse path for Cartesian Twist commands.
            moveit_config.joint_limits,
        ],
    )

    teleop_node = Node(
        package=package_name,
        executable="quest_dual_arm_teleop_node",
        name="quest_dual_arm_teleop",
        output="screen",
        parameters=[teleop_config, {"tcp_port": tcp_port}],
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config],
        parameters=[moveit_all_params, {"use_sim_time": False}],
        condition=IfCondition(launch_rviz),
        additional_env={
            "LC_NUMERIC": "C",
            "QT_QPA_PLATFORM": "xcb",
            "RVIZ_DISABLE_LIBINPUT": "1",
        },
    )

    set_servo_command_type = TimerAction(
        period=3.0,
        actions=[
            ExecuteProcess(
                cmd=[
                    "ros2", "service", "call", "/left_servo/switch_command_type",
                    "moveit_msgs/srv/ServoCommandType", "{command_type: 1}"
                ],
                output="screen",
            ),
            ExecuteProcess(
                cmd=[
                    "ros2", "service", "call", "/right_servo/switch_command_type",
                    "moveit_msgs/srv/ServoCommandType", "{command_type: 1}"
                ],
                output="screen",
            ),
        ],
    )

    after_right_controller = RegisterEventHandler(
        OnProcessExit(
            target_action=right_arm_spawner,
            on_exit=[
                move_group,
                left_servo,
                right_servo,
                teleop_node,
                rviz,
                set_servo_command_type,
            ],
        )
    )

    after_left_controller = RegisterEventHandler(
        OnProcessExit(
            target_action=left_arm_spawner,
            on_exit=[right_arm_spawner],
        )
    )

    after_joint_state = RegisterEventHandler(
        OnProcessExit(
            target_action=joint_state_spawner,
            on_exit=[left_arm_spawner],
        )
    )

    return LaunchDescription([
        DeclareLaunchArgument("launch_rviz", default_value="true"),
        DeclareLaunchArgument("tcp_port", default_value="5005"),
        DeclareLaunchArgument(
            "use_fake_hardware",
            default_value="false",
            description="Use mock joints instead of the EtherCAT arms",
        ),
        robot_state_publisher,
        ros2_control_node,
        after_joint_state,
        after_left_controller,
        after_right_controller,
        joint_state_spawner,
    ])
