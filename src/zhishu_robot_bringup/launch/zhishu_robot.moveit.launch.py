#!/usr/bin/env python3
import os

from launch import LaunchDescription
from launch.actions import (
    ExecuteProcess,
    RegisterEventHandler,
)
from launch.event_handlers import OnProcessExit, OnProcessStart
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    # ==============================
    # 包路径查找
    # ==============================
    robot_bringup_share = FindPackageShare("zhishu_robot_bringup").find("zhishu_robot_bringup")
    robot_moveit_config_share = FindPackageShare("zhishu_robot_moveit_config").find("zhishu_robot_moveit_config")

    # ==============================
    # 配置文件路径
    # ==============================
    controllers_yaml = os.path.join(robot_moveit_config_share, "config", "ros2_controllers.yaml")
    rviz_config = os.path.join(robot_moveit_config_share, "config", "moveit.rviz")

    # ==============================
    # MoveIt 配置构建
    # ==============================
    moveit_config = (
        MoveItConfigsBuilder("zhishu_robot", package_name="zhishu_robot_moveit_config")
        .robot_description(file_path="config/zhishu_robot.urdf.xacro")
        .robot_description_semantic(file_path="config/zhishu_robot.srdf")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .joint_limits(file_path="config/joint_limits.yaml")
        .planning_pipelines(pipelines=["ompl", "pilz_industrial_motion_planner"])
        .to_moveit_configs()
    )

    moveit_all_params = moveit_config.to_dict()

    # ==============================
    # 1. robot_state_publisher
    # ==============================
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[moveit_all_params, {"use_sim_time": False}],
    )

    # ==============================
    # 2. ros2_control_node
    # ==============================
    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        output="screen",
        parameters=[moveit_all_params, controllers_yaml, {"use_sim_time": False}],
        additional_env={"LC_NUMERIC": "C"},
    )

    # ==============================
    # 3. 控制器加载器
    # ==============================
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
        output="screen",
    )

    left_arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["left_arm_controller", "--controller-manager", "/controller_manager"],
        output="screen",
    )

    right_arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["right_arm_controller", "--controller-manager", "/controller_manager"],
        output="screen",
    )

    dual_arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["dual_arm_controller", "--controller-manager", "/controller_manager"],
        output="screen",
    )



    right_gripper_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["right_gripper_controller", "--controller-manager", "/controller_manager"],
        output="screen",
    )

    # ==============================
    # 4. move_group
    # ==============================
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            moveit_all_params,
            {"use_sim_time": False},
            {"allow_trajectory_execution": True},
            {"moveit_manage_controllers": True},
        ],
    )

    # ==============================
    # 5. RViz2（修复崩溃 + 移除无效 wait 命令）
    # ==============================
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config],
        parameters=[moveit_all_params, {"use_sim_time": False}],
        additional_env={
            "LC_NUMERIC": "C",
            "QT_QPA_PLATFORM": "xcb",
            "RVIZ_DISABLE_LIBINPUT": "1"  # 防止RViz崩溃
        },
    )

    # ==============================
    # 启动顺序
    # ==============================
    return LaunchDescription([
        robot_state_publisher,
        ros2_control_node,

        joint_state_broadcaster_spawner,
        
        RegisterEventHandler(
            OnProcessExit(target_action=joint_state_broadcaster_spawner,
                          on_exit=[dual_arm_controller_spawner])
        ),
        
        # RegisterEventHandler(
        #     OnProcessExit(target_action=left_arm_controller_spawner,
        #                   on_exit=[right_arm_controller_spawner])
        # ),
        
        RegisterEventHandler(
            OnProcessExit(target_action=dual_arm_controller_spawner,
                          on_exit=[right_gripper_controller_spawner])
        ),

        RegisterEventHandler(
            OnProcessExit(target_action=right_gripper_controller_spawner,
                          on_exit=[move_group_node])
        ),

        # 直接等 move_group 启动后开 RViz，不用 ros2 wait
        RegisterEventHandler(
            OnProcessStart(target_action=right_gripper_controller_spawner,
                          on_start=[rviz_node])
        ),
    ])