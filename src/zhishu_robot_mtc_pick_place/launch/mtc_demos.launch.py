#!/usr/bin/env python3
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler, TimerAction
from launch.event_handlers import OnProcessExit, OnProcessStart
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    use_fake_hardware = LaunchConfiguration("use_fake_hardware")
    use_rviz = LaunchConfiguration("use_rviz")
    robot_name = "zhishu_robot"
    moveit_config_package = "zhishu_robot_moveit_config"
    mtc_package = "zhishu_robot_mtc_pick_place"

    moveit_config_share = get_package_share_directory(moveit_config_package)

    ros2_controllers_yaml = os.path.join(
        moveit_config_share,
        "config",
        "ros2_controllers.yaml",
    )

    rviz_config = os.path.join(
        moveit_config_share,
        "config",
        "mtc_solution.rviz",
    )

    moveit_config = (
        MoveItConfigsBuilder(robot_name, package_name=moveit_config_package)
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

    static_tf_node = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_publisher_world_base",
        output="screen",
        arguments=["0", "0", "0", "0", "0", "0", "world", "base_footprint"],
    )

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
        prefix=["taskset -c 6"],
        output="screen",
        parameters=[ros2_controllers_yaml, {"use_sim_time": False}],
        additional_env={"LC_NUMERIC": "C"},
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager", "/controller_manager",
            "--param-file", ros2_controllers_yaml,
            "--controller-manager-timeout", "120",
        ],
        output="screen",
    )

    left_arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "left_arm_controller",
            "--controller-manager", "/controller_manager",
            "--param-file", ros2_controllers_yaml,
            "--controller-manager-timeout", "120",
        ],
        output="screen",
    )

    right_arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "right_arm_controller",
            "--controller-manager", "/controller_manager",
            "--param-file", ros2_controllers_yaml,
            "--controller-manager-timeout", "120",
        ],
        output="screen",
    )

    dual_arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "dual_arm_controller",
            "--controller-manager", "/controller_manager",
            "--param-file", ros2_controllers_yaml,
            "--controller-manager-timeout", "120",
        ],
        output="screen",
    )

    left_gripper_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "left_gripper_controller",
            "--controller-manager", "/controller_manager",
            "--param-file", ros2_controllers_yaml,
            "--controller-manager-timeout", "120",
        ],
        output="screen",
    )

    right_gripper_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "right_gripper_controller",
            "--controller-manager", "/controller_manager",
            "--param-file", ros2_controllers_yaml,
            "--controller-manager-timeout", "120",
        ],
        output="screen",
    )

    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        name="move_group",
        output="screen",
        parameters=[
            moveit_all_params,
            {"use_sim_time": False},
            {"allow_trajectory_execution": True},
            {"moveit_manage_controllers": True},
            {"capabilities": "move_group/ExecuteTaskSolutionCapability"},
        ],
    )

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
            "RVIZ_DISABLE_LIBINPUT": "1",
        },
        condition=IfCondition(use_rviz),
    )

    mtc_task_node = Node(
        package=mtc_package,
        executable="mtc_task_node",
        name="mtc_node",
        output="screen",
        parameters=[
            moveit_all_params,
            {"use_sim_time": False},

            {"arm_group_name": "right_arm"},
            {"gripper_group_name": "right_gripper"},
            {"end_effector_name": "right_arm_eef"},
            {"gripper_frame": "right_gripper_tcp"},
            {"gripper_open_pose": "right_gripper_opened"},
            {"gripper_close_pose": "right_gripper_closed"},
            {"arm_home_pose": "right_arm_home"},
            {"controller_names": ["dual_arm_controller", "right_gripper_controller"]},

            # Scene/object parameters. Tune these according to the real table/object pose.
            {"world_frame": "base_link"},
            {"object_reference_frame": "base_link"},
            {"object_name": "object"},
            {"object_type": "cylinder"},
            {"object_dimensions": [0.35, 0.0125]},
            # {"object_pose": [0.35, 0.35, 0.85, 0.0, 0.0, 0.0]},
            # {"object_pose": [0.63, -0.06, 1.28, 0.0, 0.0, 0.0]},
            # {"place_pose": [0.63, -0.2, 1.10, 0.0, 0.0, 0.0]},
            {"object_pose": [0.634, -0.064, 1.28, 1.5708, 0.0, 0.0]},
            # Z的数值加上了瓶子高度的一半，所以需要剪掉
            {"place_pose": [0.634, -0.2, 1.1, 1.5708, 0.0, 0.0]},

            # # TCP offset from gripper_base_link to the grasp point between fingers.
            {"grasp_frame_transform": [0.0, 0.0, 0.0, 0.0, 0.0, 1.5708]}, #TCP            # {"grasp_frame_transform": [0.0, 0.0, 0.06, 1.5708, 0.7854, 3.14]}, #TCP

            # Conservative first test.
            {"execute": False},
            {"max_solutions": 10},
            {"grasp_pose_angle_delta": 0.1309}, # 7.5 degrees
            {"grasp_pose_max_ik_solutions": 50},
            {"cartesian_step_size": 0.005},
            {"cartesian_max_velocity_scaling": 0.3},
            {"cartesian_max_acceleration_scaling": 0.3},

            {"approach_object_min_dist": 0.005},
            {"approach_object_max_dist": 0.06},
            {"lift_object_min_dist": 0.005},
            {"lift_object_max_dist": 0.08},
            {"lower_object_min_dist": 0.005},
            {"lower_object_max_dist": 0.08},
            {"retreat_min_distance": 0.005},
            {"retreat_max_distance": 0.08},
            
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "use_fake_hardware",
            default_value="false",
            description="Use mock joints instead of the EtherCAT arms",
        ),
        DeclareLaunchArgument(
            "use_rviz",
            default_value="true",
            description="Start RViz2",
        ),
        static_tf_node,
        robot_state_publisher,
        ros2_control_node,
        joint_state_broadcaster_spawner,

        RegisterEventHandler(
            OnProcessExit(
                target_action=joint_state_broadcaster_spawner,
                on_exit=[dual_arm_controller_spawner],
            )
        ),

        # RegisterEventHandler(
        #     OnProcessExit(
        #         target_action=left_arm_controller_spawner,
        #         on_exit=[right_arm_controller_spawner],
        #     )
        # ),



        RegisterEventHandler(
            OnProcessExit(
                target_action=dual_arm_controller_spawner,
                on_exit=[left_gripper_controller_spawner],
            )
        ),

        RegisterEventHandler(
            OnProcessExit(
                target_action=left_gripper_controller_spawner,
                on_exit=[right_gripper_controller_spawner],
            )
        ),

        RegisterEventHandler(
            OnProcessExit(
                target_action=right_gripper_controller_spawner,
                on_exit=[move_group_node],
            )
        ),

        RegisterEventHandler(
            OnProcessStart(
                target_action=move_group_node,
                on_start=[
                    TimerAction(period=3.0, actions=[rviz_node]),
                    TimerAction(period=10.0, actions=[mtc_task_node]),
                ],
            )
        ),
    ])
