#!/usr/bin/env python3
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler, TimerAction
from launch.event_handlers import OnProcessExit, OnProcessStart
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    use_fake_hardware = LaunchConfiguration("use_fake_hardware")
    use_rviz = LaunchConfiguration("use_rviz")
    execute = LaunchConfiguration("execute")
    crate_x = LaunchConfiguration("crate_x")
    crate_y = LaunchConfiguration("crate_y")
    crate_z = LaunchConfiguration("crate_z")
    crate_size_x = LaunchConfiguration("crate_size_x")
    crate_size_y = LaunchConfiguration("crate_size_y")
    crate_size_z = LaunchConfiguration("crate_size_z")
    wall_thickness = LaunchConfiguration("wall_thickness")
    move_x = LaunchConfiguration("move_x")
    move_y = LaunchConfiguration("move_y")
    move_z = LaunchConfiguration("move_z")
    pregrasp_clearance = LaunchConfiguration("pregrasp_clearance")
    grasp_clearance = LaunchConfiguration("grasp_clearance")
    left_tcp_roll = LaunchConfiguration("left_tcp_roll")
    left_tcp_pitch = LaunchConfiguration("left_tcp_pitch")
    left_tcp_yaw = LaunchConfiguration("left_tcp_yaw")
    right_tcp_roll = LaunchConfiguration("right_tcp_roll")
    right_tcp_pitch = LaunchConfiguration("right_tcp_pitch")
    right_tcp_yaw = LaunchConfiguration("right_tcp_yaw")
    rim_grasp_depth = LaunchConfiguration("rim_grasp_depth")
    carry_steps = LaunchConfiguration("carry_steps")
    strict_ik_collision_filter = LaunchConfiguration("strict_ik_collision_filter")
    initial_velocity_scaling = LaunchConfiguration("initial_velocity_scaling")
    initial_acceleration_scaling = LaunchConfiguration("initial_acceleration_scaling")
    planning_attempts = LaunchConfiguration("planning_attempts")
    task_attempts = LaunchConfiguration("task_attempts")
    stage_timeout = LaunchConfiguration("stage_timeout")
    state_wait_timeout = LaunchConfiguration("state_wait_timeout")
    moveit_config_package = "zhishu_robot_moveit_config"
    moveit_share = get_package_share_directory(moveit_config_package)
    controllers_yaml = os.path.join(
        moveit_share, "config", "ros2_controllers.yaml"
    )
    rviz_config = os.path.join(
        moveit_share, "config", "mtc_solution.rviz"
    )

    moveit_config = (
        MoveItConfigsBuilder("zhishu_robot", package_name=moveit_config_package)
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
        .planning_pipelines(
            pipelines=["ompl", "pilz_industrial_motion_planner"]
        )
        .to_moveit_configs()
    )
    moveit_params = moveit_config.to_dict()

    static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="world_to_base_tf",
        output="screen",
        arguments=["0", "0", "0", "0", "0", "0", "world", "base_footprint"],
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[moveit_config.robot_description, {"use_sim_time": False}],
    )

    ros2_control = Node(
        package="controller_manager",
        executable="ros2_control_node",
        prefix=["taskset -c 6"],
        output="screen",
        parameters=[controllers_yaml, {"use_sim_time": False}],
        additional_env={"LC_NUMERIC": "C"},
    )

    def spawner(controller_name):
        return Node(
            package="controller_manager",
            executable="spawner",
            output="screen",
            arguments=[
                controller_name,
                "--controller-manager",
                "/controller_manager",
                "--controller-manager-timeout",
                "120",
                "--param-file",
                controllers_yaml,
            ],
        )

    joint_state_broadcaster = spawner("joint_state_broadcaster")
    dual_arm_controller = spawner("dual_arm_controller")
    left_gripper_controller = spawner("left_gripper_controller")
    right_gripper_controller = spawner("right_gripper_controller")

    move_group = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        # name="move_group",
        output="screen",
        parameters=[
            moveit_params,
            {"use_sim_time": False},
            {"allow_trajectory_execution": True},
            {"moveit_manage_controllers": True},
            {"capabilities": "move_group/ExecuteTaskSolutionCapability"},
        ],
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        # name="rviz2",
        output="screen",
        arguments=["-d", rviz_config],
        parameters=[moveit_params, {"use_sim_time": False}],
        additional_env={
            "LC_NUMERIC": "C",
            "QT_QPA_PLATFORM": "xcb",
            "RVIZ_DISABLE_LIBINPUT": "1",
        },
        condition=IfCondition(use_rviz),
    )

    mtc_node = Node(
        package="zhishu_robot_mtc_pick_place",
        executable="dual_arm_crate_carry",
        # name="dual_arm_crate_carry",
        output="screen",
        parameters=[
            moveit_params,
            {
                "use_sim_time": False,
                "world_frame": "base_link",
                "crate_x": ParameterValue(crate_x, value_type=float),
                "crate_y": ParameterValue(crate_y, value_type=float),
                "crate_z": ParameterValue(crate_z, value_type=float),
                "crate_size_x": ParameterValue(crate_size_x, value_type=float),
                "crate_size_y": ParameterValue(crate_size_y, value_type=float),
                "crate_size_z": ParameterValue(crate_size_z, value_type=float),
                "wall_thickness": ParameterValue(wall_thickness, value_type=float),
                "move_x": ParameterValue(move_x, value_type=float),
                "move_y": ParameterValue(move_y, value_type=float),
                "move_z": ParameterValue(move_z, value_type=float),
                "pregrasp_clearance": ParameterValue(
                    pregrasp_clearance, value_type=float
                ),
                "grasp_clearance": ParameterValue(grasp_clearance, value_type=float),
                "left_tcp_roll": ParameterValue(left_tcp_roll, value_type=float),
                "left_tcp_pitch": ParameterValue(left_tcp_pitch, value_type=float),
                "left_tcp_yaw": ParameterValue(left_tcp_yaw, value_type=float),
                "right_tcp_roll": ParameterValue(right_tcp_roll, value_type=float),
                "right_tcp_pitch": ParameterValue(right_tcp_pitch, value_type=float),
                "right_tcp_yaw": ParameterValue(right_tcp_yaw, value_type=float),
                "rim_grasp_depth": ParameterValue(rim_grasp_depth, value_type=float),
                "carry_steps": ParameterValue(carry_steps, value_type=int),
                "strict_ik_collision_filter": ParameterValue(
                    strict_ik_collision_filter, value_type=bool
                ),
                "execute": ParameterValue(execute, value_type=bool),
                "max_solutions": 5,
                "planning_attempts": ParameterValue(
                    planning_attempts, value_type=int
                ),
                "task_attempts": ParameterValue(task_attempts, value_type=int),
                "stage_timeout": ParameterValue(stage_timeout, value_type=float),
                "state_wait_timeout": ParameterValue(
                    state_wait_timeout, value_type=float
                ),
                "planner_id": "RRTConnectkConfigDefault",
                "initial_velocity_scaling": ParameterValue(
                    initial_velocity_scaling, value_type=float
                ),
                "initial_acceleration_scaling": ParameterValue(
                    initial_acceleration_scaling, value_type=float
                ),
                "velocity_scaling": 0.20,
                "acceleration_scaling": 0.20,
            },
        ],
    )

    # Controllers are activated sequentially. The individual arm controllers
    # must not be active together with dual_arm_controller because they claim
    # the same command interfaces.
    start_dual_arm = RegisterEventHandler(
        OnProcessExit(
            target_action=joint_state_broadcaster,
            on_exit=[dual_arm_controller],
        )
    )
    start_left_gripper = RegisterEventHandler(
        OnProcessExit(
            target_action=dual_arm_controller,
            on_exit=[left_gripper_controller],
        )
    )
    start_right_gripper = RegisterEventHandler(
        OnProcessExit(
            target_action=left_gripper_controller,
            on_exit=[right_gripper_controller],
        )
    )
    start_move_group = RegisterEventHandler(
        OnProcessExit(
            target_action=right_gripper_controller,
            on_exit=[move_group],
        )
    )
    start_ui_and_task = RegisterEventHandler(
        OnProcessStart(
            target_action=move_group,
            on_start=[
                TimerAction(period=3.0, actions=[rviz]),
                # Reduce startup contention. The MTC node also performs a
                # bounded readiness check instead of relying on this delay.
                TimerAction(period=8.0, actions=[mtc_node]),
            ],
        )
    )

    return LaunchDescription([
        DeclareLaunchArgument("execute", default_value="false"),
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
        # Tuned demo defaults. The original 20 cm cube can still be used with
        # crate_size_x:=0.20 crate_size_y:=0.20 crate_size_z:=0.20.
        DeclareLaunchArgument("crate_x", default_value="0.42"),
        DeclareLaunchArgument("crate_y", default_value="0.0"),
        DeclareLaunchArgument("crate_z", default_value="0.6"),
        DeclareLaunchArgument("crate_size_x", default_value="0.17"),
        DeclareLaunchArgument("crate_size_y", default_value="0.35"),
        DeclareLaunchArgument("crate_size_z", default_value="0.20"),
        DeclareLaunchArgument("wall_thickness", default_value="0.01"),
        DeclareLaunchArgument("move_x", default_value="0.12"),
        DeclareLaunchArgument("move_y", default_value="0.0"),
        DeclareLaunchArgument("move_z", default_value="0.28"),
        DeclareLaunchArgument("pregrasp_clearance", default_value="0.14"),
        DeclareLaunchArgument("grasp_clearance", default_value="0.0"),
        DeclareLaunchArgument("left_tcp_roll", default_value="-2.09439510239"),
        DeclareLaunchArgument("left_tcp_pitch", default_value="0.0"),
        DeclareLaunchArgument("left_tcp_yaw", default_value="1.57079632679"),
        DeclareLaunchArgument("right_tcp_roll", default_value="-1.04719755120"),
        DeclareLaunchArgument("right_tcp_pitch", default_value="0.0"),
        DeclareLaunchArgument("right_tcp_yaw", default_value="-1.57079632679"),
        DeclareLaunchArgument("rim_grasp_depth", default_value="0.02"),
        DeclareLaunchArgument("carry_steps", default_value="15"),
        DeclareLaunchArgument("strict_ik_collision_filter", default_value="false"),
        DeclareLaunchArgument("initial_velocity_scaling", default_value="0.25"),
        DeclareLaunchArgument("initial_acceleration_scaling", default_value="0.25"),
        # PipelinePlanner defaults to one attempt. Multiple OMPL attempts plus
        # complete-task retries make the stochastic dual-arm plan robust.
        DeclareLaunchArgument("planning_attempts", default_value="8"),
        DeclareLaunchArgument("task_attempts", default_value="3"),
        DeclareLaunchArgument("stage_timeout", default_value="15.0"),
        DeclareLaunchArgument("state_wait_timeout", default_value="30.0"),
        static_tf,
        robot_state_publisher,
        ros2_control,
        joint_state_broadcaster,
        start_dual_arm,
        start_left_gripper,
        start_right_gripper,
        start_move_group,
        start_ui_and_task,
    ])
