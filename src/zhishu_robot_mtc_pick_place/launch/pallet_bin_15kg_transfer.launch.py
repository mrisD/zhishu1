#!/usr/bin/env python3
"""15 kg pallet coverage + mobile-base transfer into the STEP-sized large bin."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    headless = LaunchConfiguration("headless")
    use_rviz = LaunchConfiguration("use_rviz")
    execute = LaunchConfiguration("execute")
    validate_all_nine = LaunchConfiguration("validate_all_nine")
    full_transfer_box_index = LaunchConfiguration("full_transfer_box_index")
    pre_extract_enabled = LaunchConfiguration("pre_extract_enabled")
    skip_pre_extract_stage = LaunchConfiguration("skip_pre_extract_stage")
    pre_extract_box_index = LaunchConfiguration("pre_extract_box_index")
    continue_after_pre_extract_plan = LaunchConfiguration("continue_after_pre_extract_plan")
    pre_extract_distance = LaunchConfiguration("pre_extract_distance")
    pre_extract_waist_lift = LaunchConfiguration("pre_extract_waist_lift")
    pre_extract_waist_pitch = LaunchConfiguration("pre_extract_waist_pitch")
    stop_on_failure = LaunchConfiguration("stop_on_failure")
    hook_hole_z_offset = LaunchConfiguration("hook_hole_z_offset")
    hook_hole_inset = LaunchConfiguration("hook_hole_inset")
    hook_tcp_hole_clearance = LaunchConfiguration("hook_tcp_hole_clearance")
    hook_approach_height = LaunchConfiguration("hook_approach_height")
    hook_approach_outset = LaunchConfiguration("hook_approach_outset")
    hook_seat_drop = LaunchConfiguration("hook_seat_drop")
    handle_half_span = LaunchConfiguration("handle_half_span")
    grasp_half_span = LaunchConfiguration("grasp_half_span")
    handle_z_offset = LaunchConfiguration("handle_z_offset")
    grasp_waist_pitch = LaunchConfiguration("grasp_waist_pitch")
    grasp_waist_lift = LaunchConfiguration("grasp_waist_lift")
    left_tcp_roll = LaunchConfiguration("left_tcp_roll")
    left_tcp_pitch = LaunchConfiguration("left_tcp_pitch")
    left_tcp_yaw = LaunchConfiguration("left_tcp_yaw")
    right_tcp_roll = LaunchConfiguration("right_tcp_roll")
    right_tcp_pitch = LaunchConfiguration("right_tcp_pitch")
    right_tcp_yaw = LaunchConfiguration("right_tcp_yaw")
    grasp_x_offset = LaunchConfiguration("grasp_x_offset")
    pregrasp_clearance = LaunchConfiguration("pregrasp_clearance")
    dual_ik_attempts = LaunchConfiguration("dual_ik_attempts")
    dual_ik_timeout = LaunchConfiguration("dual_ik_timeout")
    source_grasp_x = LaunchConfiguration("source_grasp_x")
    source_grasp_y = LaunchConfiguration("source_grasp_y")
    swap_arm_grasps = LaunchConfiguration("swap_arm_grasps")

    gazebo_share = get_package_share_directory("zhishu_robot_gazebo")
    moveit_config_package = "zhishu_robot_moveit_config"

    moveit_config = (
        MoveItConfigsBuilder("zhishu_robot", package_name=moveit_config_package)
        .robot_description(
            file_path="config/zhishu_robot.urdf.xacro",
            mappings={
                "use_gazebo": "true",
                "use_fake_hardware": "false",
                "mobile_base_sim_enabled": "true",
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

    gazebo_moveit = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                gazebo_share, "launch", "zhishu_robot.gazebo_moveit.launch.py"
            )
        ),
        launch_arguments={
            "headless": headless,
            "use_rviz": use_rviz,
            "world_file": "pallet_bin_15kg.sdf",
            "use_left_arm_controller": "false",
            "mobile_base_sim_enabled": "true",
            # Start at the calibrated arm work station.  The first tote is
            # initially presented at the 20 cm front station in the world;
            # this x offset is the short final dock needed for the oblique
            # hook pose to remain inside the right-arm IK workspace.
            "x": "0.190",
            "y": "-0.400617",
            # The wheel joint centres are already 82.5 mm above base_footprint
            # and the wheel radius is 85.8 mm.  The base collision bottom is
            # z=0 in that frame, so the settled spawn height is only 3.3 mm;
            # 90 mm would leave the whole robot floating and it then falls,
            # corrupting the initial odometry/yaw before docking.
            "z": "0.004",
            "yaw": "0.0",
            "allow_renaming": "false",
        }.items(),
    )

    transfer = Node(
        package="zhishu_robot_mtc_pick_place",
        executable="pallet_bin_15kg_transfer",
        name="pallet_bin_15kg_transfer",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            {
                "use_sim_time": True,
                "execute": ParameterValue(execute, value_type=bool),
                "validate_all_nine": ParameterValue(
                    validate_all_nine, value_type=bool
                ),
                "full_transfer_box_index": ParameterValue(
                    full_transfer_box_index, value_type=int
                ),
                "pre_extract_enabled": ParameterValue(
                    pre_extract_enabled, value_type=bool
                ),
                "skip_pre_extract_stage": ParameterValue(
                    skip_pre_extract_stage, value_type=bool
                ),
                "pre_extract_box_index": ParameterValue(
                    pre_extract_box_index, value_type=int
                ),
                "continue_after_pre_extract_plan": ParameterValue(
                    continue_after_pre_extract_plan, value_type=bool
                ),
                "pre_extract_distance": ParameterValue(
                    pre_extract_distance, value_type=float
                ),
                "pre_extract_waist_lift": ParameterValue(
                    pre_extract_waist_lift, value_type=float
                ),
                "pre_extract_waist_pitch": ParameterValue(
                    pre_extract_waist_pitch, value_type=float
                ),
                "stop_on_failure": ParameterValue(
                    stop_on_failure, value_type=bool
                ),
                "hook_hole_z_offset": ParameterValue(
                    hook_hole_z_offset, value_type=float
                ),
                "hook_hole_inset": ParameterValue(
                    hook_hole_inset, value_type=float
                ),
                "hook_tcp_hole_clearance": ParameterValue(
                    hook_tcp_hole_clearance, value_type=float
                ),
                "hook_approach_height": ParameterValue(
                    hook_approach_height, value_type=float
                ),
                "hook_approach_outset": ParameterValue(
                    hook_approach_outset, value_type=float
                ),
                "hook_seat_drop": ParameterValue(
                    hook_seat_drop, value_type=float
                ),
                "handle_half_span": ParameterValue(
                    handle_half_span, value_type=float
                ),
                "grasp_half_span": ParameterValue(
                    grasp_half_span, value_type=float
                ),
                "handle_z_offset": ParameterValue(
                    handle_z_offset, value_type=float
                ),
                "grasp_x_offset": ParameterValue(
                    grasp_x_offset, value_type=float
                ),
                "dual_ik_attempts": ParameterValue(
                    dual_ik_attempts, value_type=int
                ),
                "dual_ik_timeout": ParameterValue(
                    dual_ik_timeout, value_type=float
                ),
                "pregrasp_clearance": ParameterValue(
                    pregrasp_clearance, value_type=float
                ),
                "source_grasp_x": ParameterValue(source_grasp_x, value_type=float),
                "source_grasp_y": ParameterValue(source_grasp_y, value_type=float),
                "swap_arm_grasps": ParameterValue(swap_arm_grasps, value_type=bool),
                "left_tcp_roll": ParameterValue(left_tcp_roll, value_type=float),
                "left_tcp_pitch": ParameterValue(left_tcp_pitch, value_type=float),
                "left_tcp_yaw": ParameterValue(left_tcp_yaw, value_type=float),
                "right_tcp_roll": ParameterValue(right_tcp_roll, value_type=float),
                "right_tcp_pitch": ParameterValue(right_tcp_pitch, value_type=float),
                "right_tcp_yaw": ParameterValue(right_tcp_yaw, value_type=float),
                # Conservative 15 kg settings. Increase only after the 50 mm
                # load test is stable and wrist F/T feedback is reasonable.
                "velocity_scaling": 0.12,
                "acceleration_scaling": 0.10,
                "grasp_waist_lift": ParameterValue(
                    grasp_waist_lift, value_type=float
                ),
                "grasp_waist_pitch": ParameterValue(
                    grasp_waist_pitch, value_type=float
                ),
                "carry_waist_lift": 0.18,
                "carry_waist_pitch": -0.10,
                "place_waist_lift": 0.20,
                "place_waist_pitch": -0.05,
                "load_test_lift": 0.05,
                "carry_lift": 0.10,
                "hold_seconds": 3.0,
                "base_max_linear": 0.18,
                "base_max_angular": 0.45,
                "state_wait_timeout": 60.0,
                "physical_base_offset_x": 0.190,
                "physical_base_offset_y": -0.400617,
            },
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("headless", default_value="false"),
            DeclareLaunchArgument("use_rviz", default_value="true"),
            DeclareLaunchArgument(
                "execute",
                default_value="true",
                description="Execute Gazebo motion; false keeps MTC planning-only",
            ),
            DeclareLaunchArgument(
                "validate_all_nine",
                default_value="true",
                description=(
                    "Visit all 9 pallet cells and perform a 50 mm / 3 s 15 kg "
                    "pick-hold-return test before the full transfer"
                ),
            ),
            DeclareLaunchArgument(
                "full_transfer_box_index",
                default_value="4",
                description=(
                    "0..8 selects the tote used for the final pallet-to-bin transfer; "
                    "4 is the centre tote and therefore the hardest reach"
                ),
            ),
            DeclareLaunchArgument(
                "pre_extract_enabled",
                default_value="false",
                description="Plan/execute one-arm extraction of a negative-Y perimeter tote",
            ),
            DeclareLaunchArgument(
                "skip_pre_extract_stage", default_value="false",
                description="Planning probe only: use the post-pull tote pose without running the hook stage",
            ),
            DeclareLaunchArgument(
                "pre_extract_box_index",
                default_value="0",
                description="Nearest negative-Y perimeter tote used for one-arm extraction experiments",
            ),
            DeclareLaunchArgument(
                "continue_after_pre_extract_plan",
                default_value="false",
                description="Continue planning-only into the dual-arm transfer after the hook stage",
            ),
            DeclareLaunchArgument(
                "pre_extract_distance",
                default_value="0.12",
                description="Initial perimeter pull distance in metres",
            ),
            DeclareLaunchArgument("pre_extract_waist_lift", default_value="0.04"),
            DeclareLaunchArgument("pre_extract_waist_pitch", default_value="-0.70"),
            DeclareLaunchArgument("stop_on_failure", default_value="true"),
            DeclareLaunchArgument("hook_hole_z_offset", default_value="0.053"),
            DeclareLaunchArgument("hook_hole_inset", default_value="0.015"),
            DeclareLaunchArgument("hook_tcp_hole_clearance", default_value="0.100"),
            DeclareLaunchArgument("hook_approach_height", default_value="0.11"),
            DeclareLaunchArgument("hook_approach_outset", default_value="0.055"),
            DeclareLaunchArgument("hook_seat_drop", default_value="0.0"),
            DeclareLaunchArgument("handle_half_span", default_value="0.195"),
            DeclareLaunchArgument("grasp_half_span", default_value="0.195"),
            DeclareLaunchArgument("handle_z_offset", default_value="0.035"),
            DeclareLaunchArgument("grasp_waist_pitch", default_value="-0.70"),
            DeclareLaunchArgument("grasp_waist_lift", default_value="0.04"),
            DeclareLaunchArgument("left_tcp_roll", default_value="-1.57079632679"),
            DeclareLaunchArgument("left_tcp_pitch", default_value="0.0"),
            DeclareLaunchArgument("left_tcp_yaw", default_value="1.57079632679"),
            DeclareLaunchArgument("right_tcp_roll", default_value="-1.57079632679"),
            DeclareLaunchArgument("right_tcp_pitch", default_value="0.0"),
            DeclareLaunchArgument("right_tcp_yaw", default_value="-1.57079632679"),
            DeclareLaunchArgument("grasp_x_offset", default_value="0.05"),
            DeclareLaunchArgument("dual_ik_attempts", default_value="100"),
            DeclareLaunchArgument("dual_ik_timeout", default_value="0.50"),
            DeclareLaunchArgument("pregrasp_clearance", default_value="0.12"),
            DeclareLaunchArgument("source_grasp_x", default_value="0.45"),
            DeclareLaunchArgument("source_grasp_y", default_value="-0.76"),
            DeclareLaunchArgument("swap_arm_grasps", default_value="false"),
            gazebo_moveit,
            transfer,
        ]
    )
