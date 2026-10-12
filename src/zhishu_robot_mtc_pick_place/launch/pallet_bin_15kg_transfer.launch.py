#!/usr/bin/env python3
"""15 kg pallet coverage + mobile-base transfer into the STEP-sized large bin."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    moveit_share = get_package_share_directory("zhishu_robot_moveit_config")
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

    target_model = LaunchConfiguration("target_model")
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
                "mobile_base_sim_mass": "71.53",
                "waist_pitch_min": LaunchConfiguration("waist_pitch_min"),
                "tote_finger_length": LaunchConfiguration("tote_finger_length"),
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
        condition=UnlessCondition(LaunchConfiguration("reuse_gazebo")),
        launch_arguments={
            "headless": headless,
            "paused": LaunchConfiguration("paused"),
            "spawn_robot": LaunchConfiguration("spawn_robot"),
            "managed_gazebo_shutdown": LaunchConfiguration("managed_gazebo_shutdown"),
            "use_head_camera": LaunchConfiguration("use_head_camera"),
            "use_rviz": use_rviz,
            "world_file": LaunchConfiguration("world_file"),
            "use_left_arm_controller": "false",
            "mobile_base_sim_enabled": "true",
            "mobile_base_sim_mass": "71.53",
            "waist_pitch_min": LaunchConfiguration("waist_pitch_min"),
            "tote_finger_length": LaunchConfiguration("tote_finger_length"),
            "initial_positions_file": LaunchConfiguration("initial_positions_file"),
            # The base collision front is +0.234 m in base coordinates and
            # the pallet near edge is x=0.434 m.  Spawn at x=0 so the
            # chassis-to-pallet edge gap is exactly 0.200 m.
            "x": LaunchConfiguration("spawn_x"),
            "y": LaunchConfiguration("spawn_y"),
            # The wheel joint centres are already 82.5 mm above base_footprint
            # and the wheel radius is 85.8 mm.  The base collision bottom is
            # z=0 in that frame, so the settled spawn height is only 3.3 mm;
            # 90 mm would leave the whole robot floating and it then falls,
            # corrupting the initial odometry/yaw before docking.
            "z": "0.004",
            "yaw": LaunchConfiguration("spawn_yaw"),
            "allow_renaming": "false",
        }.items(),
    )

    transfer = Node(
        package="zhishu_robot_mtc_pick_place",
        executable="pallet_bin_15kg_transfer",
        name="pallet_bin_15kg_transfer",
        condition=IfCondition(LaunchConfiguration("start_transfer")),
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            {
                "use_sim_time": True,
                "robot_description_planning.joint_limits.waist_pitch_joint.min_position": ParameterValue(LaunchConfiguration("waist_pitch_min"), value_type=float),
                "execute": ParameterValue(execute, value_type=bool),
                "cleared_first_row_count": ParameterValue(LaunchConfiguration("cleared_first_row_count"), value_type=int),
                "source_row_only": ParameterValue(LaunchConfiguration("source_row_only"), value_type=int),
                "return_to_pallet_after_transfer": ParameterValue(LaunchConfiguration("return_to_pallet_after_transfer"), value_type=bool),
                "return_empty_only": ParameterValue(LaunchConfiguration("return_empty_only"), value_type=bool),
                "resume_after_pick": ParameterValue(LaunchConfiguration("resume_after_pick"), value_type=bool),
                "resume_floor_seating": ParameterValue(LaunchConfiguration("resume_floor_seating"), value_type=bool),
                "place_on_floor_before_release": ParameterValue(LaunchConfiguration("place_on_floor_before_release"), value_type=bool),
                "place_center_height": ParameterValue(LaunchConfiguration("place_center_height"), value_type=float),
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
                "hook_pose_x_offset": ParameterValue(LaunchConfiguration("hook_pose_x_offset"), value_type=float),
                "rear_push_probe": ParameterValue(LaunchConfiguration("rear_push_probe"), value_type=bool),
                "rear_push_execute": ParameterValue(LaunchConfiguration("rear_push_execute"), value_type=bool),
                "front_pull_probe": ParameterValue(LaunchConfiguration("front_pull_probe"), value_type=bool),
                "front_pull_execute": ParameterValue(LaunchConfiguration("front_pull_execute"), value_type=bool),
                "completed_source_columns": ParameterValue(LaunchConfiguration("completed_source_columns")),
                "side_push_execute": ParameterValue(LaunchConfiguration("side_push_execute"), value_type=bool),
                "side_push_distance": ParameterValue(LaunchConfiguration("side_push_distance"), value_type=float),
                "side_push_overtravel": ParameterValue(LaunchConfiguration("side_push_overtravel"), value_type=float),
                "side_push_continue_transfer": ParameterValue(LaunchConfiguration("side_push_continue_transfer"), value_type=bool),
                "side_push_alpha": ParameterValue(LaunchConfiguration("side_push_alpha"), value_type=float),
                "side_push_lean": ParameterValue(LaunchConfiguration("side_push_lean"), value_type=float),
                "side_push_contact_dx": ParameterValue(LaunchConfiguration("side_push_contact_dx"), value_type=float),
                "side_push_contact_dy": ParameterValue(LaunchConfiguration("side_push_contact_dy"), value_type=float),
                "side_push_waist_pitch": ParameterValue(LaunchConfiguration("side_push_waist_pitch"), value_type=float),
                "side_push_waist_lift": ParameterValue(LaunchConfiguration("side_push_waist_lift"), value_type=float),
                "side_push_use_left_arm": ParameterValue(LaunchConfiguration("side_push_use_left_arm"), value_type=bool),
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
                "align_with_extracted_center": ParameterValue(LaunchConfiguration("align_with_extracted_center"), value_type=bool),
                "prefer_symmetric_arm_branch": ParameterValue(LaunchConfiguration("prefer_symmetric_arm_branch"), value_type=bool),
                "swap_arm_grasps": ParameterValue(swap_arm_grasps, value_type=bool),
                "left_tcp_roll": ParameterValue(left_tcp_roll, value_type=float),
                "left_tcp_pitch": ParameterValue(left_tcp_pitch, value_type=float),
                "left_tcp_yaw": ParameterValue(left_tcp_yaw, value_type=float),
                "right_tcp_roll": ParameterValue(right_tcp_roll, value_type=float),
                "right_tcp_pitch": ParameterValue(right_tcp_pitch, value_type=float),
                "right_tcp_yaw": ParameterValue(right_tcp_yaw, value_type=float),
                # Conservative 15 kg settings. Increase only after the 50 mm
                # load test is stable and wrist F/T feedback is reasonable.
                # The Gazebo velocity-state feedback is discrete under the
                # 15 kg contact pull; slower time-parameterisation prevents a
                # transient path-tolerance hit without relaxing goal error.
                "velocity_scaling": ParameterValue(LaunchConfiguration("velocity_scaling"), value_type=float),
                "continuous_arm_motion": ParameterValue(LaunchConfiguration("continuous_arm_motion"), value_type=bool),
                "acceleration_scaling": ParameterValue(LaunchConfiguration("acceleration_scaling"), value_type=float),
                "grasp_waist_lift": ParameterValue(
                    grasp_waist_lift, value_type=float
                ),
                "grasp_waist_pitch": ParameterValue(
                    grasp_waist_pitch, value_type=float
                ),
                "carry_waist_lift": ParameterValue(LaunchConfiguration("carry_waist_lift"), value_type=float),
                "carry_waist_pitch": ParameterValue(LaunchConfiguration("carry_waist_pitch"), value_type=float),
                "carry_center_height": ParameterValue(LaunchConfiguration("carry_center_height"), value_type=float),
                # Half-sized table posture change preserves shallow grip
                # stability while leaving the arms enough insertion reach.
                "place_waist_lift": ParameterValue(LaunchConfiguration("place_waist_lift"), value_type=float),
                "place_waist_pitch": ParameterValue(LaunchConfiguration("place_waist_pitch"), value_type=float),
                "load_test_lift": 0.05,
                "carry_lift": ParameterValue(LaunchConfiguration("carry_lift"), value_type=float),
                "hold_seconds": 3.0,
                "base_max_linear": ParameterValue(LaunchConfiguration("base_max_linear"), value_type=float),
                "base_motion_timeout": ParameterValue(LaunchConfiguration("base_motion_timeout"), value_type=float),
                "base_max_angular": ParameterValue(LaunchConfiguration("base_max_angular"), value_type=float),
                "base_linear_acceleration": ParameterValue(LaunchConfiguration("base_linear_acceleration"), value_type=float),
                "base_angular_acceleration": ParameterValue(LaunchConfiguration("base_angular_acceleration"), value_type=float),
                "state_wait_timeout": 60.0,
                "physical_base_offset_x": 0.0,
                "physical_base_offset_y": -0.400617,
            },
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("initial_positions_file", default_value=os.path.join(moveit_share, "config", "pallet_bin_initial_positions.yaml")),
            DeclareLaunchArgument("spawn_x", default_value="0.000"),
            DeclareLaunchArgument("spawn_y", default_value="-0.400617"),
            DeclareLaunchArgument("spawn_yaw", default_value="0.0"),
            DeclareLaunchArgument("cleared_first_row_count", default_value="0"),
            DeclareLaunchArgument("source_row_only", default_value="-1"),
            DeclareLaunchArgument("return_to_pallet_after_transfer", default_value="false"),
            DeclareLaunchArgument("return_empty_only", default_value="false"),
            DeclareLaunchArgument("target_model", default_value="pallet_box_r0_c0"),
            DeclareLaunchArgument("measurement_bridge", default_value="true"),
            DeclareLaunchArgument("headless", default_value="true"),
            DeclareLaunchArgument("paused", default_value="false"),
            DeclareLaunchArgument("spawn_robot", default_value="true"),
            DeclareLaunchArgument("managed_gazebo_shutdown", default_value="false"),
            DeclareLaunchArgument("use_head_camera", default_value="true"),
            DeclareLaunchArgument("velocity_scaling", default_value="0.12"),
            DeclareLaunchArgument("continuous_arm_motion", default_value="true"),
            DeclareLaunchArgument("acceleration_scaling", default_value="0.08"),
            DeclareLaunchArgument("base_motion_timeout", default_value="90.0"),
            DeclareLaunchArgument("world_file", default_value="pallet_bin_15kg.sdf"),
            DeclareLaunchArgument("start_transfer", default_value="true"),
            DeclareLaunchArgument("transfer_start_delay", default_value="30.0"),
            DeclareLaunchArgument("use_rviz", default_value="false"),
            DeclareLaunchArgument("reuse_gazebo", default_value="false"),
            DeclareLaunchArgument("resume_after_pick", default_value="false"),
            DeclareLaunchArgument("resume_floor_seating", default_value="false"),
            DeclareLaunchArgument("place_on_floor_before_release", default_value="false"),
            DeclareLaunchArgument("place_center_height", default_value="0.96"),
            DeclareLaunchArgument("waist_pitch_min", default_value="-1.57079632679"),
            DeclareLaunchArgument("tote_finger_length", default_value="0.200"),
            DeclareLaunchArgument("carry_waist_lift", default_value="0.18"),
            DeclareLaunchArgument("carry_waist_pitch", default_value="-0.10"),
            DeclareLaunchArgument("carry_center_height", default_value="0.96"),
            DeclareLaunchArgument("base_max_linear", default_value="0.18"),
            DeclareLaunchArgument("base_max_angular", default_value="0.45"),
            DeclareLaunchArgument("base_linear_acceleration", default_value="0.30"),
            DeclareLaunchArgument("base_angular_acceleration", default_value="0.60"),
            DeclareLaunchArgument("place_waist_lift", default_value="0.14"),
            DeclareLaunchArgument("place_waist_pitch", default_value="-0.20"),
            DeclareLaunchArgument("carry_lift", default_value="0.20"),
            DeclareLaunchArgument(
                "execute",
                default_value="true",
                description="Execute Gazebo motion; false keeps MTC planning-only",
            ),
            DeclareLaunchArgument(
                "validate_all_nine",
                default_value="false",
                description=(
                    "Visit all 9 pallet cells and perform a 50 mm / 3 s 15 kg "
                    "pick-hold-return test before the full transfer"
                ),
            ),
            DeclareLaunchArgument(
                "full_transfer_box_index",
                default_value="0",
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
            DeclareLaunchArgument("pre_extract_waist_pitch", default_value="-1.0"),
            DeclareLaunchArgument("stop_on_failure", default_value="true"),
            DeclareLaunchArgument("hook_hole_z_offset", default_value="0.053"),
            DeclareLaunchArgument("hook_pose_x_offset", default_value="-0.080"),
            DeclareLaunchArgument("rear_push_probe", default_value="false"),
            DeclareLaunchArgument("rear_push_execute", default_value="false"),
            DeclareLaunchArgument("front_pull_probe", default_value="false"),
            DeclareLaunchArgument("front_pull_execute", default_value="false"),
            DeclareLaunchArgument("completed_source_columns", default_value="[-1]"),
            DeclareLaunchArgument("side_push_execute", default_value="false"),
            DeclareLaunchArgument("side_push_distance", default_value="0.324"),
            DeclareLaunchArgument("side_push_overtravel", default_value="0.0"),
            DeclareLaunchArgument("side_push_continue_transfer", default_value="false"),
            DeclareLaunchArgument("side_push_alpha", default_value="3.14159265358979"),
            DeclareLaunchArgument("side_push_lean", default_value="0.26"),
            DeclareLaunchArgument("side_push_contact_dx", default_value="0.23"),
            DeclareLaunchArgument("side_push_contact_dy", default_value="0.10"),
            DeclareLaunchArgument("side_push_waist_pitch", default_value="-0.785"),
            DeclareLaunchArgument("side_push_waist_lift", default_value="0.04"),
            DeclareLaunchArgument("side_push_use_left_arm", default_value="false"),
            DeclareLaunchArgument("hook_hole_inset", default_value="0.0"),
            DeclareLaunchArgument("hook_tcp_hole_clearance", default_value="0.060"),
            DeclareLaunchArgument("hook_approach_height", default_value="0.22"),
            DeclareLaunchArgument("hook_approach_outset", default_value="0.055"),
            DeclareLaunchArgument("hook_seat_drop", default_value="0.0"),
            DeclareLaunchArgument("handle_half_span", default_value="0.195"),
            DeclareLaunchArgument("grasp_half_span", default_value="0.195"),
            DeclareLaunchArgument("handle_z_offset", default_value="0.108"),
            DeclareLaunchArgument("grasp_waist_pitch", default_value="-1.10"),
            DeclareLaunchArgument("grasp_waist_lift", default_value="0.04"),
            DeclareLaunchArgument("left_tcp_roll", default_value="-1.57079632679"),
            DeclareLaunchArgument("left_tcp_pitch", default_value="0.0"),
            DeclareLaunchArgument("left_tcp_yaw", default_value="1.57079632679"),
            DeclareLaunchArgument("right_tcp_roll", default_value="-1.57079632679"),
            DeclareLaunchArgument("right_tcp_pitch", default_value="0.0"),
            DeclareLaunchArgument("right_tcp_yaw", default_value="-1.57079632679"),
            DeclareLaunchArgument("grasp_x_offset", default_value="0.0"),
            DeclareLaunchArgument("dual_ik_attempts", default_value="100"),
            DeclareLaunchArgument("dual_ik_timeout", default_value="0.05"),
            DeclareLaunchArgument("pregrasp_clearance", default_value="0.16"),
            DeclareLaunchArgument("source_grasp_x", default_value="0.0"),
            DeclareLaunchArgument("source_grasp_y", default_value="-0.400617"),
            DeclareLaunchArgument("align_with_extracted_center", default_value="false"),
            DeclareLaunchArgument("prefer_symmetric_arm_branch", default_value="false"),
            DeclareLaunchArgument("swap_arm_grasps", default_value="false"),
            gazebo_moveit,
            Node(
                package="ros_gz_bridge", executable="parameter_bridge",
                condition=IfCondition(LaunchConfiguration("measurement_bridge")),
            arguments=[
                ["/model/", target_model, "/pose@geometry_msgs/msg/PoseStamped[gz.msgs.Pose"],
                "/model/zhishu_robot/pose@geometry_msgs/msg/PoseStamped[gz.msgs.Pose",
                ["/world/pallet_bin_15kg/model/", target_model, "/link/box_link/sensor/tote_bottom_contact/contact@ros_gz_interfaces/msg/Contacts[gz.msgs.Contacts"],
            ],
            remappings=[
                (["/model/", target_model, "/pose"], "/pallet_bin/measured_box_pose"),
                ("/model/zhishu_robot/pose", "/pallet_bin/measured_base_pose"),
                (["/world/pallet_bin_15kg/model/", target_model, "/link/box_link/sensor/tote_bottom_contact/contact"], "/pallet_bin/tote_bottom_contacts"),
            ],
                output="screen",
            ),
            # The Gazebo controller spawners are intentionally delayed to let
            # gz_ros_control finish registering all interfaces.  Start the
            # transfer node after that settling window so its initial waist
            # trajectory is not published before body_posture_controller exists.
            TimerAction(period=LaunchConfiguration("transfer_start_delay"), actions=[transfer]),
        ]
    )
