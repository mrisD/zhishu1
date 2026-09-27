#!/usr/bin/env python3
"""Complete moving-bottle perception simulation."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_share = get_package_share_directory("zhishu_robot_bottle_grasp")
    gazebo_share = get_package_share_directory("zhishu_robot_gazebo")
    trained_weights = os.path.join(package_share, "models", "bottle_yolo26n_seg.pt")

    headless = LaunchConfiguration("headless")
    use_rviz = LaunchConfiguration("use_rviz")
    run_perception = LaunchConfiguration("run_perception")
    run_motion = LaunchConfiguration("run_motion")
    execute_grasp = LaunchConfiguration("execute_grasp")
    detector_backend = LaunchConfiguration("detector_backend")
    yolo_weights = LaunchConfiguration("yolo_weights")
    yolo_device = LaunchConfiguration("yolo_device")

    simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_share, "launch", "zhishu_robot.gazebo_moveit.launch.py")
        ),
        launch_arguments={
            "headless": headless,
            "use_rviz": use_rviz,
            "world_file": "bottle_grasp.sdf",
            "use_camera": "false",
            "use_head_camera": "true",
            "handeye_target_enabled": "false",
            "bottle_grasp_sim_enabled": "true",
            "managed_move_group_shutdown": "true",
            "managed_gazebo_shutdown": "true",
            "use_left_arm_controller": "false",
            "software_rendering": "true",
            "initial_positions_file": os.path.join(
                package_share, "config", "bottle_initial_positions.yaml"
            ),
        }.items(),
    )

    bottle_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="bottle_sim_bridge",
        output="screen",
        parameters=[{
            "config_file": os.path.join(package_share, "config", "bottle_sim_bridge.yaml")
        }],
    )
    camera_info_relay = Node(
        package="zhishu_robot_handeye_calibration",
        executable="camera_info_relay.py",
        name="bottle_camera_info_relay",
        output="screen",
        parameters=[{"use_sim_time": True}],
    )
    moving_bottle = Node(
        package="zhishu_robot_bottle_grasp",
        executable="moving_bottle",
        name="moving_bottle",
        output="screen",
        parameters=[{"use_sim_time": True}],
        condition=IfCondition(run_motion),
    )
    perception = Node(
        package="zhishu_robot_bottle_grasp",
        executable="bottle_perception",
        name="bottle_perception",
        output="screen",
        parameters=[
            os.path.join(package_share, "config", "bottle_grasp.yaml"),
            {
                "use_sim_time": True,
                "detector_backend": detector_backend,
                "yolo_weights": yolo_weights,
                "yolo_device": yolo_device,
            },
        ],
        condition=IfCondition(run_perception),
    )
    obstacle_cloud = Node(
        package="zhishu_robot_bottle_grasp",
        executable="obstacle_cloud",
        name="obstacle_cloud",
        output="screen",
        parameters=[{"use_sim_time": True}],
        condition=IfCondition(run_perception),
    )
    planning_scene_obstacles = Node(
        package="zhishu_robot_bottle_grasp",
        executable="planning_scene_obstacles",
        name="planning_scene_obstacles",
        output="screen",
        parameters=[{"use_sim_time": True}],
        condition=IfCondition(run_perception),
    )
    grasp_executor = Node(
        package="zhishu_robot_mtc_pick_place",
        executable="moving_bottle_grasp",
        name="moving_bottle_grasp",
        output="screen",
        parameters=[{
            "use_sim_time": True,
            "world_frame": "base_link",
            "intercept_x": 0.62,
            "intercept_y": 0.0,
            "grasp_z_offset": 0.025,
            "trigger_y": -0.19,
            "intercept_timeout": 60.0,
        }],
        condition=IfCondition(execute_grasp),
    )

    return LaunchDescription([
        DeclareLaunchArgument("headless", default_value="true"),
        DeclareLaunchArgument("use_rviz", default_value="false"),
        DeclareLaunchArgument("run_perception", default_value="true"),
        DeclareLaunchArgument("run_motion", default_value="true"),
        DeclareLaunchArgument("execute_grasp", default_value="true"),
        DeclareLaunchArgument("detector_backend", default_value="yolo"),
        DeclareLaunchArgument(
            "yolo_weights",
            default_value=trained_weights if os.path.exists(trained_weights) else "yolo26n-seg.pt",
        ),
        DeclareLaunchArgument("yolo_device", default_value="cpu"),
        simulation,
        bottle_bridge,
        TimerAction(period=5.0, actions=[camera_info_relay, moving_bottle]),
        TimerAction(
            period=7.0,
            actions=[perception, obstacle_cloud, planning_scene_obstacles],
        ),
        TimerAction(period=12.0, actions=[grasp_executor]),
    ])
