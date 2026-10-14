#!/usr/bin/env python3
"""Full automatic left/right TCP pivot calibration in Gazebo Harmonic.

One arm is calibrated per launch to avoid issuing concurrent goals to the
existing dual_arm_controller. The same launch works for either arm/tool.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import AppendEnvironmentVariable, DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.conditions import IfCondition, UnlessCondition
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    arm = LaunchConfiguration('arm')
    tool = LaunchConfiguration('tool')
    method = LaunchConfiguration('method')
    noise = LaunchConfiguration('noise')
    samples = LaunchConfiguration('samples')
    seed_pose = LaunchConfiguration('seed_pose')
    output_dir = LaunchConfiguration('output_dir')
    use_rviz = LaunchConfiguration('use_rviz')
    calibration_share = get_package_share_directory('zhishu_robot_tcp_calibration')
    gazebo_share = get_package_share_directory('zhishu_robot_gazebo')

    moveit_config = (MoveItConfigsBuilder('zhishu_robot', package_name='zhishu_robot_moveit_config')
                     .robot_description(mappings={
                         'use_gazebo': 'true',
                         'use_fake_hardware': 'false',
                         'initial_positions_file': LaunchConfiguration('initial_positions_file'),
                         'tcp_calibration_enabled': 'true',
                         'tcp_calibration_tool': tool,
                     }).to_moveit_configs())

    original_gazebo_moveit = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            gazebo_share, 'launch', 'zhishu_robot.gazebo_moveit.launch.py')),
        condition=UnlessCondition(LaunchConfiguration('sampler_only')),
        launch_arguments={
            'world_file': 'empty.sdf',
            'use_rviz': use_rviz,
            'headless': LaunchConfiguration('headless'),
            'paused': LaunchConfiguration('paused'),
            'initial_positions_file': LaunchConfiguration('initial_positions_file'),
            'z': '0.0033',
            'use_left_arm_controller': 'false',
            'tcp_calibration_enabled': 'true',
            'tcp_calibration_tool': tool,
        }.items(),
    )
    fixture_spawner = Node(
        condition=UnlessCondition(LaunchConfiguration('sampler_only')),
        package='zhishu_robot_tcp_calibration', executable='tcp_fixture_spawner.py',
        output='screen', parameters=[{'world': 'empty', 'name': 'tcp_pivot_fixture'}])
    sampler = Node(
        condition=IfCondition(LaunchConfiguration('auto_sample')),
        package='zhishu_robot_tcp_calibration', executable='tcp_auto_sampler',
        output='screen', parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            {'arm': arm, 'tool': tool, 'method': method, 'noise': noise,
             'samples': samples, 'seed_pose': seed_pose,
             'base_frame': 'base_link',
             'output_dir': output_dir, 'noise_seed': LaunchConfiguration('noise_seed'),
             'speed_scale': LaunchConfiguration('speed_scale'),
             'rotation_step_rad': LaunchConfiguration('rotation_step_rad'),
             'position_sigma_mm': LaunchConfiguration('position_sigma_mm'),
             'orientation_sigma_deg': LaunchConfiguration('orientation_sigma_deg'),
             'elongator_length_m': LaunchConfiguration('elongator_length_m'),
             'max_tip_error_mm': LaunchConfiguration('max_tip_error_mm'),
             'spawn_fixture': True, 'use_sim_time': True,
             'validation_samples': 3, 'max_condition_number': 150.0,
             'max_fit_rms_mm': 3.0},
        ])
    return LaunchDescription([
        DeclareLaunchArgument('arm', default_value='right', description='left or right'),
        DeclareLaunchArgument('tool', default_value='probe', description='probe or long_probe'),
        DeclareLaunchArgument('method', default_value='multi_point',
                              description='four_point or multi_point'),
        DeclareLaunchArgument('samples', default_value='12', description='Number of fitting poses'),
        DeclareLaunchArgument('noise', default_value='ideal', description='ideal or gaussian'),
        DeclareLaunchArgument('noise_seed', default_value='42'),
        DeclareLaunchArgument('seed_pose', default_value='current',
                              description='Named SRDF starting pose or current; for left choose left_arm_pose1'),
        DeclareLaunchArgument('output_dir', default_value='/tmp/guohua_tcp_calibration'),
        DeclareLaunchArgument('speed_scale', default_value='0.10'),
        DeclareLaunchArgument('rotation_step_rad', default_value='0.32'),
        DeclareLaunchArgument('position_sigma_mm', default_value='0.2'),
        DeclareLaunchArgument('orientation_sigma_deg', default_value='0.02'),
        DeclareLaunchArgument('elongator_length_m', default_value='0.05',
                              description='ABB TCP&Z,X axis extension length in meters'),
        DeclareLaunchArgument('max_tip_error_mm', default_value='8.0'),
        DeclareLaunchArgument('headless', default_value='false'),
        DeclareLaunchArgument('initial_positions_file', default_value=os.path.join(
            calibration_share, 'config', 'tcp_initial_positions.yaml')),
        DeclareLaunchArgument('paused', default_value='false'),
        DeclareLaunchArgument('auto_sample', default_value='true'),
        DeclareLaunchArgument('sampler_only', default_value='false'),
        DeclareLaunchArgument('sampling_delay', default_value='40.0'),
        DeclareLaunchArgument('use_rviz', default_value='true'),
        AppendEnvironmentVariable('GZ_SIM_RESOURCE_PATH', os.path.dirname(calibration_share)),
        original_gazebo_moveit,
        fixture_spawner,
        # Gazebo spawns the robot and activates the joint-state and trajectory
        # controllers asynchronously. Give that startup chain time to finish
        # before MoveIt attempts the seed-pose execution.
        TimerAction(period=LaunchConfiguration('sampling_delay'), actions=[sampler]),
    ])
