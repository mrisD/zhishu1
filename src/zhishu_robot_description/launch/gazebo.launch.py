#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_share = FindPackageShare('zhishu_robot_description')
    gazebo_ros_share = FindPackageShare('gazebo_ros')

    model = LaunchConfiguration('model')
    use_sim_time = LaunchConfiguration('use_sim_time')
    entity_name = LaunchConfiguration('entity_name')

    default_model = PathJoinSubstitution([
        pkg_share,
        'urdf',
        'zhishu_robot_description.urdf',
    ])

    robot_description = ParameterValue(
        Command(['cat ', model]),
        value_type=str,
    )

    gazebo_launch = PathJoinSubstitution([
        gazebo_ros_share,
        'launch',
        'gazebo.launch.py',
    ])

    return LaunchDescription([
        DeclareLaunchArgument(
            'model',
            default_value=default_model,
            description='Absolute path to the robot URDF file.',
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use Gazebo simulation clock.',
        ),
        DeclareLaunchArgument(
            'entity_name',
            default_value='zhishu_robot_description',
            description='Name of the Gazebo entity to spawn.',
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(gazebo_launch),
        ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': robot_description,
                'use_sim_time': use_sim_time,
            }],
        ),
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            output='screen',
        ),
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            name='spawn_zhishu_robot',
            output='screen',
            arguments=[
                '-topic', 'robot_description',
                '-entity', entity_name,
            ],
        ),
    ])
