#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import FindExecutable

def generate_launch_description():
    pkg_share = FindPackageShare('zhishu_robot_description')

    model = LaunchConfiguration('model')
    use_gui = LaunchConfiguration('use_gui')
    use_sim_time = LaunchConfiguration('use_sim_time')
    rviz_config = LaunchConfiguration('rviz_config')

    default_model = PathJoinSubstitution([
        pkg_share,
        'urdf',
        'zhishu_robot.urdf.xacro',
    ])
    default_rviz_config = PathJoinSubstitution([
        pkg_share,
        'rviz',
        'urdf.rviz',
    ])

    robot_description = ParameterValue(
        Command([
            FindExecutable(name='xacro'),
            ' ',
            model,
        ]),
        value_type=str,
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'model',
            default_value=default_model,
            description='Absolute path to the DH-oriented robot URDF file.',
        ),
        DeclareLaunchArgument(
            'use_gui',
            default_value='true',
            description='Start joint_state_publisher_gui when true; otherwise start joint_state_publisher.',
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation clock when true.',
        ),
        DeclareLaunchArgument(
            'rviz_config',
            default_value=default_rviz_config,
            description='Absolute path to the RViz2 config file.',
        ),

        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
            condition=IfCondition(use_gui),
            output='screen',
        ),
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            condition=UnlessCondition(use_gui),
            output='screen',
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
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config],
            output='screen',
        ),
    ])
