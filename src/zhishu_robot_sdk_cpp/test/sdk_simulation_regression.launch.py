"""Read-only SDK regressions against the separately started GenericSystem graph."""
from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    config = (
        MoveItConfigsBuilder('zhishu_robot', package_name='zhishu_robot_moveit_config')
        .robot_description(mappings={
            'use_gazebo': 'false', 'use_fake_hardware': 'true',
            'left_arm_use_fake_hardware': 'true', 'right_arm_use_fake_hardware': 'true',
            'torso_head_use_fake_hardware': 'true', 'enable_motors_on_activate': 'false',
            'pdo_diagnostics_auto_start': 'false',
        }).to_moveit_configs()
    )
    return LaunchDescription([
        Node(package='zhishu_robot_sdk_cpp', executable='sdk_simulation_regression',
             parameters=[config.to_dict()], output='screen'),
    ])
