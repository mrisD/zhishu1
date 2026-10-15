"""Isolated, no-UI SDK integration test; ONLY GenericSystem mock hardware.

Use as a source launch file. No physical devices, Gazebo, saved calibration data,
or production commander library is used. Does not issue motion on startup.
"""
import os
from pathlib import Path
import xml.etree.ElementTree as ET
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    own = get_package_share_directory('zhishu_robot_commander_cpp')
    moveit = get_package_share_directory('zhishu_robot_moveit_config')
    config = (MoveItConfigsBuilder('zhishu_robot', package_name='zhishu_robot_moveit_config')
              .robot_description(mappings={
                  'use_gazebo': 'false', 'use_fake_hardware': 'true',
                  'left_arm_use_fake_hardware': 'true', 'right_arm_use_fake_hardware': 'true',
                  'torso_head_use_fake_hardware': 'true', 'enable_motors_on_activate': 'false',
                  'tcp_calibration_enabled': 'false',
                  'initial_positions_file': str(Path(__file__).resolve().parent/'validation_initial_positions.yaml'),
              }).to_moveit_configs())
    robot = ET.fromstring(config.robot_description['robot_description'])
    plugins = [p.text for p in robot.findall('ros2_control/hardware/plugin')]
    if not plugins or any(p != 'mock_components/GenericSystem' for p in plugins):
        raise RuntimeError('Refusing validation launch: hardware is not exclusively GenericSystem')
    description = config.robot_description
    state = Node(package='robot_state_publisher', executable='robot_state_publisher', parameters=[description])
    control = Node(package='controller_manager', executable='ros2_control_node',
                   parameters=[description, os.path.join(moveit, 'config/ros2_controllers.yaml'),
                               str(Path(__file__).resolve().parent/'mock_controller_manager.yaml')],
                   remappings=[('~/robot_description', '/robot_description')], output='screen')
    spawners = Node(package='controller_manager', executable='spawner', output='screen', arguments=[
        'joint_state_broadcaster', 'dual_arm_controller', 'left_gripper_controller',
        'right_gripper_controller', '--controller-manager', '/controller_manager'])
    group = Node(package='moveit_ros_move_group', executable='move_group',
                 parameters=[config.to_dict(), {'publish_robot_description': True,
                                               'publish_robot_description_semantic': True}], output='screen')
    commander = Node(package='zhishu_robot_commander_cpp', executable='commander_node',
                     name='zhishu_robot_commander_validation', output='screen', parameters=[
                         config.to_dict(), os.path.join(own, 'config/commander.yaml'),
                         {'commander.coordinate_frames_file': '', 'commander.velocity_scaling': .15,
                          'commander.acceleration_scaling': .15, 'commander.planning_time': 10.0}])
    return LaunchDescription([SetEnvironmentVariable('ROS_DOMAIN_ID', '169'),
                              state, control, spawners, group, commander])
