"""Add disabled admittance and two Servo nodes to an existing pallet scene.

Switch from dual_arm_controller only after MTC has stopped and the tote is held.
"""
import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    config = (MoveItConfigsBuilder('zhishu_robot', package_name='zhishu_robot_moveit_config')
              .robot_description(file_path='config/zhishu_robot.urdf.xacro', mappings={
                  'use_gazebo': 'true', 'use_fake_hardware': 'false',
                  'mobile_base_sim_enabled': 'true', 'mobile_base_sim_mass': '71.53',
                  'waist_pitch_min': '-1.57079632679', 'tote_finger_length': '0.360'})
              .robot_description_semantic(file_path='config/zhishu_robot.srdf')
              .robot_description_kinematics(file_path='config/kinematics.yaml')
              .joint_limits(file_path='config/joint_limits.yaml').to_moveit_configs())
    nodes = []
    for side in ('left', 'right'):
        path = os.path.join(get_package_share_directory('quest_dual_arm_teleop'),
                            'config', side + '_servo.yaml')
        with open(path, encoding='utf-8') as stream:
            servo = yaml.safe_load(stream)
        servo.update(publish_period=0.01, use_smoothing=False)
        nodes.append(Node(package='moveit_servo', executable='servo_node',
                          name=side + '_servo', output='screen', parameters=[
                              {'moveit_servo': servo}, config.robot_description,
                              config.robot_description_semantic,
                              config.robot_description_kinematics, config.joint_limits,
                              {'use_sim_time': True}]))
    nodes.append(Node(package='ros_gz_bridge', executable='parameter_bridge',
                      arguments=['/left_wrist_ft@geometry_msgs/msg/WrenchStamped[gz.msgs.Wrench',
                                 '/right_wrist_ft@geometry_msgs/msg/WrenchStamped[gz.msgs.Wrench'],
                      parameters=[{'use_sim_time': True}], output='screen'))
    nodes.append(Node(package='zhishu_robot_admittance',
                      executable='dual_arm_admittance_controller', name='dual_arm_admittance',
                      parameters=[os.path.join(get_package_share_directory('zhishu_robot_admittance'),
                                               'config', 'admittance.yaml'),
                                  {'use_sim_time': True, 'enable_on_start': False}], output='screen'))
    return LaunchDescription(nodes)
