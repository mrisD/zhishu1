import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import AppendEnvironmentVariable, DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.conditions import UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    own = get_package_share_directory('zhishu_robot_workobject_calibration')
    tcp = get_package_share_directory('zhishu_robot_tcp_calibration')
    gazebo = get_package_share_directory('zhishu_robot_gazebo')
    declarations = [
        DeclareLaunchArgument('ros_domain_id', default_value='68'),
        DeclareLaunchArgument('gz_partition', default_value='workobject_calibration_sim'),
        DeclareLaunchArgument('sampler_only', default_value='false'),
        DeclareLaunchArgument('headless', default_value='false'),
        DeclareLaunchArgument('use_rviz', default_value='false'),
        DeclareLaunchArgument('origin_mode', default_value='axis_intersection'),
        DeclareLaunchArgument('board_x', default_value='0.55'),
        DeclareLaunchArgument('board_y', default_value='-0.225'),
        DeclareLaunchArgument('board_z', default_value='0.53'),
        DeclareLaunchArgument('board_roll', default_value='-0.04'),
        DeclareLaunchArgument('board_pitch', default_value='0.06'),
        DeclareLaunchArgument('board_yaw', default_value='0.28'),
        DeclareLaunchArgument('speed_scale', default_value='0.15'),
        DeclareLaunchArgument('noise_sigma_mm', default_value='0.0'),
        DeclareLaunchArgument('tcp_file', default_value=os.path.join(own, 'config', 'calibrated_probe.yaml')),
        DeclareLaunchArgument('output_dir', default_value='/tmp/guohua_workobject_calibration'),
    ]
    environment = [
        SetEnvironmentVariable('ROS_DOMAIN_ID', LaunchConfiguration('ros_domain_id')),
        SetEnvironmentVariable('GZ_PARTITION', LaunchConfiguration('gz_partition')),
        AppendEnvironmentVariable('GZ_SIM_RESOURCE_PATH', os.path.dirname(tcp)),
    ]
    simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(gazebo, 'launch', 'zhishu_robot.gazebo_moveit.launch.py')),
        condition=UnlessCondition(LaunchConfiguration('sampler_only')),
        launch_arguments={
            'world_file': 'empty.sdf', 'headless': LaunchConfiguration('headless'),
            'use_rviz': LaunchConfiguration('use_rviz'), 'z': '0.0033',
            'tcp_calibration_enabled': 'true', 'tcp_calibration_tool': 'probe',
            'tcp_calibration_arm': 'right',
            'initial_positions_file': os.path.join(tcp, 'config', 'tcp_initial_positions.yaml'),
            'use_left_arm_controller': 'false',
        }.items())
    parameters = {'use_sim_time': True}
    for name in ('board_x', 'board_y', 'board_z', 'board_roll', 'board_pitch',
                 'board_yaw', 'speed_scale', 'noise_sigma_mm'):
        parameters[name] = ParameterValue(LaunchConfiguration(name), value_type=float)
    for name in ('origin_mode', 'tcp_file', 'output_dir'):
        parameters[name] = LaunchConfiguration(name)
    sampler = Node(package='zhishu_robot_workobject_calibration', executable='workobject_sampler.py',
                   output='screen', parameters=[parameters])
    return LaunchDescription(declarations+environment+[simulation, sampler])
