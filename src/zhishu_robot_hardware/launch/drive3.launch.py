from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_share = get_package_share_directory('ethercat_driver')
    default_config = os.path.join(pkg_share, 'config', 'driver.yaml')

    args = [
        DeclareLaunchArgument('config_file', default_value=default_config),
        DeclareLaunchArgument('motor_name', default_value='drive3'),
        DeclareLaunchArgument('topic_prefix', default_value='drive3'),
        DeclareLaunchArgument('joint_name', default_value='joint3'),
        DeclareLaunchArgument('master_index', default_value='0'),
        DeclareLaunchArgument('alias', default_value='0'),
        DeclareLaunchArgument('slave_position', default_value='2'),
        DeclareLaunchArgument('cycle_ns', default_value='1000000'),
        DeclareLaunchArgument('auto_enable', default_value='false'),
        DeclareLaunchArgument('publish_joint_states', default_value='true'),
        DeclareLaunchArgument('max_velocity_counts_s', default_value='8000.0'),
        DeclareLaunchArgument('max_accel_counts_s2', default_value='20000.0'),
        DeclareLaunchArgument('sync0_shift_ns', default_value='622620'),
        DeclareLaunchArgument('select_dc_reference_clock', default_value='true'),
        DeclareLaunchArgument('dc_reference_alias', default_value='0'),
        DeclareLaunchArgument('dc_reference_slave_position', default_value='0'),
        DeclareLaunchArgument('counts_per_rev', default_value='131072.0'),
    ]

    node = Node(
        package='ethercat_driver',
        executable='ethercat_driver_node',
        name='ethercat_driver_node',
        output='screen',
        parameters=[
            LaunchConfiguration('config_file'),
            {
                'motor_name': LaunchConfiguration('motor_name'),
                'topic_prefix': LaunchConfiguration('topic_prefix'),
                'joint_name': LaunchConfiguration('joint_name'),
                'master_index': ParameterValue(LaunchConfiguration('master_index'), value_type=int),
                'alias': ParameterValue(LaunchConfiguration('alias'), value_type=int),
                'slave_position': ParameterValue(LaunchConfiguration('slave_position'), value_type=int),
                'cycle_ns': ParameterValue(LaunchConfiguration('cycle_ns'), value_type=int),
                'auto_enable': ParameterValue(LaunchConfiguration('auto_enable'), value_type=bool),
                'publish_joint_states': ParameterValue(LaunchConfiguration('publish_joint_states'), value_type=bool),
                'max_velocity_counts_s': ParameterValue(LaunchConfiguration('max_velocity_counts_s'), value_type=float),
                'max_accel_counts_s2': ParameterValue(LaunchConfiguration('max_accel_counts_s2'), value_type=float),
                'sync0_shift_ns': ParameterValue(LaunchConfiguration('sync0_shift_ns'), value_type=int),
                'select_dc_reference_clock': ParameterValue(LaunchConfiguration('select_dc_reference_clock'), value_type=bool),
                'dc_reference_alias': ParameterValue(LaunchConfiguration('dc_reference_alias'), value_type=int),
                'dc_reference_slave_position': ParameterValue(LaunchConfiguration('dc_reference_slave_position'), value_type=int),
                'counts_per_rev': ParameterValue(LaunchConfiguration('counts_per_rev'), value_type=float),
            },
        ],
    )

    return LaunchDescription(args + [node])
