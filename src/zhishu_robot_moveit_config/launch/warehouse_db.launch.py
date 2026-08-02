from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import EnvironmentVariable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    database_path = LaunchConfiguration("moveit_warehouse_database_path")

    # moveit_configs_utils still generates a MongoDB wrapper launch, but
    # warehouse_ros_mongo is not released for Ubuntu 24.04 / ROS 2 Jazzy.
    # SQLite is the supported in-process backend and uses warehouse_host as
    # the database file path (warehouse_port is ignored).
    warehouse_services = Node(
        package="moveit_ros_warehouse",
        executable="moveit_warehouse_services",
        name="moveit_warehouse_services",
        output="screen",
        parameters=[
            {
                "warehouse_plugin": "warehouse_ros_sqlite::DatabaseConnection",
                "warehouse_host": database_path,
                "warehouse_port": 0,
            }
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "moveit_warehouse_database_path",
                default_value=PathJoinSubstitution(
                    [
                        EnvironmentVariable("HOME"),
                        ".ros",
                        "zhishu_robot_warehouse.sqlite",
                    ]
                ),
                description="SQLite file used for persistent MoveIt states and scenes",
            ),
            warehouse_services,
        ]
    )
