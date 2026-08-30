from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="zhishu_robot_rest_api",
                executable="zhishu_robot_rest_api_server",
                name="zhishu_robot_rest_api_server",
                output="screen",
            )
        ]
    )
