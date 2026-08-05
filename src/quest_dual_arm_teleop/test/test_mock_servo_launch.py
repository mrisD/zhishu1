#!/usr/bin/env python3

import os
import unittest

import launch_testing
import launch_testing.actions
import launch_testing.asserts
import launch_testing.markers
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    EmitEvent,
    IncludeLaunchDescription,
    RegisterEventHandler,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


@launch_testing.markers.keep_alive
def generate_test_description():
    package_share = get_package_share_directory("quest_dual_arm_teleop")
    main_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(package_share, "launch", "quest_dual_arm_teleop.launch.py")
        ),
        launch_arguments={
            "use_fake_hardware": "true",
            "launch_rviz": "false",
            "tcp_port": "5006",
        }.items(),
    )

    test_process = Node(
        package="quest_dual_arm_teleop",
        executable="mock_servo_integration_test.py",
        arguments=[
            "--arm",
            "left",
            "--timeout",
            "35.0",
            "--tcp-port",
            "5006",
        ],
        output="screen",
    )

    return (
        LaunchDescription(
            [
                SetEnvironmentVariable("ROS_DOMAIN_ID", "77"),
                main_launch,
                TimerAction(period=8.0, actions=[test_process]),
                RegisterEventHandler(
                    OnProcessExit(
                        target_action=test_process,
                        on_exit=[
                            EmitEvent(
                                event=Shutdown(
                                    reason="mock Servo integration test completed"
                                )
                            )
                        ],
                    )
                ),
                launch_testing.actions.ReadyToTest(),
            ]
        ),
        {"test_process": test_process},
    )


class TestMockServoWhileRunning(unittest.TestCase):
    def test_wait_for_mock_process(self, proc_info, test_process):
        proc_info.assertWaitForStartup(test_process, timeout=20)
        proc_info.assertWaitForShutdown(test_process, timeout=60)


@launch_testing.post_shutdown_test()
class TestMockServoExitCode(unittest.TestCase):
    def test_exit_code(self, proc_info, test_process):
        launch_testing.asserts.assertExitCodes(
            proc_info, process=test_process, allowable_exit_codes=[0]
        )
