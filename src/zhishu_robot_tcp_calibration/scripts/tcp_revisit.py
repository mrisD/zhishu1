#!/usr/bin/env python3
"""Return to the recorded starting pivot pose through MoveIt's planner."""
import csv
import sys
import subprocess
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, PositionConstraint, OrientationConstraint
from geometry_msgs.msg import Pose
from shape_msgs.msg import SolidPrimitive


def main():
    subprocess.run(['gz', 'service', '-s', '/world/empty/scene/info',
                    '--reqtype', 'gz.msgs.Empty', '--reptype', 'gz.msgs.Scene',
                    '--timeout', '3000', '--req', ''],
                   capture_output=True, check=True, timeout=8)
    with open(sys.argv[1], newline='') as stream:
        row = next(r for r in csv.DictReader(stream) if r['role'] == 'pivot_fit')
    rclpy.init()
    node = Node('tcp_revisit', parameter_overrides=[])
    client = ActionClient(node, MoveGroup, '/move_action')
    if not client.wait_for_server(timeout_sec=15):
        raise RuntimeError('Simulation MoveIt action unavailable')
    goal = MoveGroup.Goal()
    request = goal.request
    request.group_name = 'right_arm'
    request.allowed_planning_time = 10.0
    request.num_planning_attempts = 4
    request.max_velocity_scaling_factor = 0.1
    request.max_acceleration_scaling_factor = 0.1
    request.start_state.is_diff = True
    position = PositionConstraint()
    position.header.frame_id = 'base_link'
    position.link_name = 'R_hand_link'
    position.weight = 1.0
    sphere = SolidPrimitive(type=SolidPrimitive.SPHERE, dimensions=[0.0005])
    pose = Pose()
    pose.position.x, pose.position.y, pose.position.z = (float(row[k]) for k in ('x', 'y', 'z'))
    pose.orientation.w = 1.0
    position.constraint_region.primitives = [sphere]
    position.constraint_region.primitive_poses = [pose]
    orientation = OrientationConstraint()
    orientation.header.frame_id = 'base_link'
    orientation.link_name = 'R_hand_link'
    orientation.orientation.x, orientation.orientation.y, orientation.orientation.z, orientation.orientation.w = (
        float(row[k]) for k in ('qx', 'qy', 'qz', 'qw'))
    orientation.absolute_x_axis_tolerance = 0.001
    orientation.absolute_y_axis_tolerance = 0.001
    orientation.absolute_z_axis_tolerance = 0.001
    orientation.weight = 1.0
    constraints = Constraints()
    constraints.position_constraints = [position]
    constraints.orientation_constraints = [orientation]
    request.goal_constraints = [constraints]
    goal.planning_options.plan_only = False
    future = client.send_goal_async(goal)
    rclpy.spin_until_future_complete(node, future)
    handle = future.result()
    if not handle.accepted:
        raise RuntimeError('Revisit request rejected')
    result = handle.get_result_async()
    rclpy.spin_until_future_complete(node, result)
    code = result.result().result.error_code.val
    if code != 1:
        raise RuntimeError('Revisit failed: %d' % code)
    print('Returned to recorded right-arm starting pivot pose')
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
