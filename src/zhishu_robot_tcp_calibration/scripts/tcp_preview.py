#!/usr/bin/env python3
"""Place the reference needle at the model's actual tip, without arm motion."""
import time
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from geometry_msgs.msg import PoseStamped
from tf2_ros import Buffer, TransformListener


def main():
    rclpy.init()
    node = Node('tcp_static_preview')
    node.declare_parameter('arm', 'right')
    arm = node.get_parameter('arm').value
    if arm not in ('left', 'right'):
        raise ValueError('arm must be left or right')
    buffer = Buffer()
    listener = TransformListener(buffer, node)
    qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                     durability=DurabilityPolicy.TRANSIENT_LOCAL)
    publisher = node.create_publisher(PoseStamped, '/tcp_calibration/fixture_pose', qos)
    deadline = time.monotonic()+15
    tip = arm+'_calibration_tip_truth'
    flange = 'L_hand_link' if arm == 'left' else 'R_hand_link'
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
        if not buffer.can_transform('base_link', tip, rclpy.time.Time()):
            continue
        transform = buffer.lookup_transform('base_link', tip, rclpy.time.Time())
        offset = buffer.lookup_transform(flange, tip, rclpy.time.Time()).transform.translation
        pose = PoseStamped()
        pose.header.frame_id = 'base_link'
        pose.pose.position.x = transform.transform.translation.x
        pose.pose.position.y = transform.transform.translation.y
        pose.pose.position.z = transform.transform.translation.z
        pose.pose.orientation.w = 1.0
        node.get_logger().info('Actual model TCP flange offset: [%.6f, %.6f, %.6f] m' %
                               (offset.x, offset.y, offset.z))
        for _ in range(30):
            publisher.publish(pose)
            rclpy.spin_once(node, timeout_sec=0.1)
        break
    else:
        raise RuntimeError('Actual model tip TF unavailable')
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
