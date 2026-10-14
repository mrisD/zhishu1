#!/usr/bin/env python3
"""Spawn the Gazebo fixture at the pivot selected by the auto sampler.

Uses ros_gz_sim create's Gazebo transport command so no service bridge is needed.
Publishes a transient-local acknowledgment; node never commands the robot before ACK.
"""
import subprocess
import threading
import json
import xml.etree.ElementTree as ET
import rclpy
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from std_msgs.msg import Bool
from pathlib import Path


class FixtureSpawner(Node):
    def __init__(self):
        super().__init__('tcp_fixture_spawner')
        self.declare_parameter('world', 'empty')
        self.declare_parameter('name', 'tcp_pivot_fixture')
        self.lock = threading.Lock()
        self.spawning = False
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.publisher = self.create_publisher(Bool, '/tcp_calibration/fixture_ready', qos)
        self.subscriber = self.create_subscription(
            PoseStamped, '/tcp_calibration/fixture_pose', self.on_pose, qos)

    def on_pose(self, msg):
        with self.lock:
            if self.spawning:
                return
            self.spawning = True
        threading.Thread(target=self.spawn, args=(msg,), daemon=True).start()

    def spawn(self, msg):
        model = Path(get_package_share_directory('zhishu_robot_tcp_calibration')) / 'models/tcp_pivot_fixture.sdf'
        pos = msg.pose.position
        # Sampler coordinates are in base_link; Gazebo create expects world.
        # base_link and the model root base_footprint coincide in this robot.
        try:
            if msg.header.frame_id not in ('base_link', 'base_footprint'):
                raise ValueError('Unsupported pivot frame: ' + msg.header.frame_id)
            result = subprocess.run(
                ['gz', 'topic', '-e', '-n', '1', '--json-output', '-t',
                 '/world/' + self.get_parameter('world').value + '/pose/info'],
                capture_output=True, text=True, timeout=10, check=True)
            poses = json.loads(result.stdout)['pose']
            robot = next(p for p in poses if p.get('name') == 'zhishu_robot')
            q = robot['orientation']
            x, y, z, w = (q.get(k, 0.0) for k in ('x', 'y', 'z', 'w'))
            norm = (x*x+y*y+z*z+w*w)**0.5
            x, y, z, w = (v/norm for v in (x, y, z, w))
            px, py, pz = pos.x, pos.y, pos.z
            translation = robot.get('position', {})
            world_xyz = (
                (1-2*y*y-2*z*z)*px + (2*x*y-2*z*w)*py + (2*x*z+2*y*w)*pz + translation.get('x', 0.0),
                (2*x*y+2*z*w)*px + (1-2*x*x-2*z*z)*py + (2*y*z-2*x*w)*pz + translation.get('y', 0.0),
                (2*x*z-2*y*w)*px + (2*y*z+2*x*w)*py + (1-2*x*x-2*y*y)*pz + translation.get('z', 0.0))
        except (subprocess.SubprocessError, OSError, ValueError, KeyError, StopIteration, ZeroDivisionError) as exc:
            self.get_logger().error('Cannot transform pivot to Gazebo world: %s' % exc)
            self.publisher.publish(Bool(data=False))
            return
        sdf = ET.parse(model).getroot()
        link = sdf.find('model/link')
        link.find("visual[@name='base']/pose").text = '0 0 %.9f 0 0 0' % (0.015-world_xyz[2])
        height = world_xyz[2]-0.27
        if height > 0.0:
            pedestal = ET.SubElement(link, 'visual', name='pedestal')
            ET.SubElement(pedestal, 'pose').text = '0 0 %.9f 0 0 0' % (-0.5*world_xyz[2]-0.105)
            cylinder = ET.SubElement(ET.SubElement(pedestal, 'geometry'), 'cylinder')
            ET.SubElement(cylinder, 'radius').text = '0.025'
            ET.SubElement(cylinder, 'length').text = str(height)
            material = ET.SubElement(pedestal, 'material')
            ET.SubElement(material, 'ambient').text = '0.18 0.18 0.22 1'
            ET.SubElement(material, 'diffuse').text = '0.18 0.18 0.22 1'
        cmd = ['ros2', 'run', 'ros_gz_sim', 'create',
               '-world', self.get_parameter('world').value,
               '-string', ET.tostring(sdf, encoding='unicode'),
               '-name', self.get_parameter('name').value,
               '-x', str(world_xyz[0]), '-y', str(world_xyz[1]), '-z', str(world_xyz[2])]
        self.get_logger().info('Creating Gazebo fixture at [%.3f, %.3f, %.3f]' %
                               (pos.x, pos.y, pos.z))
        try:
            result = subprocess.run(cmd, capture_output=True, text=True, timeout=25, check=False)
            ok = result.returncode == 0
            if not ok:
                self.get_logger().error('Fixture creation failed: %s %s' %
                                        (result.stdout[-500:], result.stderr[-500:]))
            else:
                self.get_logger().info('Gazebo fixture created')
            self.publisher.publish(Bool(data=ok))
        except (subprocess.TimeoutExpired, OSError) as exc:
            self.get_logger().error('Fixture creation exception: %s' % exc)
            self.publisher.publish(Bool(data=False))


def main():
    rclpy.init()
    node = FixtureSpawner()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
