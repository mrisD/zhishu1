#!/usr/bin/env python3
"""Spawn the Gazebo fixture at the pivot selected by the auto sampler.

Uses ros_gz_sim create's Gazebo transport command so no service bridge is needed.
Publishes a transient-local acknowledgment; node never commands the robot before ACK.
"""
import subprocess
import threading
import json
import copy
import math
import xml.etree.ElementTree as ET
import rclpy
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from std_msgs.msg import Bool
from moveit_msgs.msg import CollisionObject
from moveit_msgs.srv import ApplyPlanningScene
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import Pose
from pathlib import Path


class FixtureSpawner(Node):
    def __init__(self):
        super().__init__('tcp_fixture_spawner')
        self.declare_parameter('world', 'empty')
        self.declare_parameter('name', 'tcp_pivot_fixture')
        self.declare_parameter('auto_focus', True)
        self.lock = threading.Lock()
        self.spawning = False
        self.created = False
        self.last_stamp = None
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.publisher = self.create_publisher(Bool, '/tcp_calibration/fixture_ready', qos)
        self.pose_publisher = self.create_publisher(PoseStamped, '/tcp_calibration/fixture_ready_pose', qos)
        self.scene_client = self.create_client(ApplyPlanningScene, '/apply_planning_scene')
        self.subscriber = self.create_subscription(
            PoseStamped, '/tcp_calibration/fixture_pose', self.on_pose, qos)

    def on_pose(self, msg):
        with self.lock:
            if self.spawning:
                return
            stamp = (msg.header.stamp.sec, msg.header.stamp.nanosec)
            if stamp == self.last_stamp:
                self.pose_publisher.publish(msg)
                return
            self.spawning = True
        threading.Thread(target=self.run_request, args=(msg,), daemon=True).start()

    def run_request(self, msg):
        self.publisher.publish(Bool(data=False))
        try:
            if self.spawn(msg):
                self.last_stamp = (msg.header.stamp.sec, msg.header.stamp.nanosec)
                self.pose_publisher.publish(msg)
                self.publisher.publish(Bool(data=True))
        except Exception as exc:
            self.get_logger().error('Fixture request failed: %s' % exc)
        finally:
            with self.lock:
                self.spawning = False

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
            self.created = any(p.get('name') == self.get_parameter('name').value for p in poses)
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
            return False
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
        # Solid rod/base geometry is shared with physics. The sharp contact
        # tip stays visual-only; this is not a force-contact calibration test.
        for visual in list(link.findall('visual')):
            if visual.get('name') == 'tip':
                continue
            collision = ET.SubElement(link, 'collision', name=visual.get('name'))
            collision.append(copy.deepcopy(visual.find('pose')))
            collision.append(copy.deepcopy(visual.find('geometry')))
        if self.created:
            removal = subprocess.run([
                'gz', 'service', '-s', '/world/'+self.get_parameter('world').value+'/remove',
                '--reqtype', 'gz.msgs.Entity', '--reptype', 'gz.msgs.Boolean',
                '--timeout', '3000', '--req', 'name: "'+self.get_parameter('name').value+'", type: MODEL'],
                capture_output=True, text=True, timeout=6, check=True)
            if 'data: true' not in removal.stdout:
                raise RuntimeError('Cannot replace previous calibration fixture')
            self.created = False
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
                self.created = True
                ok = self.apply_collision_scene(msg, world_xyz[2], (x, y, z, w))
                if ok and self.get_parameter('auto_focus').value:
                    self.focus_camera(world_xyz)
                self.get_logger().info('Gazebo fixture and MoveIt collision scene ready: %s' % ok)
            return ok
        except (subprocess.TimeoutExpired, OSError) as exc:
            self.get_logger().error('Fixture creation exception: %s' % exc)
            self.publisher.publish(Bool(data=False))
            return False

    def apply_collision_scene(self, msg, world_height, quaternion):
        if not self.scene_client.wait_for_service(timeout_sec=15):
            raise RuntimeError('MoveIt collision scene service unavailable')
        x, y, z, w = quaternion
        # World vertical expressed in the robot base frame.
        up = (2*x*z-2*y*w, 2*y*z+2*x*w, 1-2*x*x-2*y*y)
        obstacle = CollisionObject()
        obstacle.header.frame_id = msg.header.frame_id
        obstacle.id = self.get_parameter('name').value
        obstacle.operation = CollisionObject.ADD
        # Exclude the last 2 mm around the reference apex from collision
        # geometry so a tip-to-tip calibration pose remains admissible.
        cylinders = [(0.14, 0.20, 0.012), (0.021, 0.038, 0.012),
                     (world_height-0.015, 0.03, 0.065)]
        if world_height > 0.27:
            cylinders.append((0.5*world_height+0.105, world_height-0.27, 0.025))
        for depth, length, radius in cylinders:
            obstacle.primitives.append(SolidPrimitive(
                type=SolidPrimitive.CYLINDER, dimensions=[length, radius]))
            pose = Pose()
            pose.position.x = msg.pose.position.x-depth*up[0]
            pose.position.y = msg.pose.position.y-depth*up[1]
            pose.position.z = msg.pose.position.z-depth*up[2]
            pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w = (-x, -y, -z, w)
            obstacle.primitive_poses.append(pose)
        request = ApplyPlanningScene.Request()
        request.scene.is_diff = True
        request.scene.world.collision_objects = [obstacle]
        future = self.scene_client.call_async(request)
        done = threading.Event()
        future.add_done_callback(lambda _: done.set())
        if not done.wait(15):
            raise RuntimeError('Timed out updating MoveIt collision scene')
        return future.result().success

    def focus_camera(self, point):
        yaw = 3*math.pi/4
        pitch = math.atan2(0.55, math.sqrt(2)*0.85)
        q = (-math.sin(pitch/2)*math.sin(yaw/2),
             math.sin(pitch/2)*math.cos(yaw/2),
             math.cos(pitch/2)*math.sin(yaw/2),
             math.cos(pitch/2)*math.cos(yaw/2))
        request = ('pose: {position: {x: %.9f, y: %.9f, z: %.9f}, '
                   'orientation: {x: %.9f, y: %.9f, z: %.9f, w: %.9f}}') % (
                       point[0]+0.85, point[1]-0.85, point[2]+0.65, *q)
        try:
            subprocess.run(['gz', 'service', '-s', '/gui/move_to/pose',
                            '--reqtype', 'gz.msgs.GUICamera', '--reptype', 'gz.msgs.Boolean',
                            '--timeout', '1000', '--req', request],
                           capture_output=True, timeout=3, check=False)
        except (OSError, subprocess.TimeoutExpired):
            pass  # Headless operation has no GUI camera service.


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
