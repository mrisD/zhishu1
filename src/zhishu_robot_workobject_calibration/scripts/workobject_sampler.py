#!/usr/bin/env python3
"""Three-point workobject teaching with measured flange poses and calibrated TCP.

Gazebo truth is used ONLY by the simulated operator and independent evaluator.
The three-point solver receives measured TCP positions, never board pose.
"""
import csv
import json
import math
from pathlib import Path
import subprocess
import time
import xml.etree.ElementTree as ET

import numpy as np
import yaml
import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.time import Time
from rclpy.executors import ExternalShutdownException
from geometry_msgs.msg import Pose, TransformStamped
from sensor_msgs.msg import JointState
from shape_msgs.msg import SolidPrimitive
from moveit_msgs.action import MoveGroup, ExecuteTrajectory
from moveit_msgs.msg import (Constraints, PositionConstraint, OrientationConstraint,
                             BoundingVolume, CollisionObject)
from moveit_msgs.srv import ApplyPlanningScene, GetCartesianPath
from controller_manager_msgs.srv import ListControllers
from rcl_interfaces.srv import GetParameters
from tf2_ros import Buffer, TransformListener, StaticTransformBroadcaster
from workobject_math import (rotation_rpy, matrix_from_quaternion, quaternion_from_matrix,
                             pose_matrix, transform_point, solve_three_points, rotation_error_deg)


def ros_pose(matrix):
    result = Pose()
    result.position.x, result.position.y, result.position.z = map(float, matrix[:3, 3])
    q = quaternion_from_matrix(matrix[:3, :3])
    result.orientation.x, result.orientation.y, result.orientation.z, result.orientation.w = map(float, q)
    return result


def serial_pose(matrix):
    return {'translation_xyz_m': matrix[:3, 3].tolist(),
            'orientation_quaternion_xyzw': quaternion_from_matrix(matrix[:3, :3]).tolist()}


class Sampler(Node):
    def __init__(self):
        super().__init__('workobject_sampler')
        for name, default in {'board_x': .55, 'board_y': -.225, 'board_z': .53,
                              'board_roll': -.04, 'board_pitch': .06, 'board_yaw': .28,
                              'speed_scale': .2, 'noise_sigma_mm': 0.,
                              'origin_mode': 'axis_intersection', 'tcp_file': '',
                              'output_dir': '/tmp/guohua_workobject_calibration'}.items():
            self.declare_parameter(name, default)
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        self.joints = None
        self.joint_subscription = self.create_subscription(JointState, '/joint_states', self.on_joints, 10)
        self.broadcaster = StaticTransformBroadcaster(self)
        self.move = ActionClient(self, MoveGroup, '/move_action')
        self.execute = ActionClient(self, ExecuteTrajectory, '/execute_trajectory')
        self.cartesian = self.create_client(GetCartesianPath, '/compute_cartesian_path')
        self.scene = self.create_client(ApplyPlanningScene, '/apply_planning_scene')
        self.controllers = self.create_client(ListControllers, '/controller_manager/list_controllers')
        self.description = self.create_client(GetParameters, '/robot_state_publisher/get_parameters')
        self.speed = float(self.p('speed_scale'))
        if not .01 <= self.speed <= .3:
            raise ValueError('speed_scale must be between 0.01 and 0.30')
        if not 0 <= self.p('noise_sigma_mm') <= 1:
            raise ValueError('noise_sigma_mm must be between 0 and 1')
        if not self.p('use_sim_time'):
            raise RuntimeError('This automated teaching node is simulation-only: use_sim_time required')
        self.rows = []
        self.rng = np.random.default_rng(42)
        self.output = Path(self.p('output_dir')) / time.strftime('%Y%m%d_%H%M%S')
        self.output.mkdir(parents=True, exist_ok=False)

    def p(self, name):
        return self.get_parameter(name).value

    def on_joints(self, message):
        self.joints = dict(zip(message.name, message.position))

    def wait(self, future, seconds=120):
        rclpy.spin_until_future_complete(self, future, timeout_sec=seconds)
        if not future.done():
            raise RuntimeError('Timed out waiting for ROS operation; no further motion commanded')
        if future.exception():
            raise future.exception()
        return future.result()

    def settle(self, seconds=.4):
        end = time.monotonic()+seconds
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=.05)

    def tf(self, child):
        t = self.buffer.lookup_transform('base_link', child, Time()).transform
        return pose_matrix([t.translation.x, t.translation.y, t.translation.z],
                           [t.rotation.x, t.rotation.y, t.rotation.z, t.rotation.w])

    def ready(self):
        for client in (self.cartesian, self.scene, self.controllers, self.description):
            if not client.wait_for_service(timeout_sec=90):
                raise RuntimeError('Required simulation service not available: '+client.srv_name)
        for action in (self.move, self.execute):
            if not action.wait_for_server(timeout_sec=90):
                raise RuntimeError('MoveIt action unavailable')
        deadline = time.monotonic()+90
        while time.monotonic() < deadline:
            self.settle(.2)
            future = self.controllers.call_async(ListControllers.Request())
            rclpy.spin_until_future_complete(self, future, timeout_sec=5)
            if not future.done():
                future.cancel()
                continue  # DDS may briefly retain endpoints from a previous simulation.
            response = future.result()
            active = {c.name for c in response.controller if c.state == 'active'}
            if {'dual_arm_controller', 'body_posture_controller', 'joint_state_broadcaster'}.issubset(active):
                try:
                    self.initial = self.tf('R_hand_link')
                    self.truth_tool = np.linalg.inv(self.initial) @ self.tf('right_calibration_tip_truth')
                    break
                except Exception:
                    pass
        else:
            raise RuntimeError('Robot controllers / calibration probe TF not ready')
        # Both a dedicated domain and the simulation-only probe are mandatory.
        import os
        if os.environ.get('ROS_DOMAIN_ID') != '68' or os.environ.get('GZ_PARTITION') != 'workobject_calibration_sim':
            raise RuntimeError('Use dedicated ROS_DOMAIN_ID=68 / GZ_PARTITION=workobject_calibration_sim')
        request = GetParameters.Request(names=['robot_description'])
        description = self.wait(self.description.call_async(request), 10).values[0].string_value
        if 'gz_ros2_control/GazeboSimSystem' not in description or 'tcp_calibration_world_anchor' not in description:
            raise RuntimeError('Anchored Gazebo robot description required; refusing non-simulation hardware')
        data = yaml.safe_load(Path(self.p('tcp_file')).read_text())
        if data['parent_frame'] != 'R_hand_link':
            raise ValueError('TCP must be expressed in R_hand_link')
        self.calibrated_tool = pose_matrix(data['translation_xyz_m'], data['orientation_quaternion_xyzw'])
        self.controller_joints = [side+'_'+joint+'_joint' for side in ('left', 'right')
                                  for joint in ('shoulder_pitch', 'shoulder_roll', 'shoulder_yaw',
                                                'elbow_pitch', 'wrist_yaw', 'wrist_pitch', 'wrist_roll')]
        if self.joints is None or not set(self.controller_joints).issubset(self.joints):
            raise RuntimeError('Complete dual-arm measured joint state required')
        self.left_hold = {name: self.joints[name] for name in self.controller_joints[:7]}
        self.max_left_drift = 0.
        self.board = np.eye(4)
        self.board[:3, :3] = rotation_rpy(*[self.p('board_'+k) for k in ('roll', 'pitch', 'yaw')])
        self.board[:3, 3] = [self.p('board_'+k) for k in ('x', 'y', 'z')]
        self.points = np.array([[-.06, 0, 0], [.06, 0, 0], [0, .06, 0]])
        self.get_logger().info('Simulation controllers and calibrated TCP ready')

    def command(self, args, timeout=20):
        result = subprocess.run(args, text=True, capture_output=True, timeout=timeout, check=True)
        return result.stdout

    def model_poses(self):
        return json.loads(self.command(['gz', 'topic', '-e', '-n', '1', '--json-output',
                                      '-t', '/world/empty/pose/info']))['pose']

    def spawn(self, name, root, pose):
        if any(p.get('name') == name for p in self.model_poses()):
            reply = self.command(['gz', 'service', '-s', '/world/empty/remove', '--reqtype',
                                  'gz.msgs.Entity', '--reptype', 'gz.msgs.Boolean', '--timeout', '3000',
                                  '--req', 'name: "'+name+'", type: MODEL'])
            if 'data: true' not in reply:
                raise RuntimeError('Cannot replace own fixture '+name)
        # Pose is carried in the create request (not an SDF default overridden by create).
        r = pose[:3, :3]
        pitch = math.asin(float(np.clip(-r[2, 0], -1, 1)))
        roll, yaw = math.atan2(r[2, 1], r[2, 2]), math.atan2(r[1, 0], r[0, 0])
        self.command(['ros2', 'run', 'ros_gz_sim', 'create', '-world', 'empty',
                             '-name', name, '-string', ET.tostring(root, encoding='unicode'),
                             '-x', str(pose[0, 3]), '-y', str(pose[1, 3]), '-z', str(pose[2, 3]),
                             '-R', str(roll), '-P', str(pitch), '-Y', str(yaw)], 30)
        deadline = time.monotonic()+8
        while time.monotonic() < deadline:
            matches = [p for p in self.model_poses() if p.get('name') == name]
            if matches:
                actual = matches[0]
                t = pose_matrix([actual.get('position', {}).get(k, 0) for k in ('x', 'y', 'z')],
                                [actual['orientation'].get(k, 0) for k in ('x', 'y', 'z', 'w')])
                if np.linalg.norm(t[:3, 3]-pose[:3, 3]) < 1e-5 and rotation_error_deg(t[:3, :3], pose[:3, :3]) < .001:
                    return
            self.settle(.2)
        raise RuntimeError('Spawned model pose could not be independently confirmed: '+name)

    def fixture(self):
        robot = next(p for p in self.model_poses() if p.get('name') == 'zhishu_robot')
        self.world_base = pose_matrix([robot.get('position', {}).get(k, 0) for k in ('x', 'y', 'z')],
                                     [robot['orientation'].get(k, 0) for k in ('x', 'y', 'z', 'w')])
        world_board = self.world_base @ self.board
        root = ET.Element('sdf', version='1.9')
        model = ET.SubElement(root, 'model', name='workobject_calibration_board')
        ET.SubElement(model, 'static').text = 'true'
        link = ET.SubElement(model, 'link', name='board')
        obstacle = CollisionObject()
        obstacle.header.frame_id = 'base_link'
        obstacle.id = 'workobject_calibration_board'
        obstacle.operation = CollisionObject.ADD

        def shape(name, local, kind, dimensions, color, collision=True):
            visual = ET.SubElement(link, 'visual', name=name)
            p = local[:3, 3]
            q = quaternion_from_matrix(local[:3, :3])
            ET.SubElement(visual, 'pose', rotation_format='quat_xyzw').text = ' '.join(map(str, [*p, *q]))
            geom = ET.SubElement(visual, 'geometry')
            element = ET.SubElement(geom, 'mesh' if kind == 'cone' else kind)
            if kind == 'cone':
                ET.SubElement(element, 'uri').text = str(Path(get_package_share_directory('zhishu_robot_tcp_calibration'))/'models/unit_cone.stl')
                ET.SubElement(element, 'scale').text = '%f %f %f' % (dimensions[1], dimensions[1], dimensions[0])
                primitive = None
            elif kind == 'box':
                ET.SubElement(element, 'size').text = ' '.join(map(str, dimensions))
                primitive = SolidPrimitive(type=SolidPrimitive.BOX, dimensions=list(dimensions))
            else:
                length, radius = dimensions
                ET.SubElement(element, 'radius').text = str(radius)
                ET.SubElement(element, 'length').text = str(length)
                primitive = SolidPrimitive(type=SolidPrimitive.CYLINDER, dimensions=[length, radius])
            material = ET.SubElement(visual, 'material')
            for field in ('ambient', 'diffuse'):
                ET.SubElement(material, field).text = color
            if collision:
                import copy
                obj = ET.SubElement(link, 'collision', name=name)
                obj.append(copy.deepcopy(visual.find('pose')))
                obj.append(copy.deepcopy(geom))
                obstacle.primitives.append(primitive)
                obstacle.primitive_poses.append(ros_pose(self.board @ local))

        def position(xyz):
            t = np.eye(4)
            t[:3, 3] = xyz
            return t
        shape('table', position([0, 0, -.04]), 'box', [.22, .18, .018], '.12 .28 .38 1')
        h = float(world_board[2, 3]-.07)
        column_world = position([world_board[0, 3], world_board[1, 3], h/2+.02])
        shape('pedestal', np.linalg.inv(world_board) @ column_world, 'cylinder', [h, .025], '.2 .2 .22 1')
        base_world = position([world_board[0, 3], world_board[1, 3], .015])
        shape('base', np.linalg.inv(world_board) @ base_world, 'cylinder', [.03, .08], '.15 .15 .18 1')
        for label, p in zip(('X1', 'X2', 'Y1'), self.points):
            shape(label+'_stem', position(p+[0, 0, -.022]), 'cylinder', [.018, .003], '.9 .65 .08 1')
            # Cone tip is visual only: mathematical touch is not a force-contact test.
            shape(label+'_tip', position(p+[0, 0, -.014]), 'cone', [.014, .003], '1 .75 .1 1', False)
            # Ogre2 does not render TEXT markers. Use native, non-colliding
            # geometric strokes attached to the tabletop instead.
            glyphs = {
                'X': [((0, 0), (.008, .012)), ((0, .012), (.008, 0))],
                'Y': [((0, .012), (.004, .006)), ((.008, .012), (.004, .006)), ((.004, .006), (.004, 0))],
                '1': [((.004, 0), (.004, .012)), ((.001, .009), (.004, .012))],
                '2': [((0, .012), (.008, .012)), ((.008, .012), (.008, .007)),
                      ((.008, .007), (0, 0)), ((0, 0), (.008, 0))],
            }
            for char_index, char in enumerate(label):
                for stroke_index, (a, b) in enumerate(glyphs[char]):
                    a, b = np.asarray(a), np.asarray(b)
                    middle = (a+b)/2
                    local = position(p+[-.008+char_index*.011+middle[0], -.026+middle[1], -.0305])
                    local[:3, :3] = rotation_rpy(0, 0, math.atan2(*(b-a)[::-1]))
                    shape(label+'_glyph_%d_%d' % (char_index, stroke_index), local, 'box',
                          [float(np.linalg.norm(b-a)), .0012, .0005], '.95 .95 .95 1', False)
        self.spawn('workobject_calibration_board', root, world_board)
        request = ApplyPlanningScene.Request()
        request.scene.is_diff = True
        request.scene.world.collision_objects = [obstacle]
        if not self.wait(self.scene.call_async(request), 15).success:
            raise RuntimeError('Collision scene update rejected')
        target = world_board[:3, 3]+[0, 0, .055]
        camera = target+[.45, -.45, .35]
        yaw = 3*math.pi/4
        pitch = math.atan2(.35, math.sqrt(2)*.45)
        q = quaternion_from_matrix(rotation_rpy(0, pitch, yaw))
        req = 'pose: {position: {x: %f y: %f z: %f} orientation: {x: %f y: %f z: %f w: %f}}' % (*camera, *q)
        try:
            self.command(['gz', 'service', '-s', '/gui/move_to/pose', '--reqtype', 'gz.msgs.GUICamera',
                          '--reptype', 'gz.msgs.Boolean', '--timeout', '1000', '--req', req], 3)
        except (subprocess.SubprocessError, OSError):
            pass  # Headless validation does not require a GUI camera.
        self.get_logger().info('Tilted board, X1/X2/Y1 pins and MoveIt obstacles created')

    def move_pose(self, target):
        goal = MoveGroup.Goal()
        request = goal.request
        request.group_name = 'right_arm'
        request.num_planning_attempts = 10
        request.allowed_planning_time = 15.
        request.max_velocity_scaling_factor = self.speed
        request.max_acceleration_scaling_factor = self.speed
        request.start_state.is_diff = True
        constraints = Constraints()
        position = PositionConstraint()
        position.header.frame_id = 'base_link'
        position.link_name = 'R_hand_link'
        position.weight = 1.
        position.constraint_region = BoundingVolume(
            primitives=[SolidPrimitive(type=SolidPrimitive.SPHERE, dimensions=[.00015])],
            primitive_poses=[ros_pose(target)])
        orientation = OrientationConstraint()
        orientation.header.frame_id = 'base_link'
        orientation.link_name = 'R_hand_link'
        orientation.orientation = ros_pose(target).orientation
        orientation.absolute_x_axis_tolerance = .001
        orientation.absolute_y_axis_tolerance = .001
        orientation.absolute_z_axis_tolerance = .001
        orientation.weight = 1.
        constraints.position_constraints = [position]
        constraints.orientation_constraints = [orientation]
        request.goal_constraints = [constraints]
        goal.planning_options.plan_only = True
        handle = self.wait(self.move.send_goal_async(goal), 20)
        if not handle.accepted:
            raise RuntimeError('MoveGroup goal rejected')
        try:
            result = self.wait(handle.get_result_async(), 150).result
        except BaseException:
            self.wait(handle.cancel_goal_async(), 5)
            raise
        if result.error_code.val != 1:
            raise RuntimeError('MoveGroup error '+str(result.error_code.val))
        self.execute_checked(result.planned_trajectory)
        self.settle()

    def execute_checked(self, trajectory, evaluation_segment=None):
        # Explicit full dual-arm goals: the non-teaching arm is held at its
        # measured initial pose with ZERO velocity/acceleration, not inferred
        # by the controller from a partial right-arm goal.
        names = list(trajectory.joint_trajectory.joint_names)
        if not set(names).issubset(self.controller_joints):
            raise RuntimeError('Unexpected joints in right-arm plan')
        if max(abs(self.joints[n]-v) for n, v in self.left_hold.items()) > .01:
            raise RuntimeError('Non-teaching arm drifted before trajectory execution')
        indices = {n: i for i, n in enumerate(names)}
        for p in trajectory.joint_trajectory.points:
            positions, velocities, accelerations = list(p.positions), list(p.velocities), list(p.accelerations)
            for n in names:
                if n.startswith('left_') and abs(positions[indices[n]]-self.left_hold[n]) > .01:
                    raise RuntimeError('Right-arm plan unexpectedly moves the left arm')
            p.positions = [self.left_hold[n] if n in self.left_hold else positions[indices[n]] for n in self.controller_joints]
            p.velocities = [0. if n in self.left_hold or not velocities else velocities[indices[n]] for n in self.controller_joints]
            p.accelerations = [0. if n in self.left_hold or not accelerations else accelerations[indices[n]] for n in self.controller_joints]
            p.effort = []
        trajectory.joint_trajectory.joint_names = self.controller_joints
        handle = self.wait(self.execute.send_goal_async(ExecuteTrajectory.Goal(trajectory=trajectory)), 10)
        if not handle.accepted:
            raise RuntimeError('Trajectory rejected')
        future = handle.get_result_async()
        deadline, maximum = time.monotonic()+120, 0.
        try:
            while not future.done():
                rclpy.spin_once(self, timeout_sec=.05)
                if time.monotonic() > deadline:
                    raise RuntimeError('Trajectory execution timeout')
                drift = max(abs(self.joints[n]-v) for n, v in self.left_hold.items())
                self.max_left_drift = max(self.max_left_drift, float(drift))
                if drift > .02:
                    raise RuntimeError('Non-teaching arm drift > 0.02 rad')
                if evaluation_segment is not None:
                    start, end = evaluation_segment
                    actual = transform_point(self.tf('R_hand_link'), self.truth_tool[:3, 3])
                    segment = end-start
                    alpha = np.clip(np.dot(actual-start, segment)/np.dot(segment, segment), 0, 1)
                    error = float(np.linalg.norm(actual-(start+alpha*segment))*1000)
                    maximum = max(maximum, error)
                    if error > 2.:
                        raise RuntimeError('Observed workobject path error > 2 mm')
            result = future.result().result
            if result.error_code.val != 1:
                raise RuntimeError('Trajectory execution failed: '+str(result.error_code.val))
        except BaseException:
            self.wait(handle.cancel_goal_async(), 5)
            raise
        self.settle()
        return maximum

    def line(self, target, evaluation_segment=None):
        request = GetCartesianPath.Request()
        request.header.frame_id = 'base_link'
        request.start_state.is_diff = True
        request.group_name = 'right_arm'
        request.link_name = 'R_hand_link'
        request.waypoints = [ros_pose(target)]
        request.max_step = .002
        request.jump_threshold = 2.
        request.revolute_jump_threshold = .3
        request.avoid_collisions = True
        # Jazzy GetCartesianPath timing support; explicit fallback scales timestamps.
        native_scaling = hasattr(request, 'max_velocity_scaling_factor')
        if native_scaling:
            request.max_velocity_scaling_factor = self.speed
            request.max_acceleration_scaling_factor = self.speed
            request.cartesian_speed_limited_link = 'R_hand_link'
            request.max_cartesian_speed = .03
        result = self.wait(self.cartesian.call_async(request), 30)
        if result.error_code.val != 1 or result.fraction < .999:
            raise RuntimeError('Collision-checked approach incomplete: %.4f / %d' %
                               (result.fraction, result.error_code.val))
        trajectory = result.solution
        if not trajectory.joint_trajectory.points or trajectory.joint_trajectory.points[-1].time_from_start.sec == 0 and trajectory.joint_trajectory.points[-1].time_from_start.nanosec == 0:
            raise RuntimeError('Cartesian trajectory has no valid time parameterization')
        if not native_scaling:
            for p in trajectory.joint_trajectory.points:
                duration = (p.time_from_start.sec+p.time_from_start.nanosec*1e-9)/self.speed
                p.time_from_start.sec = int(duration)
                p.time_from_start.nanosec = int((duration-int(duration))*1e9)
                p.velocities = [v*self.speed for v in p.velocities]
                p.accelerations = [a*self.speed*self.speed for a in p.accelerations]
        return self.execute_checked(trajectory, evaluation_segment)

    def flange_for_tip(self, point, operator=False):
        target = self.initial.copy()
        tool = self.truth_tool if operator else self.calibrated_tool
        target[:3, 3] = point-target[:3, :3] @ tool[:3, 3]
        return target

    def measure(self, label):
        measurements = []
        for _ in range(6):
            self.settle(.08)
            flange = self.tf('R_hand_link')
            measurements.append(transform_point(flange, self.calibrated_tool[:3, 3]))
        if np.max(np.linalg.norm(measurements-np.mean(measurements, axis=0), axis=1)) > .0003:
            raise RuntimeError('Robot did not settle before teaching sample')
        point = np.mean(measurements, axis=0)+self.rng.normal(0, self.p('noise_sigma_mm')*.001, 3)
        self.rows.append([label, *flange[:3, 3], *quaternion_from_matrix(flange[:3, :3]), *point])
        return point

    def frame_visual(self, estimate):
        root = ET.Element('sdf', version='1.9')
        model = ET.SubElement(root, 'model', name='workobject_estimated_frame')
        ET.SubElement(model, 'static').text = 'true'
        link = ET.SubElement(model, 'link', name='axes')
        axes = [(0, '1 0.05 0.05 1'), (1, '0.05 1 0.05 1'), (2, '0.05 0.25 1 1')]
        for axis, color in axes:
            v = ET.SubElement(link, 'visual', name='xyz'[axis])
            p = np.zeros(3)
            p[axis] = .05
            rotations = [(0, math.pi/2, 0), (-math.pi/2, 0, 0), (0, 0, 0)]
            ET.SubElement(v, 'pose').text = ' '.join(map(str, [*p, *rotations[axis]]))
            cylinder = ET.SubElement(ET.SubElement(v, 'geometry'), 'cylinder')
            ET.SubElement(cylinder, 'radius').text = '.0015'
            ET.SubElement(cylinder, 'length').text = '.1'
            material = ET.SubElement(v, 'material')
            for field in ('ambient', 'diffuse', 'emissive'):
                ET.SubElement(material, field).text = color
        for index, (start, end) in enumerate(zip(self.rectangle[:-1], self.rectangle[1:])):
            v = ET.SubElement(link, 'visual', name='work_path_'+str(index))
            middle = (start+end)/2
            dimensions = np.abs(end-start)
            dimensions = np.maximum(dimensions, .001)
            ET.SubElement(v, 'pose').text = ' '.join(map(str, [*middle, 0, 0, 0]))
            box = ET.SubElement(ET.SubElement(v, 'geometry'), 'box')
            ET.SubElement(box, 'size').text = ' '.join(map(str, dimensions))
            material = ET.SubElement(v, 'material')
            for field in ('ambient', 'diffuse', 'emissive'):
                ET.SubElement(material, field).text = '0.05 0.9 1 1'
        self.spawn('workobject_estimated_frame', root, self.world_base @ estimate)
        messages = []
        for parent, child, transform in [('base_link', 'workobject_user', estimate),
                                         ('workobject_user', 'workobject_calibrated', np.eye(4))]:
            msg = TransformStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id, msg.child_frame_id = parent, child
            msg.transform.translation.x, msg.transform.translation.y, msg.transform.translation.z = map(float, transform[:3, 3])
            q = quaternion_from_matrix(transform[:3, :3])
            msg.transform.rotation.x, msg.transform.rotation.y, msg.transform.rotation.z, msg.transform.rotation.w = map(float, q)
            messages.append(msg)
        self.broadcaster.sendTransform(messages)

    def run(self):
        self.ready()
        self.fixture()
        measured = []
        normal = self.board[:3, 2]
        for label, point in zip(('X1', 'X2', 'Y1'), self.points):
            base_point = transform_point(self.board, point)
            approach = self.flange_for_tip(base_point+normal*.065, operator=True)
            self.get_logger().info('Teaching '+label+': transfer, straight approach, measured TCP, retract')
            self.move_pose(approach)
            self.line(self.flange_for_tip(base_point, operator=True))
            actual = transform_point(self.tf('R_hand_link'), self.truth_tool[:3, 3])
            if np.linalg.norm(actual-base_point) > .001:
                raise RuntimeError('Simulated teaching contact error > 1 mm')
            measured.append(self.measure(label))
            self.line(approach)
        estimate, geometry = solve_three_points(measured, self.p('origin_mode'))
        # Independent truth only for assessment, never passed to the solver above.
        truth = self.board.copy()
        if self.p('origin_mode') == 'first_point':
            truth[:3, 3] = transform_point(self.board, self.points[0])
        self.rectangle = np.array([[-.03, -.025, .055], [.03, -.025, .055],
                                   [.03, .025, .055], [-.03, .025, .055], [-.03, -.025, .055]])
        self.frame_visual(estimate)
        origin_error = float(np.linalg.norm(estimate[:3, 3]-truth[:3, 3])*1000)
        angle_error = float(rotation_error_deg(estimate[:3, :3], truth[:3, :3]))
        self.get_logger().info('SOLVED origin error %.4f mm / orientation %.4f deg' % (origin_error, angle_error))
        validation = []
        for index, local in enumerate([[-.035, .025, .05], [.035, .025, .05], [0, -.035, .05]]):
            target = transform_point(estimate, local)
            self.move_pose(self.flange_for_tip(target))
            actual = transform_point(self.tf('R_hand_link'), self.truth_tool[:3, 3])
            expected = transform_point(truth, local)
            error = float(np.linalg.norm(actual-expected)*1000)
            validation.append({'workobject_target_m': local, 'actual_base_m': actual.tolist(), 'error_mm': error})
            self.get_logger().info('Independent workobject target %d error %.4f mm' % (index+1, error))
        self.move_pose(self.flange_for_tip(transform_point(estimate, self.rectangle[0])))
        path_errors = []
        for index, (start, end) in enumerate(zip(self.rectangle[:-1], self.rectangle[1:])):
            self.get_logger().info('Workobject rectangle edge %d: straight motion along local X/Y' % (index+1))
            error = self.line(self.flange_for_tip(transform_point(estimate, end)),
                              (transform_point(truth, start), transform_point(truth, end)))
            path_errors.append(error)
        self.move_pose(self.initial)
        accepted = (origin_error < 1. and angle_error < 1. and
                    max(v['error_mm'] for v in validation) < 1. and max(path_errors) < 1.)
        report = {'method': 'X1-X2-Y1 three-point', 'origin_mode': self.p('origin_mode'),
                  'tcp_source': str(self.p('tcp_file')), 'noise_sigma_mm': self.p('noise_sigma_mm'),
                  'base_T_user': serial_pose(estimate), 'user_T_object': serial_pose(np.eye(4)),
                  'base_T_object': serial_pose(estimate), 'measured_points_base_m': np.asarray(measured).tolist(),
                  'geometry': geometry, 'origin_error_mm': origin_error, 'orientation_error_deg': angle_error,
                  'independent_validation': validation, 'returned_to_start': True, 'accepted': bool(accepted)}
        report['rectangle_local_points_m'] = self.rectangle.tolist()
        report['rectangle_observed_max_error_mm_per_edge'] = path_errors
        report['non_teaching_arm_max_joint_drift_rad'] = self.max_left_drift
        with (self.output/'calibration.yaml').open('w') as stream:
            yaml.safe_dump(report, stream, sort_keys=False)
        with (self.output/'samples.csv').open('w') as stream:
            writer = csv.writer(stream, lineterminator='\n')
            writer.writerow(['name', 'flange_x', 'flange_y', 'flange_z', 'qx', 'qy', 'qz', 'qw', 'tcp_x', 'tcp_y', 'tcp_z'])
            writer.writerows(self.rows)
        if not accepted:
            raise RuntimeError('Calibration acceptance failed; report saved at '+str(self.output))
        (self.output/'SUCCESS').write_text('Three taught points, three independent targets and four rectangle edges passed; robot returned to start.\n')
        self.get_logger().info('VALIDATION PASSED: '+str(self.output))


def main():
    rclpy.init()
    node = Sampler()
    try:
        node.run()
        rclpy.spin(node)  # Retain the static frame publisher for newly joining consumers.
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except Exception as exc:
        # SIGINT can invalidate the context between spin_once and wait-set
        # construction. A normal shutdown must not turn a passed run into FAILURE.
        if rclpy.ok():
            node.get_logger().error('STOPPED: '+str(exc))
            (node.output/'FAILURE').write_text(str(exc)+'\n')
            raise
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
