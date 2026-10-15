#!/usr/bin/env python3
"""Explicit-opt-in software integration validation in mock-only ROS domain 169.

No saved TCP/workobject calibration files are consumed. Manual synthetic test
definitions live only in a temporary directory and are reset before exit.
"""
import argparse
import json
import math
import os
from pathlib import Path
import tempfile
import threading
import time
import xml.etree.ElementTree as ET
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from rclpy.time import Time
from rcl_interfaces.srv import GetParameters
from controller_manager_msgs.srv import ListControllers
from geometry_msgs.msg import PoseStamped, TransformStamped
from sensor_msgs.msg import JointState
from tf2_ros import Buffer, TransformListener, StaticTransformBroadcaster
from zhishu_robot_commander_py import Commander, ErrorCode, Offs, RelTool
from zhishu_robot_interfaces.srv import MoveCartesian


def matrix(position, q):
    x, y, z, w = np.asarray(q)/np.linalg.norm(q)
    t = np.eye(4)
    t[:3, :3] = [[1-2*y*y-2*z*z, 2*x*y-2*z*w, 2*x*z+2*y*w],
                 [2*x*y+2*z*w, 1-2*x*x-2*z*z, 2*y*z-2*x*w],
                 [2*x*z-2*y*w, 2*y*z+2*x*w, 1-2*x*x-2*y*y]]
    t[:3, 3] = position
    return t


def pose(t, frame='base_link'):
    # Rotation-matrix -> quaternion via the largest diagonal branch.
    r = t[:3, :3]
    if np.trace(r) > 0:
        s = 2*math.sqrt(1+np.trace(r))
        q = [(r[2, 1]-r[1, 2])/s, (r[0, 2]-r[2, 0])/s, (r[1, 0]-r[0, 1])/s, s/4]
    else:
        i = int(np.argmax(np.diag(r)))
        j, k = (i+1) % 3, (i+2) % 3
        s = 2*math.sqrt(1+r[i, i]-r[j, j]-r[k, k])
        q = np.zeros(4)
        q[i], q[j], q[k], q[3] = s/4, (r[j, i]+r[i, j])/s, (r[k, i]+r[i, k])/s, (r[k, j]-r[j, k])/s
    msg = PoseStamped()
    msg.header.frame_id = frame
    msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = map(float, t[:3, 3])
    msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w = map(float, q)
    return msg


def from_pose(msg):
    p, q = msg.pose.position, msg.pose.orientation
    return matrix([p.x, p.y, p.z], [q.x, q.y, q.z, q.w])


class Reader(Node):
    def __init__(self):
        super().__init__('sdk_frames_validation_reader')
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        self.joints = {}
        self.create_subscription(JointState, '/joint_states', self.on_joints, 10)
        self.watch_tool = None
        self.trace = []
        self.create_timer(.01, self.capture_path)

    def capture_path(self):
        if self.watch_tool is not None:
            try:
                self.trace.append((self.tf('R_hand_link') @ self.watch_tool)[:3, 3].copy())
            except Exception:
                pass

    def on_joints(self, msg):
        self.joints = dict(zip(msg.name, msg.position))

    def tf(self, frame):
        msg = self.buffer.lookup_transform('base_link', frame, Time()).transform
        p, q = msg.translation, msg.rotation
        return matrix([p.x, p.y, p.z], [q.x, q.y, q.z, q.w])

    def call(self, client, request):
        future = client.call_async(request)
        event = threading.Event()
        future.add_done_callback(lambda _: event.set())
        if not event.wait(10):
            raise RuntimeError('read-only service timeout')
        return future.result()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--execute', action='store_true')
    parser.add_argument('--output', default='/tmp/commander_sdk_validation.json')
    args = parser.parse_args()
    if os.environ.get('ROS_DOMAIN_ID') != '169':
        raise RuntimeError('Validation only allowed in isolated ROS_DOMAIN_ID=169')
    rclpy.init()
    reader = Reader()
    executor = SingleThreadedExecutor()
    executor.add_node(reader)
    spinner = threading.Thread(target=executor.spin, daemon=True)
    spinner.start()
    report = {'mode': 'mock_only', 'checks': [], 'accepted': False}
    robot = None
    peer = None
    try:
        client = reader.create_client(GetParameters, '/robot_state_publisher/get_parameters')
        if not client.wait_for_service(timeout_sec=30):
            raise RuntimeError('mock robot description unavailable')
        desc = reader.call(client, GetParameters.Request(names=['robot_description'])).values[0].string_value
        plugins = [p.text for p in ET.fromstring(desc).findall('ros2_control/hardware/plugin')]
        if not plugins or any(p != 'mock_components/GenericSystem' for p in plugins):
            raise RuntimeError('Refusing motion: not exclusively GenericSystem')
        control = reader.create_client(ListControllers, '/controller_manager/list_controllers')
        if not control.wait_for_service(timeout_sec=30):
            raise RuntimeError('mock controllers unavailable')
        deadline = time.monotonic()+30
        while time.monotonic() < deadline:
            active = {c.name for c in reader.call(control, ListControllers.Request()).controller if c.state == 'active'}
            if {'dual_arm_controller', 'joint_state_broadcaster'}.issubset(active):
                break
            time.sleep(.2)
        else:
            raise RuntimeError('mock controllers not active')
        if not args.execute:
            print('Mock-only checks passed; no motion requested. Use --execute for integration validation.')
            return
        names = ['right_'+n+'_joint' for n in ['shoulder_pitch', 'shoulder_roll', 'shoulder_yaw',
                                               'elbow_pitch', 'wrist_yaw', 'wrist_pitch', 'wrist_roll']]
        deadline = time.monotonic()+10
        while time.monotonic() < deadline and not set(names).issubset(reader.joints):
            time.sleep(.1)
        initial_joints = [reader.joints[n] for n in names]
        deadline = time.monotonic()+10
        while time.monotonic() < deadline and not reader.buffer.can_transform('base_link', 'R_hand_link', Time()):
            time.sleep(.1)
        initial = reader.tf('R_hand_link')
        robot = Commander(namespace='/zhishu_robot_commander_validation', default_timeout=90.)
        if not robot.wait_until_ready(20):
            raise RuntimeError('validation commander not ready')
        peer = Commander(namespace='/zhishu_robot_commander_validation', default_timeout=5.)

        def check(label, result, expected=None, tool=None, link='R_hand_link'):
            result.raise_for_error()
            time.sleep(.15)
            entry = {'name': label, 'success': True}
            if expected is not None:
                actual = reader.tf(link)
                if tool is not None:
                    actual = actual @ tool
                error = float(np.linalg.norm(actual[:3, 3]-expected[:3, 3])*1000)
                angle = float(np.degrees(np.arccos(np.clip((np.trace(actual[:3, :3].T @ expected[:3, :3])-1)/2, -1, 1))))
                entry.update(position_error_mm=error, orientation_error_deg=angle)
                if error > 1. or angle > .2:
                    raise RuntimeError(label+' endpoint accuracy failed: '+str(entry))
            report['checks'].append(entry)
            print(json.dumps(entry), flush=True)

        def linear(label, operation, expected, tool=None):
            offset = np.eye(4) if tool is None else tool
            start = (reader.tf('R_hand_link') @ offset)[:3, 3]
            reader.trace = []
            reader.watch_tool = offset
            busy_results = []
            busy_thread = None
            if label == 'MoveL Offs tool0':
                def query_during_motion():
                    time.sleep(.1)
                    busy_results.append(peer.get_coordinate_frames()[0])
                busy_thread = threading.Thread(target=query_during_motion)
                busy_thread.start()
            try:
                result = operation()
            finally:
                reader.watch_tool = None
                if busy_thread is not None:
                    busy_thread.join(timeout=8)
            check(label, result, expected, tool)
            segment = expected[:3, 3]-start
            errors = []
            for actual in reader.trace:
                alpha = 0. if np.dot(segment, segment) < 1e-15 else np.clip(
                    np.dot(actual-start, segment)/np.dot(segment, segment), 0, 1)
                errors.append(float(np.linalg.norm(actual-(start+alpha*segment))*1000))
            if not errors or max(errors) > 1.:
                raise RuntimeError(label+' observed tool path error > 1 mm or no observations')
            report['checks'][-1]['observed_path_max_mm'] = max(errors)
            if busy_thread is not None:
                if not busy_results or busy_results[0].code != ErrorCode.BUSY:
                    raise RuntimeError('coordinate management did not reject while motion was active')
                report['checks'].append({'name': 'registry BUSY during motion', 'success': True})

        result, registry = robot.get_coordinate_frames()
        result.raise_for_error()
        if 'tool1' in registry or 'wobj1' in registry:
            raise RuntimeError('Registry must start empty of test/calibration definitions')
        target = initial.copy()
        target[2, 3] += .015
        check('MoveJ default tool0/wobj0', robot.MoveJ('right', pose(target)), target)
        target = from_pose(Offs(pose(target), .01, 0, .005))
        linear('MoveL Offs tool0', lambda: robot.MoveL('right', pose(target)), target)
        tool0_target = from_pose(RelTool(pose(target), 0, 0, .005, rz_rad=.025))
        linear('MoveL RelTool tool0', lambda: robot.MoveL('right', pose(tool0_target)), tool0_target)
        gripper_target = reader.tf('right_gripper_tcp')
        gripper_target[2, 3] += .005
        check('legacy TCP defaults to gripper', robot.move_tcp_pose('right', pose(gripper_target)),
              gripper_target, link='right_gripper_tcp')
        target = reader.tf('R_hand_link')
        target[2, 3] += .005
        check('legacy TCP explicit tool0', robot.move_tcp_pose('right', pose(target), tool='tool0'), target)
        for label, kwargs in [('unknown tool', {'tool': 'missing'}), ('unknown wobj', {'wobj': 'missing'})]:
            before = [reader.joints[n] for n in names]
            result = robot.MoveJ('right', pose(target), **kwargs)
            if result.code != ErrorCode.INVALID_ARGUMENT:
                raise RuntimeError('Expected unknown-name rejection: '+label)
            if max(abs(a-reader.joints[n]) for a, n in zip(before, names)) > 1e-5:
                raise RuntimeError('Rejected command moved joints')
            report['checks'].append({'name': label+' rejected without motion', 'success': True})

        # Synthetic manual registry only; never copied to package defaults.
        with tempfile.TemporaryDirectory(prefix='sdk-manual-frames-') as temp:
            config = Path(temp)/'frames.yaml'
            config.write_text('''version: 1
tools:
  right:
    tool1:
      translation_xyz_m: [0.01, -0.005, 0.02]
      orientation_quaternion_xyzw: [0, 0, 0.09983341664682815, 0.9950041652780258]
workobjects:
  wobj1:
    translation_xyz_m: [0.05, -0.03, 0.02]
    orientation_quaternion_xyzw: [0, 0, 0.14943813247359922, 0.9887710779360422]
''')
            empty = Path(temp)/'empty.yaml'
            empty.write_text('version: 1\ntools: {left: {}, right: {}}\nworkobjects: {}\n')
            tool = matrix([.01, -.005, .02], [0, 0, math.sin(.1), math.cos(.1)])
            work = matrix([.05, -.03, .02], [0, 0, math.sin(.15), math.cos(.15)])
            check('load synthetic manual registry', robot.load_coordinate_frames(str(config)))
            local = np.linalg.inv(work) @ reader.tf('R_hand_link') @ tool
            local = from_pose(Offs(pose(local), .008, 0, .005))
            check('MoveJ named tool/wobj', robot.MoveJ('right', pose(local), tool='tool1', wobj='wobj1'), work @ local, tool)
            local = from_pose(RelTool(pose(local), .004, 0, .003, rz_rad=.02))
            linear('MoveL named tool/wobj RelTool', lambda: robot.MoveL('right', pose(local), tool='tool1', wobj='wobj1'), work @ local, tool)
            check('MoveAbsJ absolute joints with named frames', robot.MoveAbsJ('right', initial_joints,
                                                                              tool='tool1', wobj='wobj1'), initial)
            if max(abs(a-reader.joints[n]) for a, n in zip(initial_joints, names)) > .001:
                raise RuntimeError('MoveAbsJ shifted absolute joints')
            check('reset empty registry', robot.load_coordinate_frames(str(empty)))
        reference = matrix([.02, .01, .03], [0, 0, math.sin(.05), math.cos(.05)])
        broadcaster = StaticTransformBroadcaster(reader)
        ref_msg = TransformStamped()
        ref_msg.header.frame_id, ref_msg.child_frame_id = 'base_link', 'sdk_validation_reference'
        ref_msg.header.stamp = reader.get_clock().now().to_msg()
        ref_msg.transform.translation.x, ref_msg.transform.translation.y, ref_msg.transform.translation.z = map(float, reference[:3, 3])
        ref_msg.transform.rotation = pose(reference).pose.orientation
        broadcaster.sendTransform(ref_msg)
        time.sleep(.5)
        expected = reader.tf('R_hand_link')
        expected[2, 3] += .005
        local = np.linalg.inv(reference) @ expected
        linear('MoveL transforms external frame_id', lambda: robot.MoveL(
            'right', pose(local, 'sdk_validation_reference')), expected)
        left_initial = reader.tf('L_hand_link')
        left_names = [n.replace('right_', 'left_') for n in names]
        left_joints = [reader.joints[n] for n in left_names]
        left_target = left_initial.copy()
        left_target[2, 3] += .005
        check('left MoveJ default tool0', robot.MoveJ('left', pose(left_target)), left_target, link='L_hand_link')
        check('left MoveAbsJ restores joints', robot.MoveAbsJ('left', left_joints), left_initial, link='L_hand_link')
        check('right MoveAbsJ final restore', robot.MoveAbsJ('right', initial_joints), initial)
        result, registry = robot.get_coordinate_frames()
        result.raise_for_error()
        if 'tool1' in registry or 'wobj1' in registry:
            raise RuntimeError('Synthetic definitions not cleared')
        # Expired queued commands must never reach planning/execution.
        raw = reader.create_client(MoveCartesian, '/zhishu_robot_commander_validation/move_cartesian')
        raw.wait_for_service(timeout_sec=5)
        expired = reader.call(raw, MoveCartesian.Request(group='right', right_target=pose(initial), deadline_unix_ms=1))
        if expired.error_code != int(ErrorCode.TIMEOUT):
            raise RuntimeError('Expired command was not rejected')
        report['checks'].append({'name': 'expired queued command rejected', 'success': True})
        report['accepted'] = True
        print('SDK INTEGRATION PASSED', flush=True)
    except Exception as exc:
        report['failure'] = str(exc)
        raise
    finally:
        Path(args.output).write_text(json.dumps(report, indent=2)+'\n')
        if robot:
            robot.close()
        if peer:
            peer.close()
        executor.shutdown(timeout_sec=2)
        spinner.join(timeout=2)
        reader.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
