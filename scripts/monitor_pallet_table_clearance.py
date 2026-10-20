#!/usr/bin/env python3
"""Audit live robot primitive geometry against Gazebo's tabletop and legs.

Uses measured base pose and joints, independent of MoveIt's allowed-collision
matrix. Cylinders/spheres use conservative bounding boxes; URDF boxes are exact.
This diagnostic does not command motion or replace the motion collision guard.
"""
import argparse
import itertools
import json
import math
import struct
from pathlib import Path
import time
import xml.etree.ElementTree as ET
import numpy as np
import rclpy
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState


def rotation(axis, angle):
    axis = np.asarray(axis, dtype=float)
    axis /= np.linalg.norm(axis)
    x, y, z = axis
    cross = np.array([[0, -z, y], [z, 0, -x], [-y, x, 0]])
    return np.eye(3) + math.sin(angle) * cross + (1 - math.cos(angle)) * (cross @ cross)


def origin(element):
    result = np.eye(4)
    if element is not None:
        result[:3, 3] = [float(v) for v in element.get('xyz', '0 0 0').split()]
        r, p, y = [float(v) for v in element.get('rpy', '0 0 0').split()]
        result[:3, :3] = rotation([0, 0, 1], y) @ rotation([0, 1, 0], p) @ rotation([1, 0, 0], r)
    return result


def separation(a, ah, b, bh):
    """Largest signed separating-axis gap; negative means intersecting OBBs."""
    # Most robot/table pairs are far apart. An AABB separating axis proves
    # clearance immediately and avoids expensive OBB work at every sample.
    aabb_gap = np.max(np.abs(b[:3, 3] - a[:3, 3]) - np.abs(a[:3, :3]) @ ah - np.abs(b[:3, :3]) @ bh)
    if aabb_gap > 0:
        return float(aabb_gap)
    axes = [*a[:3, :3].T, *b[:3, :3].T]
    axes += [np.cross(u, v) for u in a[:3, :3].T for v in b[:3, :3].T]
    delta = b[:3, 3] - a[:3, 3]
    best = -math.inf
    for axis in axes:
        norm = np.linalg.norm(axis)
        if norm < 1e-8:
            continue
        axis = axis / norm
        radius = np.abs(a[:3, :3].T @ axis) @ ah + np.abs(b[:3, :3].T @ axis) @ bh
        best = max(best, abs(delta @ axis) - radius)
    return best


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--urdf', type=Path, required=True)
    parser.add_argument('--scene-log', type=Path, required=True)
    parser.add_argument('--output-dir', type=Path, required=True)
    args = parser.parse_args()
    root = ET.parse(args.urdf).getroot()
    joints = root.findall('joint')
    shapes = []
    for link in root.findall('link'):
        for collision in link.findall('collision'):
            geometry = collision.find('geometry')
            shape = list(geometry)[0]
            if shape.tag == 'box':
                size = np.array([float(v) for v in shape.get('size').split()])
            elif shape.tag == 'sphere':
                size = np.full(3, 2 * float(shape.get('radius')))
            elif shape.tag == 'cylinder':
                size = np.array([2 * float(shape.get('radius'))] * 2 + [float(shape.get('length'))])
            else:
                raise ValueError('Unsupported collision geometry: ' + shape.tag)
            shapes.append((link.get('name'), origin(collision.find('origin')), size / 2))
    # Use conservative local visual AABB corners plus collision corners. This
    # matches the independent C++ guard and includes both parts of the upright.
    column_points = {}
    for name in ('waist_up_down_link', 'waist_pitch_link'):
        link = root.find(f"link[@name='{name}']")
        visual = link.find('visual')
        mesh = visual.find('geometry/mesh')
        filename = mesh.get('filename')
        if filename.startswith('package://zhishu_robot_description/'):
            path = Path(__file__).resolve().parents[1] / 'src/zhishu_robot_description' / filename.split('package://zhishu_robot_description/')[1]
        else:
            path = Path(filename.removeprefix('file://'))
        raw = path.read_bytes()
        count = struct.unpack_from('<I', raw, 80)[0]
        vertices = np.frombuffer(raw, offset=84, count=count, dtype=[('n','<f4',3), ('v','<f4',(3,3)), ('a','<u2')])['v'].reshape(-1,3).astype(float)
        vertices *= np.array([float(v) for v in mesh.get('scale', '1 1 1').split()])
        bounds = np.array(list(itertools.product(*zip(vertices.min(axis=0), vertices.max(axis=0)))))
        local = origin(visual.find('origin'))
        points = [bounds @ local[:3,:3].T + local[:3,3]]
        if name == "waist_up_down_link": lift_stage_visible_points = points[0].copy()
        for collision_link, local, half in shapes:
            if collision_link == name:
                corners = np.array(list(itertools.product(*[[-h,h] for h in half])))
                points.append(corners @ local[:3,:3].T + local[:3,3])
        column_points[name] = np.vstack(points)
    fixtures = [('top', [1.03, -1.55, .525], [.75, .95, .05])]
    fixtures += [(f'leg_{dx}_{dy}', [1.03 + dx, -1.55 + dy, .25], [.06, .06, .50])
                 for dx in [-.31, .31] for dy in [-.41, .41]]
    tables = []
    for name, center, size in fixtures:
        tf = np.eye(4)
        tf[:3, 3] = center
        tables.append((name, tf, np.array(size) / 2))
    rclpy.init()
    node = rclpy.create_node('pallet_table_geometry_audit', parameter_overrides=[Parameter('use_sim_time', value=True)])
    state = {'base': None, 'joints': {}, 'base_received': 0, 'joints_received': 0}

    def base(msg):
        q = msg.pose.orientation
        # Quaternion -> rotation matrix, without assuming a level chassis.
        x, y, z, w = q.x, q.y, q.z, q.w
        tf = np.eye(4)
        tf[:3, :3] = [[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                       [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                       [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]]
        p = msg.pose.position
        tf[:3, 3] = [p.x, p.y, p.z]
        state.update(base=tf, base_received=time.monotonic())

    def joint(msg):
        state['joints'].update(zip(msg.name, msg.position))
        state['joints_received'] = time.monotonic()

    node.create_subscription(PoseStamped, '/pallet_bin/measured_base_pose', base, 10)
    node.create_subscription(JointState, '/joint_states', joint, qos_profile_sensor_data)
    out = args.output_dir
    out.mkdir(parents=True, exist_ok=True)
    summary = {'samples': 0, 'collision_samples': 0, 'minimum_separating_gap_m': math.inf,
               'minimum_waist_tabletop_gap_m_when_overlapping_xy': math.inf,
               'minimum_column_horizontal_gap_m': math.inf, 'column_gap_violation_samples': 0,
               'minimum_lift_stage_horizontal_gap_m': math.inf, 'lift_stage_gap_violation_samples': 0,
               'coverage': 'Upright visual and collision bounding envelopes; all URDF collision primitives; sphere/cylinder bounding boxes are conservative.'}
    trace = (out / 'table_clearance_trace.jsonl').open('w')

    def sample():
        now = time.monotonic()
        if state['base'] is None or now - state['base_received'] > .5 or now - state['joints_received'] > .5:
            return
        transforms = {'base_footprint': state['base']}
        pending = list(joints)
        values = state['joints']
        while pending:
            progressed = False
            for j in list(pending):
                parent, child = j.find('parent').get('link'), j.find('child').get('link')
                if parent not in transforms:
                    continue
                value = values.get(j.get('name'), 0.0)
                mimic = j.find('mimic')
                if mimic is not None:
                    value = values.get(mimic.get('joint'), 0.0) * float(mimic.get('multiplier', '1')) + float(mimic.get('offset', '0'))
                tf = origin(j.find('origin'))
                axis = np.array([float(v) for v in j.find('axis').get('xyz').split()]) if j.find('axis') is not None else np.array([1., 0, 0])
                motion = np.eye(4)
                if j.get('type') in ('revolute', 'continuous'):
                    motion[:3, :3] = rotation(axis, value)
                elif j.get('type') == 'prismatic':
                    motion[:3, 3] = axis * value
                transforms[child] = transforms[parent] @ tf @ motion
                pending.remove(j)
                progressed = True
            if not progressed:
                raise RuntimeError('Disconnected robot tree')
        collisions = []
        minimum = math.inf
        waist_gap = None
        for link, local, half in shapes:
            tf = transforms[link] @ local
            # Far objects need no separating-axis calculation.
            if np.linalg.norm(tf[:3, 3] - np.array([1.03, -1.55, .4])) > 2:
                continue
            for fixture, table, th in tables:
                gap = separation(tf, half, table, th)
                minimum = min(minimum, gap)
                if gap < -1e-5:
                    collisions.append([link, fixture, gap])
            if link == 'waist_up_down_link':
                corners = np.array([tf[:3, 3] + tf[:3, :3] @ np.array(c) for c in itertools.product(*[[-h, h] for h in half])])
                if corners[:, 0].max() >= .655 and corners[:, 0].min() <= 1.405 and corners[:, 1].max() >= -2.025 and corners[:, 1].min() <= -1.075:
                    waist_gap = float(corners[:, 2].min() - .55)
                    summary['minimum_waist_tabletop_gap_m_when_overlapping_xy'] = min(summary['minimum_waist_tabletop_gap_m_when_overlapping_xy'], waist_gap)
        column_gaps = {}
        for name, points in column_points.items():
            tf = transforms[name]
            world_points = points @ tf[:3,:3].T + tf[:3,3]
            low, high = world_points[:,:2].min(axis=0), world_points[:,:2].max(axis=0)
            gaps = np.maximum(0, np.maximum(np.array([.655,-2.025])-high, low-np.array([1.405,-1.075])))
            column_gaps[name] = float(np.linalg.norm(gaps))
        column_min = min(column_gaps.values())
        summary['minimum_column_horizontal_gap_m'] = min(summary['minimum_column_horizontal_gap_m'], column_min)
        summary['column_gap_violation_samples'] += column_min < .200
        tf = transforms['waist_up_down_link']
        visible_world = lift_stage_visible_points @ tf[:3,:3].T + tf[:3,3]
        low, high = visible_world[:,:2].min(axis=0), visible_world[:,:2].max(axis=0)
        lift_gap = float(np.linalg.norm(np.maximum(0, np.maximum(np.array([.655,-2.025])-high, low-np.array([1.405,-1.075])))))
        summary['minimum_lift_stage_horizontal_gap_m'] = min(summary['minimum_lift_stage_horizontal_gap_m'], lift_gap)
        summary['lift_stage_gap_violation_samples'] += lift_gap < .200
        row = {'lift_stage_visible_horizontal_gap_m': lift_gap, 'column_horizontal_gaps_m': column_gaps, 'base_xyz_m': state['base'][:3,3].tolist(),
               'sim_time_s' : node.get_clock().now().nanoseconds * 1e-9, 'minimum_separating_gap_m': minimum,
               'waist_tabletop_gap_m': waist_gap, 'collisions': collisions}
        trace.write(json.dumps(row) + '\n')
        trace.flush()
        summary['samples'] += 1
        summary['collision_samples'] += bool(collisions)
        summary['minimum_separating_gap_m'] = min(summary['minimum_separating_gap_m'], minimum)
        if collisions:
            print('TABLE_COLLISION', row, flush=True)

    node.create_timer(.1, sample)
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=.2)
            log = args.scene_log.read_text(errors='replace') if args.scene_log.exists() else ''
            if 'Complete transfer finished' in log or 'COLUMN_CLEARANCE_CHECK_SUCCESS' in log or '[FATAL]' in log:
                break
    except KeyboardInterrupt:
        pass
    finally:
        trace.close()
        for key, value in summary.items():
            if isinstance(value, float) and not math.isfinite(value):
                summary[key] = None
        (out / 'table_clearance_summary.json').write_text(json.dumps(summary, indent=2))
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
