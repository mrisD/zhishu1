#!/usr/bin/env python3
"""Save both wrist six-axis signals and the controller's exact stage snapshots."""
import argparse
import csv
import json
import math
from pathlib import Path
import re
import time
import rclpy
from rclpy.parameter import Parameter
from geometry_msgs.msg import WrenchStamped, PoseStamped
from sensor_msgs.msg import JointState
from rclpy.qos import qos_profile_sensor_data


STAGE = re.compile(
    r'WRIST_FT_STAGE stage=(.*?) sim_time=([\d.]+) left_stamp=([\d.]+) '
    r'right_stamp=([\d.]+) left=\[([^]]+)\] right=\[([^]]+)\] '
    r'estimated_support=([-\d.]+) grippers=\[([^]]+)\]')


def export_stages(text, out):
    stages = []
    for match in STAGE.finditer(text):
        stage, t, lt, rt, left, right, support, jaws = match.groups()
        stages.append({'stage': stage, 'sim_time_s': float(t),
                       'left_stamp_s': float(lt), 'right_stamp_s': float(rt),
                       'left': [float(v) for v in left.split(',')],
                       'right': [float(v) for v in right.split(',')],
                       'estimated_support_n': float(support),
                       'grippers_m': [float(v) for v in jaws.split(',')]})
    (out / 'key_stage_forces.json').write_text(json.dumps(stages, indent=2))
    with (out / 'key_stage_forces.csv').open('w', newline='') as stream:
        writer = csv.writer(stream)
        writer.writerow(['stage', 'sim_time_s', 'side', 'sensor_stamp_s',
                         'Fx_N', 'Fy_N', 'Fz_N', 'Tx_Nm', 'Ty_Nm', 'Tz_Nm',
                         'force_norm_N', 'estimated_support_N'])
        for row in stages:
            for side in ('left', 'right'):
                values = row[side]
                writer.writerow([row['stage'], row['sim_time_s'], side,
                                 row[side + '_stamp_s'], *values,
                                 math.sqrt(sum(v*v for v in values[:3])),
                                 row['estimated_support_n']])
    return stages


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--scene-log', type=Path, required=True)
    parser.add_argument('--output-dir', type=Path, required=True)
    args = parser.parse_args()
    out = args.output_dir.resolve()
    out.mkdir(parents=True, exist_ok=True)
    rclpy.init()
    node = rclpy.create_node('pallet_wrist_force_recorder', parameter_overrides=[
        Parameter('use_sim_time', Parameter.Type.BOOL, True)])
    samples, state = {}, {}

    def wrench(side, msg):
        f, t = msg.wrench.force, msg.wrench.torque
        samples[side] = {'stamp_s': msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9,
                         'frame_id': msg.header.frame_id,
                         'wrench': [f.x, f.y, f.z, t.x, t.y, t.z]}

    def joints(msg):
        values = dict(zip(msg.name, msg.position))
        state['grippers_m'] = [values.get('left_gripper_left_finger_joint'),
                                values.get('gripper_left_finger_joint')]

    for side in ('left', 'right'):
        node.create_subscription(WrenchStamped, f'/{side}_wrist_ft',
            lambda msg, side=side: wrench(side, msg), qos_profile_sensor_data)
    node.create_subscription(WrenchStamped, '/pallet_bin/estimated_bottom_reaction',
        lambda msg: state.update(estimated_support_n=msg.wrench.force.z), 10)
    node.create_subscription(PoseStamped, '/pallet_bin/measured_box_pose',
        lambda msg: state.update(box_xyz_m=[msg.pose.position.x, msg.pose.position.y,
                                           msg.pose.position.z]), 10)
    node.create_subscription(JointState, '/joint_states', joints, qos_profile_sensor_data)
    deadline, next_check, last_stamp, count = time.monotonic() + 1800, 0.0, -1.0, 0
    text, complete = '', False
    try:
        with (out / 'wrist_force_trace.jsonl').open('w') as trace:
            while time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=0.01)
                if len(samples) == 2:
                    left, right = samples['left'], samples['right']
                    stamp = max(left['stamp_s'], right['stamp_s'])
                    if stamp - last_stamp >= 0.02 and abs(left['stamp_s'] - right['stamp_s']) <= 0.02:
                        trace.write(json.dumps({'sim_time_s': stamp, **samples, **state}) + '\n')
                        last_stamp = stamp
                        count += 1
                if time.monotonic() >= next_check:
                    next_check = time.monotonic() + 1.0
                    trace.flush()
                    text = args.scene_log.read_text(errors='replace')
                    if 'Complete transfer finished' in text:
                        complete = True
                        break
                    if '[FATAL]' in text:
                        break
    finally:
        stages = export_stages(args.scene_log.read_text(errors='replace'), out)
        (out / 'recording_status.json').write_text(json.dumps({
            'transfer_complete': complete, 'samples': count,
            'stage_snapshots': len(stages),
            'raw_order': ['Fx_N', 'Fy_N', 'Fz_N', 'Tx_Nm', 'Ty_Nm', 'Tz_Nm'],
            'coordinate_convention': 'Each sensor local axes; signs and axes differ between hands.'}, indent=2))
        node.destroy_node()
        rclpy.shutdown()
    print('Transfer complete:', complete, 'raw samples:', count, 'stages:', len(stages))
    return 0 if complete else 1


if __name__ == '__main__':
    raise SystemExit(main())
