#!/usr/bin/env python3
"""Verify the completed first-box test using its log and live Gazebo poses."""
import argparse
import json
import math
from pathlib import Path
import threading
from gz.transport13 import Node
from gz.msgs10.pose_v_pb2 import Pose_V


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output-dir', type=Path, required=True)
    parser.add_argument('--continuation-dir', type=Path, help='Final resumed phase in the same unchanged Gazebo scene')
    args = parser.parse_args()
    out = args.output_dir.resolve()
    manifest = json.loads((out / 'manifest.json').read_text())
    initial_log = (out / 'scene.log').read_text(errors='replace')
    final_dir = args.continuation_dir.resolve() if args.continuation_dir else out
    log = (final_dir / 'scene.log').read_text(errors='replace')
    stages = json.loads((final_dir / 'key_stage_forces.json').read_text())
    audit_dirs = [out] if final_dir == out else [out] + [p for p in sorted(out.iterdir()) if p.is_dir() and (p / 'table_clearance_summary.json').exists()]
    audits = {str(p.relative_to(out)): json.loads((p / 'table_clearance_summary.json').read_text()) for p in audit_dirs}
    table = {'samples': sum(a['samples'] for a in audits.values()),
             'collision_samples': sum(a['collision_samples'] for a in audits.values()),
             'minimum_separating_gap_m': min(a['minimum_separating_gap_m'] for a in audits.values() if a['minimum_separating_gap_m'] is not None),
             'phases': audits}

    poses, event = {}, threading.Event()
    node = Node()

    def receive(msg):
        for p in msg.pose:
            if p.name in manifest['initial_box_xyz_m'] or p.name == 'zhishu_robot':
                q = p.orientation
                poses[p.name] = {'xyz_m': [p.position.x, p.position.y, p.position.z],
                                 'tilt_deg': math.degrees(math.acos(max(-1., min(1., 1 - 2*(q.x*q.x + q.y*q.y)))))}
        if all(k in poses for k in manifest['initial_box_xyz_m']):
            event.set()

    node.subscribe(Pose_V, '/world/pallet_bin_15kg/dynamic_pose/info', receive)
    if not event.wait(10):
        raise RuntimeError('Could not read all nine physical tote poses')
    target = manifest['target']
    placed = poses[target]
    center_error = math.dist(placed['xyz_m'][:2], manifest['bin_center_xy_m'])
    displacements = {k: math.dist(v, poses[k]['xyz_m']) for k, v in manifest['initial_box_xyz_m'].items() if k != target}
    by_stage = {r['stage']: r for r in stages}
    supported = by_stage.get('floor_supported_before_release')
    released = by_stage.get('release_start')
    opened = by_stage.get('fully_open')
    unloaded = by_stage.get('support_transfer_before_partial_unload')
    source_xyz = manifest['initial_box_xyz_m'][target]
    pull_samples = []
    with (out / 'wrist_force_trace.jsonl').open() as stream:
        for line in stream:
            row = json.loads(line)
            xyz = row.get('box_xyz_m')
            if xyz and abs(xyz[2] - source_xyz[2]) < .006 and abs(xyz[0] - source_xyz[0]) < .05:
                pull_samples.append(row)
    pull_distance = max((source_xyz[1] - r['box_xyz_m'][1] for r in pull_samples), default=0.)
    side_pull_snapshot = next((r for r in pull_samples if source_xyz[1] - r['box_xyz_m'][1] >= .115), None)
    if side_pull_snapshot:
        (out / 'side_pull_wrist_snapshot.json').write_text(json.dumps(side_pull_snapshot, indent=2))
    lift_audits = [a for a in audits.values() if a.get('minimum_lift_stage_horizontal_gap_m') is not None]
    minimum_lift_gap = min((a['minimum_lift_stage_horizontal_gap_m'] for a in lift_audits), default=None)
    lift_violations = sum(a.get('lift_stage_gap_violation_samples', 0) for a in lift_audits)
    checks = {
        'full_flow_completed': 'Complete transfer finished' in log and '[FATAL]' not in log,
        'side_pull_performed': 'Pre-extracting perimeter tote pallet_box_r0_c0' in initial_log and 'Relocalized pallet_box_r0_c0 world center=' in initial_log and .10 <= pull_distance <= .15,
        'center_within_5mm': center_error <= .005,
        'settled_on_floor': abs(placed['xyz_m'][2] - .636) <= .0015 and placed['tilt_deg'] <= .5,
        'other_eight_unchanged': max(displacements.values()) <= .001,
        'lift_stage_edge_at_least_200mm': bool(lift_audits) and minimum_lift_gap >= .200 and lift_violations == 0,
        'no_robot_table_intersection': table['samples'] > 0 and table['collision_samples'] == 0,
        'clamp_unloaded_before_verified_support': bool(unloaded and supported and unloaded['sim_time_s'] < supported['sim_time_s']),
        'bounded_support_before_release': bool(supported and .9*147.15 <= supported['estimated_support_n'] <= 1.1*147.15),
        'release_after_support_verified': bool(supported and released and supported['sim_time_s'] < released['sim_time_s']),
        'fully_open_recorded': bool(opened and max(opened['grippers_m']) < .001),
        'both_six_axis_signals_finite': bool(stages) and all(math.isfinite(v) for r in stages for side in ('left', 'right') for v in r[side]),
    }
    report = {'passed': all(checks.values()), 'checks': checks, 'target': target,
              'continued_same_scene': final_dir != out, 'final_phase_dir': str(final_dir),
              'final_tote_pose': placed, 'center_error_mm': center_error*1000,
              'measured_side_pull_distance_mm': pull_distance*1000,
              'other_box_displacement_m': displacements, 'table_audit': table, 'minimum_lift_stage_horizontal_gap_mm': minimum_lift_gap*1000 if minimum_lift_gap is not None else None,
              'clearance_reference_link': 'waist_up_down_link',
              'support_before_release_n': supported['estimated_support_n'] if supported else None,
              'key_stages': stages}
    (out / 'acceptance.json').write_text(json.dumps(report, indent=2))
    print(json.dumps({k: v for k, v in report.items() if k not in ('key_stages', 'other_box_displacement_m')}, indent=2))
    return 0 if report['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
