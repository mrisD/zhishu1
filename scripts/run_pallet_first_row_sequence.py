#!/usr/bin/env python3
"""Three physical transfers in one Gazebo scene; explicit destination resets."""
import argparse
import copy
import json
import os
import re
from pathlib import Path
import signal
import subprocess
import time
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[1]
YS = [-0.400617, 0.002017, 0.406220]

def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output-dir', type=Path, required=True)
    parser.add_argument('--continue-after-output', type=Path, help='Resume third-row side pulls after the verified c0 empty-return checkpoint')
    parser.add_argument('--continue-after-placement-output', type=Path,
                        help='Also preserve verified c1 placement/home from this output; reconstruct remaining four-box scene, explicitly discarding incomplete empty return')
    parser.add_argument('--resume-checkpoint', type=Path)
    parser.add_argument('--reuse-scene', action='store_true')
    parser.add_argument('--start-col', type=int, choices=(0, 1, 2))
    parser.add_argument('--single-box', action='store_true')
    parser.add_argument('--middle-only', action='store_true', help='Remove front/rear rows; transfer the original middle row')
    parser.add_argument('--source-row', type=int, choices=(0, 1, 2),
                        help='Source row index; defaults to 1 for --middle-only and 0 otherwise')
    parser.add_argument('--middle-relocation', action='store_true', help='Continuous verified c2 side push, then c1/c0 front clamp pulls')
    parser.add_argument('--rear-relocation', action='store_true', help='Push the middle-row first box from the opposite edge, then return and transfer')
    parser.add_argument('--rear-side-pull', action='store_true', help='Keep both remaining rows; mirror first-row side pull at the rear dock, then transfer c0/c1/c2')
    parser.add_argument('--rear-direct', action='store_true', help='Directly grasp a back-row box from the rear station without pre-extraction')
    parser.add_argument('--pre-extract-pitch', type=float, default=-0.785)
    parser.add_argument('--hook-approach-height', type=float, default=0.22)
    parser.add_argument('--hook-x-offset', type=float, default=-0.10)
    parser.add_argument('--stop-on-failure', action='store_true')
    parser.add_argument('--first-already-extracted', action='store_true',
                        help='Continue from measured extraction, without pulling the same box twice')
    parser.add_argument('--first-already-held', action='store_true')
    parser.add_argument('--resume-floor-seating', action='store_true')
    parser.add_argument('--no-record', action='store_true', help='Run the Gazebo sequence without observer cameras or video encoding')
    parser.add_argument('--rear-station-test-offset', type=float, default=0.0,
                        help='Initial rear-station lateral error in metres, to verify automatic correction')
    args = parser.parse_args()
    rear_scene = args.rear_direct or args.rear_side_pull
    prior_completed = []
    if args.continue_after_output:
        if not args.rear_side_pull:
            raise ValueError('Continuation requires --rear-side-pull')
        prior_log = (args.continue_after_output/'box_0.log').read_text(errors='replace')
        prior_sequence = (args.continue_after_output/'sequence.log').read_text(errors='replace')
        if 'Complete transfer finished' not in prior_log or 'Empty robot returned to pallet; destination clearance now permitted' not in prior_log or 'SEQUENCE_REMOVED pallet_box_r2_c0' not in prior_sequence:
            raise ValueError('Continuation lacks verified c0 placement, empty return and destination clearance')
        prior_completed = ['pallet_box_r2_c0']
    if args.continue_after_placement_output:
        if not args.continue_after_output or not args.rear_side_pull:
            raise ValueError('Placement continuation requires the verified c0 checkpoint and rear side pull')
        placed_log = (args.continue_after_placement_output/'box_1.log').read_text(errors='replace')
        placed_sequence = (args.continue_after_placement_output/'sequence.log').read_text(errors='replace')
        expected_parent = f'SEQUENCE_CHECKPOINT verified c0 return from {args.continue_after_output}'
        for marker in ('Gazebo measured pallet_box_r2_c1', 'Floor support verified BEFORE opening',
                       'Physical large-bin placement verified', 'return both arms home: execution completed'):
            if marker not in placed_log:
                raise ValueError(f'Placement continuation lacks {marker}')
        if expected_parent not in placed_sequence:
            raise ValueError('Second-box placement does not reference this verified first-box checkpoint')
        prior_completed.append('pallet_box_r2_c1')
    if args.rear_side_pull and (args.source_row != 2 or args.rear_direct or args.rear_relocation or args.middle_relocation or args.single_box or args.start_col is not None):
        raise ValueError("--rear-side-pull requires the full third row and a fresh scene")
    if args.rear_relocation or rear_scene:
        args.middle_only = True
        if args.rear_relocation:
            args.start_col = 2
    if args.rear_relocation or rear_scene:
        import zlib
        os.environ['ROS_DOMAIN_ID'] = str(20 + zlib.crc32(str(args.output_dir).encode()) % 180)
        os.environ['GZ_PARTITION'] = args.output_dir.name
    if args.rear_direct and (args.source_row != 2 or args.rear_relocation or args.middle_relocation):
        raise ValueError('--rear-direct requires --source-row 2 and cannot be combined with relocation modes')
    if args.middle_relocation:
        args.middle_only = True
        if args.start_col is not None or args.single_box or args.resume_checkpoint or args.reuse_scene:
            raise ValueError('Middle relocation requires all three boxes in a fresh continuous scene')
    row = args.source_row if args.source_row is not None else (1 if args.middle_only else 0)
    if args.source_row is not None:
        args.middle_only = True
    if args.middle_only and (args.resume_checkpoint or args.reuse_scene):
        raise ValueError('Middle-only recording requires a fresh, explicitly configured scene')
    out = args.output_dir.resolve()
    out.mkdir(parents=True, exist_ok=False)
    start_col = 1 if args.resume_checkpoint or args.reuse_scene else 0
    if args.start_col is not None: start_col = args.start_col
    if prior_completed: start_col = len(prior_completed)
    end_col = start_col + 1 if args.single_box else 3
    order = ([start_col] if args.single_box else [2, 1, 0]) if (args.rear_relocation or args.rear_direct) else ([2, 1, 0] if args.middle_relocation else list(range(start_col, end_col)))
    if args.resume_checkpoint:
        evidence = (args.resume_checkpoint/'box_0.log').read_text(errors='replace')
        if 'Empty robot returned to pallet; destination clearance now permitted' not in evidence:
            raise RuntimeError('Checkpoint lacks verified first-box return')
    tree = ET.parse(ROOT/'src/zhishu_robot_gazebo/worlds/pallet_bin_15kg.sdf')
    world = tree.getroot().find('world')
    reference = world.find("model[@name='pallet_box_r0_c0']")
    for col in range(3):
        name = f'pallet_box_r{row}_c{col}'
        original = world.find(f"model[@name='{name}']")
        replacement = copy.deepcopy(reference)
        replacement.set('name', name)
        replacement.find('pose').text = original.find('pose').text
        replacement.find('link/sensor/topic').text = (
            f'/world/pallet_bin_15kg/model/{name}/link/box_link/sensor/tote_bottom_contact/contact')
        world.remove(original)
        world.append(replacement)
    if args.middle_only:
        for other_row in range(3):
            if other_row == row or (rear_scene and other_row == 1):
                continue
            for col in range(3):
                world.remove(world.find(f"model[@name='pallet_box_r{other_row}_c{col}']"))
    for completed_name in prior_completed:
        world.remove(world.find(f"model[@name='{completed_name}']"))
    if args.single_box and not args.reuse_scene and not args.rear_relocation:
        for col in range(3):
            if col != start_col:
                world.remove(world.find(f"model[@name='pallet_box_r{row}_c{col}']"))
    elif start_col and not args.rear_relocation and not rear_scene:
        world.find("model[@name='pallet_box_r0_c0']/pose").text = '0.9737387545 -1.5496559487 0.6359998333 0 0 0'
    if start_col:
        import yaml
        positions = yaml.safe_load((ROOT/'src/zhishu_robot_moveit_config/config/pallet_bin_initial_positions.yaml').read_text())
        joints = positions['initial_positions']
        home = ET.parse(ROOT/'src/zhishu_robot_moveit_config/config/zhishu_robot.srdf').getroot().find("group_state[@name='both_arms_home']")
        joints.update({j.attrib['name']:float(j.attrib['value']) for j in home})
        joints.update(waist_up_down_joint=0.21, waist_pitch_joint=-0.20)
        (out/'checkpoint_positions.yaml').write_text(yaml.safe_dump(positions))
    if args.middle_relocation or args.rear_relocation or rear_scene:
        import yaml
        positions = yaml.safe_load((ROOT/'src/zhishu_robot_moveit_config/config/pallet_bin_initial_positions.yaml').read_text())
        home = ET.parse(ROOT/'src/zhishu_robot_moveit_config/config/zhishu_robot.srdf').getroot().find("group_state[@name='both_arms_home']")
        positions['initial_positions'].update({j.attrib['name']:float(j.attrib['value']) for j in home})
        positions['initial_positions'].update(waist_up_down_joint=0.21, waist_pitch_joint=-0.20)
        if args.rear_relocation or rear_scene:
            positions['initial_positions']['left_shoulder_roll_joint'] = -0.261799
        (out/'checkpoint_positions.yaml').write_text(yaml.safe_dump(positions))
    if args.rear_relocation or rear_scene:
        urdf = subprocess.check_output(['xacro', str(ROOT/'src/zhishu_robot_moveit_config/config/zhishu_robot.urdf.xacro'),
            'use_gazebo:=true', 'use_fake_hardware:=false',
            f'initial_positions_file:={out}/checkpoint_positions.yaml', 'use_head_camera:=false',
            'mobile_base_sim_enabled:=true', 'mobile_base_sim_mass:=71.53',
            'waist_pitch_min:=-1.57079632679', 'tote_finger_length:=0.360'], text=True)
        ET.ElementTree(ET.fromstring(urdf)).write(out/'preloaded_robot.urdf', encoding='utf-8', xml_declaration=True)
        robot = ET.fromstring(subprocess.check_output(['gz', 'sdf', '-p', str(out/'preloaded_robot.urdf')], text=True)).find('model')
        robot.set('name', 'zhishu_robot')
        pose = robot.find('pose')
        if pose is None: pose = ET.SubElement(robot, 'pose')
        pose.text = f'2.068 {YS[start_col]-0.12 if args.rear_side_pull else YS[2]-(0.0 if args.rear_direct else 0.10)+args.rear_station_test_offset} 0.004 0 0 3.14159265359'
        world.append(robot)
    tree.write(out/'sequence_source.sdf', encoding='utf-8', xml_declaration=True)
    world_file = out/'sequence_source.sdf'
    if not args.no_record:
        subprocess.run(['python3', str(ROOT/'scripts/record_pallet_bin_transfer.py'), 'world',
                        '--source', str(out/'sequence_source.sdf'), '--output', str(out/'video_world.sdf')], check=True)
        world_file = out/'video_world.sdf'
    processes = []
    handles = []
    def start(command, filename):
        handle = (out/filename).open('w')
        handles.append(handle)
        proc = subprocess.Popen(command, stdout=handle, stderr=subprocess.STDOUT, start_new_session=True)
        processes.append(proc)
        return proc
    def stop(proc):
        if proc.poll() is None:
            os.killpg(proc.pid, signal.SIGINT)
            try: proc.wait(timeout=30)
            except subprocess.TimeoutExpired:
                os.killpg(proc.pid, signal.SIGTERM)
                proc.wait(timeout=15)
    combined = out/'sequence.log'
    combined.touch()
    def event(line):
        with combined.open('a') as log: log.write(line+'\n')
        print(line, flush=True)
    common = ['headless:=true', 'use_rviz:=false', 'waist_pitch_min:=-1.57079632679',
              'tote_finger_length:=0.360', f'pre_extract_waist_pitch:={args.pre_extract_pitch}',
              'grasp_waist_pitch:=-1.10', 'carry_waist_lift:=0.25', 'place_waist_lift:=0.21',
              'carry_center_height:=0.94', 'dual_ik_attempts:=400',
              'place_center_height:=0.94', 'base_max_linear:=0.12', 'base_max_angular:=0.18',
              'base_linear_acceleration:=0.08', 'base_angular_acceleration:=0.10',
              'place_on_floor_before_release:=true', 'base_motion_timeout:=600.0',
              'velocity_scaling:=0.12', 'acceleration_scaling:=0.08',
              'source_grasp_x:=0.0', 'pre_extract_distance:=0.12', 'validate_all_nine:=false',
              'align_with_extracted_center:=true', 'prefer_symmetric_arm_branch:=true']
    if args.middle_only: common.append(f'source_row_only:={row}')
    if rear_scene: common.append('retain_middle_row:=true')
    if args.rear_relocation:
        common += ['rear_push_execute:=true', 'side_push_lean:=0.90',
                   'side_push_overtravel:=0.020', 'use_head_camera:=false',
                   'base_position_tolerance:=0.005', 'base_yaw_tolerance:=0.005',
                   f'initial_positions_file:={out}/checkpoint_positions.yaml',
                   'side_push_waist_pitch:=-1.5707', 'side_push_waist_lift:=0.0']
    if args.rear_side_pull:
        common += ['rear_side_pull:=true', 'rear_source_dock_x:=2.068', 'base_position_tolerance:=0.002', 'base_yaw_tolerance:=0.002']
    if args.rear_direct:
        common += ['rear_source_dock_x:=2.068', 'grasp_half_span:=0.165']
    launch = ['ros2', 'launch', 'zhishu_robot_mtc_pick_place', 'pallet_bin_15kg_transfer.launch.py']
    completed = list(prior_completed)
    try:
        spawn = ([f'initial_positions_file:={out}/checkpoint_positions.yaml',
                  'spawn_x:=0.0035494062', 'spawn_y:=-0.4012827425', 'spawn_yaw:=-0.005'] if start_col else [])
        if not args.resume_checkpoint and not args.reuse_scene:
            spawn = ([f'initial_positions_file:={out}/checkpoint_positions.yaml'] if start_col else []) + [
                'spawn_x:=0.0', f'spawn_y:={YS[start_col]-0.12}', 'spawn_yaw:=0.0']
        if args.middle_relocation or args.rear_relocation or rear_scene:
            spawn = [f'initial_positions_file:={out}/checkpoint_positions.yaml',
                     *(['spawn_x:=2.068', f'spawn_y:={YS[2]-0.10}', 'spawn_yaw:=3.14159265359']
                       if args.rear_relocation else ['spawn_x:=1.918', f'spawn_y:={YS[2]}', 'spawn_yaw:=3.14159265359'])]
        scene = None if args.reuse_scene else start(launch+common+spawn+['start_transfer:=false', 'measurement_bridge:=false',
                      *(['spawn_robot:=false'] if (args.rear_relocation or rear_scene) else []),
                      f'world_file:={world_file}'], 'scene.log')
        recorder = None if args.no_record else start(['python3', str(ROOT/'scripts/record_pallet_bin_transfer.py'), 'record',
                         '--output-dir', str(out), '--log', str(combined), '--speedup', '8',
                         '--finger-length-mm', '360', '--tcp-offset-mm', '300',
                         '--waist-limit-deg', '-90', '--floor-seated',
                         '--target-model', f'pallet_box_r{row}_c{order[0]}',
                         *(['--rear-push'] if args.rear_relocation else []),
                         '--initial-completed-transfers', str(len(prior_completed)),
                         '--expected-transfers', str(len(order)+len(prior_completed)), '--total-boxes', '3', '--max-seconds', '16000'], 'recorder.log')
        if prior_completed: event(f'SEQUENCE_CHECKPOINT verified c0 return from {args.continue_after_output}; resume with {6-len(prior_completed)} remaining source boxes')
        if args.continue_after_placement_output:
            event(f'SEQUENCE_PLACEMENT_CHECKPOINT verified c1 placement/home from {args.continue_after_placement_output}; empty return incomplete; reconstructed four remaining source boxes for c2')
        time.sleep(5 if args.reuse_scene else 60)
        from gz.transport13 import Node
        from gz.msgs10.entity_pb2 import Entity
        from gz.msgs10.boolean_pb2 import Boolean
        node = Node()
        if args.resume_checkpoint and not args.reuse_scene:
            event('SEQUENCE_CHECKPOINT restored verified first placement and empty return; first transfer not repeated')
            request = Entity(name='pallet_box_r0_c0', type=Entity.MODEL)
            event('SEQUENCE_CLEAR_DESTINATION pallet_box_r0_c0')
            ok, response = node.request('/world/pallet_bin_15kg/remove', request, Entity, Boolean, 5000)
            if not ok or not response.data: raise RuntimeError('Checkpoint destination removal failed')
            event('SEQUENCE_REMOVED pallet_box_r0_c0')
            time.sleep(5)
        for cycle_index, col in enumerate(order):
            y = YS[col] - 0.12
            name = f'pallet_box_r{row}_c{col}'
            event('SEQUENCE_TARGET '+name)
            options = ['reuse_gazebo:=true', 'execute:=true',
                    f'pre_extract_enabled:={str(not args.rear_direct and not ((args.first_already_extracted or args.first_already_held) and col == start_col)).lower()}',
                    f'resume_after_pick:={str(args.first_already_held and col == start_col).lower()}', 'transfer_start_delay:=5.0',
                    f'resume_floor_seating:={str(args.resume_floor_seating and col == start_col).lower()}',
                    f'target_model:={name}', f'pre_extract_box_index:={3*row+col}',
                    f'full_transfer_box_index:={3*row+col}', f'cleared_first_row_count:={0 if args.rear_direct else col}',
                    f'completed_source_columns:={[int(n[-1]) for n in completed] or [-1]}',
                    f'source_grasp_y:={y}',
                    f'hook_pose_x_offset:={args.hook_x_offset}',
                    f'hook_approach_height:={args.hook_approach_height}',
                    f'return_to_pallet_after_transfer:={str(cycle_index < len(order)-1).lower()}']
            if rear_scene:
                options.append(f'source_grasp_y:={(YS[order[cycle_index+1]] if cycle_index+1 < len(order) else YS[col]) - (0.12 if args.rear_side_pull else 0.0)}')
            if args.middle_relocation or args.rear_relocation:
                # Override the legacy near-row hook/ascending-column settings.
                options = ['reuse_gazebo:=true', 'execute:=true', 'pre_extract_enabled:=false',
                    'transfer_start_delay:=5.0', f'target_model:={name}', f'full_transfer_box_index:={3*row+col}',
                    'cleared_first_row_count:=0',
                    'side_push_continue_transfer:=true', 'side_push_distance:=0.324',
                    f'side_push_execute:={str(col == 2).lower()}', f'front_pull_execute:={str(col != 2).lower()}',
                    f'completed_source_columns:={[int(n[-1]) for n in completed] or [-1]}',
                    f'source_grasp_y:={YS[order[cycle_index+1]] + 0.10 if cycle_index+1 < len(order) else YS[col]}',
                    f'return_to_pallet_after_transfer:={str(cycle_index < len(order)-1).lower()}']
                if args.rear_relocation:
                    options += ['side_push_execute:=true', 'front_pull_execute:=false',
                                f'source_grasp_y:={YS[order[cycle_index+1]] if cycle_index+1 < len(order) else YS[col]}']
            transfer = start(launch+common+options, f'box_{col}.log')
            offset = 0
            deadline = time.monotonic()+5000
            success = False
            while time.monotonic()<deadline:
                time.sleep(1)
                with (out/f'box_{col}.log').open(errors='replace') as log:
                    log.seek(offset); chunk=log.read(); offset=log.tell()
                if chunk:
                    with combined.open('a') as log: log.write(chunk)
                if (args.stop_on_failure or args.middle_relocation or args.rear_relocation) and (
                        '[FATAL]' in chunk or 'process has died' in chunk):
                    raise RuntimeError(f'{name} failed; uninterrupted validation rejected')
                if args.rear_side_pull and not args.no_record:
                    telemetry_file = out/'telemetry.jsonl'
                    if telemetry_file.exists():
                        with telemetry_file.open('rb') as telemetry_log:
                            telemetry_log.seek(max(0, telemetry_file.stat().st_size - 4096))
                            samples = telemetry_log.read().splitlines()
                        if len(samples) > 1:
                            latest = json.loads(samples[-1])
                            if latest.get('stage') == 1 and latest.get('target') == name:
                                base_pose = latest.get('base', [])
                                box_pose = latest.get('box', [])
                                if base_pose and (abs(base_pose[0]-2.068) > 0.04 or abs(base_pose[1]-(YS[col]-0.12)) > 0.04):
                                    raise RuntimeError('Side pull rejected: base drift exceeded 40 mm')
                                if box_pose and (box_pose[2] < 0.20 or latest.get('box_tilt_deg', 0) > 8.0):
                                    raise RuntimeError('Side pull rejected: unsupported or tilted tote')
                if 'Complete transfer finished' in chunk:
                    success = True
                    break
                if '[FATAL]' in chunk:
                    event('SEQUENCE_ATTENTION '+name+' failed; preserving scene for recovery')
                    while True: time.sleep(5)
                if transfer.poll() is not None or (scene is not None and scene.poll() is not None) or (recorder is not None and recorder.poll() is not None):
                    statuses = {'transfer':transfer.poll(), 'scene':scene.poll() if scene else None,
                                'recorder':recorder.poll() if recorder else None}
                    event(f'SEQUENCE_PROCESS_EXIT {statuses}')
                    raise RuntimeError(f'Process exited before {name} completion: {statuses}')
            if not success: raise RuntimeError(f'{name} timeout')
            stop(transfer)
            completed.append(name)
            if cycle_index < len(order)-1:
                content=(out/f'box_{col}.log').read_text(errors='replace')
                if 'Empty robot returned to pallet; destination clearance now permitted' not in content:
                    raise RuntimeError('Refusing removal before confirmed empty return')
                event('SEQUENCE_CLEAR_DESTINATION '+name)
                time.sleep(3)
                request=Entity(name=name, type=Entity.MODEL)
                ok, response=node.request('/world/pallet_bin_15kg/remove', request, Entity, Boolean, 5000)
                if not ok or not response.data: raise RuntimeError('Gazebo refused destination removal')
                event('SEQUENCE_REMOVED '+name)
                time.sleep(5)
        if recorder is not None:
            recorder.wait(timeout=45)
            recording = json.loads((out/'recording_summary.json').read_text())
            if not recording.get('passed') or recording.get('completed_transfers') != len(completed):
                raise RuntimeError('Recorder lacks verified completion evidence')
            if recorder.returncode:
                event(f'SEQUENCE_RECORDER_TEARDOWN_WARNING exit={recorder.returncode}; finalized summary verified; decode videos before delivery')
        (out/'sequence_result.json').write_text(json.dumps({'passed':True, 'completed':completed,
            'removed_after_return':[name for name in completed[:-1]
                                    if not (args.continue_after_placement_output and name == 'pallet_box_r2_c1')],
            'omitted_after_prior_placement':['pallet_box_r2_c1'] if args.continue_after_placement_output else [],
            'single_continuous_scene':not args.reuse_scene and not args.resume_checkpoint and not prior_completed,
            'continued_after_output':str(args.continue_after_output) if args.continue_after_output else None,
            'prior_placement_without_empty_return':str(args.continue_after_placement_output) if args.continue_after_placement_output else None,
            'prior_completed':prior_completed, 'recorded_column_order':order,
            'prior_verified_checkpoint':str(args.resume_checkpoint) if args.resume_checkpoint else None,
            'start_column':start_col, 'reused_scene':args.reuse_scene,
            'source_row':row, 'removed_initial_rows':[r for r in range(3) if args.middle_only and r != row and not (rear_scene and r == 1)],
            'column_order':order, 'middle_relocation':args.middle_relocation,
            'rear_relocation':args.rear_relocation, 'rear_side_pull':args.rear_side_pull,
            'retained_source_rows':([1, 2] if rear_scene else [row]),
            'support_deck':{'center':[1.009355,-0.805,0.075], 'size':[1.25,0.41,0.15]},
            'no_record':args.no_record, 'rear_station_test_offset':args.rear_station_test_offset,
            'single_box_test':args.single_box, 'reject_any_fatal':args.stop_on_failure}, indent=2))
        event(f'SEQUENCE_SUCCESS {len(completed)} continuation transfers completed')
    finally:
        for proc in reversed(processes):
            try:
                stop(proc)
            except subprocess.TimeoutExpired:
                if proc.poll() is None:
                    os.killpg(proc.pid, signal.SIGKILL)
                    proc.wait(timeout=5)
        # Gazebo may detach from launch's process group. Match only this
        # unique recording world, never terminate unrelated simulations.
        result = subprocess.run(['pgrep', '-f', 'gz sim -s.*' + re.escape(str(world_file)) + r'(\s|$)'],
                                capture_output=True, text=True)
        for value in result.stdout.split():
            pid = int(value)
            command = subprocess.run(['ps', '-p', str(pid), '-o', 'args='], capture_output=True, text=True).stdout.strip()
            if command.startswith('gz sim -s') and str(world_file) in command:
                os.kill(pid, signal.SIGTERM)
        for handle in handles: handle.close()

if __name__=='__main__': main()
