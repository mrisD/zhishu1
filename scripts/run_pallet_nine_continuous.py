#!/usr/bin/env python3
"""Nine physical transfers, one Gazebo instance and one uninterrupted clean recording.

No resume, reconstruction, source-box isolation, or video assembly is supported.
The legacy receiving-bin clearance occurs only after verified empty return.
"""
import argparse
import copy
import json
import math
import os
from pathlib import Path
import re
import signal
import subprocess
import threading
import time
import xml.etree.ElementTree as ET
import zlib

ROOT = Path(__file__).resolve().parents[1]
YS = [-0.400617, 0.002017, 0.406220]
ORDER = [0, 1, 2, 6, 7, 8, 5, 4, 3]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output-dir', type=Path, required=True)
    parser.add_argument('--no-record', action='store_true', help='Run the same nine-box motion without cameras or video encoding')
    parser.add_argument('--record-after-pass', action='store_true', help='After a successful unrecorded run, start a fresh recorded run')
    parser.add_argument('--navigation-check-only', action='store_true')
    args = parser.parse_args()
    if args.navigation_check_only and not args.no_record:
        parser.error('navigation check requires --no-record')
    if args.record_after_pass and not args.no_record:
        parser.error('--record-after-pass requires --no-record')
    recording_hold = ROOT / 'output/pallet_nine_recording_hold.json'
    if not args.no_record and recording_hold.exists():
        print('RECORDING_DEFERRED: optimization and its verification must finish before recording', flush=True)
        return
    out = args.output_dir.resolve()
    out.mkdir(parents=True, exist_ok=False)
    os.environ['ROS_DOMAIN_ID'] = str(20 + zlib.crc32(str(out).encode()) % 180)
    os.environ['GZ_PARTITION'] = out.name
    # All simulation participants run on this host. Keep discovery and image/pose
    # transport on loopback so Wi-Fi reconnects do not interrupt a long run.
    os.environ['GZ_IP'] = '127.0.0.1'
    os.environ['ROS_AUTOMATIC_DISCOVERY_RANGE'] = 'LOCALHOST'
    tree = ET.parse(ROOT / 'src/zhishu_robot_gazebo/worlds/pallet_bin_15kg.sdf')
    world = tree.getroot().find('world')
    reference = world.find("model[@name='pallet_box_r0_c0']")
    original_positions = {}
    for index in range(9):
        name = f'pallet_box_r{index//3}_c{index%3}'
        old = world.find(f"model[@name='{name}']")
        pose = old.findtext('pose')
        original_positions[name] = [float(x) for x in pose.split()[:3]]
        replacement = copy.deepcopy(reference)
        replacement.set('name', name)
        replacement.find('pose').text = pose
        replacement.find('link/sensor/topic').text = (
            f'/world/pallet_bin_15kg/model/{name}/link/box_link/sensor/tote_bottom_contact/contact')
        world.remove(old)
        world.append(replacement)
    import yaml
    positions = yaml.safe_load((ROOT / 'src/zhishu_robot_moveit_config/config/pallet_bin_initial_positions.yaml').read_text())
    home = ET.parse(ROOT / 'src/zhishu_robot_moveit_config/config/zhishu_robot.srdf').getroot().find("group_state[@name='both_arms_home']")
    positions['initial_positions'].update({j.attrib['name']: float(j.attrib['value']) for j in home})
    positions['initial_positions'].update(waist_up_down_joint=0.21, waist_pitch_joint=-0.20,
                                          left_shoulder_roll_joint=-0.261799)
    initial = out / 'initial_positions.yaml'
    initial.write_text(yaml.safe_dump(positions))
    urdf = subprocess.check_output(['xacro', str(ROOT / 'src/zhishu_robot_moveit_config/config/zhishu_robot.urdf.xacro'),
        'use_gazebo:=true', 'use_fake_hardware:=false', f'initial_positions_file:={initial}',
        'use_head_camera:=false', 'mobile_base_sim_enabled:=true', 'mobile_base_sim_mass:=71.53',
        'waist_pitch_min:=-1.57079632679', 'tote_finger_length:=0.360'], text=True)
    robot_path = out / 'preloaded_robot.urdf'
    robot_path.write_text(urdf)
    robot = ET.fromstring(subprocess.check_output(['gz', 'sdf', '-p', str(robot_path)], text=True)).find('model')
    robot.set('name', 'zhishu_robot')
    pose = robot.find('pose')
    if pose is None:
        pose = ET.SubElement(robot, 'pose')
    pose.text = f'0.0 {YS[0]-0.12} 0.004 0 0 0'
    world.append(robot)
    source = out / 'sequence_source.sdf'
    tree.write(source, encoding='utf-8', xml_declaration=True)
    video_world = out / 'video_world.sdf'
    if args.no_record:
        # Preserve dynamics, controllers, collision shapes and contact sensors.
        # Camera rendering and video encoding are the only omitted work.
        for link in world.iter('link'):
            for sensor in list(link.findall('sensor')):
                if sensor.get('type') in ('camera', 'depth_camera', 'rgbd_camera'):
                    link.remove(sensor)
        tree.write(video_world, encoding='utf-8', xml_declaration=True)
    else:
        subprocess.run(['python3', str(ROOT / 'scripts/record_pallet_bin_transfer.py'), 'world',
                        '--source', str(source), '--output', str(video_world), '--fps', '30'], check=True)
    processes, handles, completed, evidence = [], [], [], []
    combined = out / 'sequence.log'
    combined.touch()

    def event(message):
        with combined.open('a') as handle:
            handle.write(message + '\n')
        print(message, flush=True)

    def start(command, name):
        handle = (out / name).open('w')
        handles.append(handle)
        process = subprocess.Popen(command, stdout=handle, stderr=subprocess.STDOUT, start_new_session=True)
        processes.append(process)
        return process

    def stop(process):
        if process.poll() is None:
            os.killpg(process.pid, signal.SIGINT)
            try:
                process.wait(timeout=30)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGTERM)
                process.wait(timeout=15)

    launch = ['ros2', 'launch', 'zhishu_robot_mtc_pick_place', 'pallet_bin_15kg_transfer.launch.py']
    common = ['headless:=true', 'use_rviz:=false', 'use_head_camera:=false', 'execute:=true',
        'waist_pitch_min:=-1.57079632679', 'tote_finger_length:=0.360',
        'keep_all_source_rows:=true', 'strict_tool_body_guard:=true', 'tool_body_clearance:=0.003',
        'velocity_scaling:=0.12', 'acceleration_scaling:=0.08', 'continuous_arm_motion:=true',
        'empty_velocity_scaling:=0.20', 'empty_acceleration_scaling:=0.12',
        'base_max_linear:=0.12', 'base_max_angular:=0.18',
        'empty_base_max_linear:=0.20', 'empty_base_max_angular:=0.30',
        'base_linear_acceleration:=0.08', 'base_angular_acceleration:=0.10',
        'base_position_tolerance:=0.002', 'base_yaw_tolerance:=0.003', 'base_motion_timeout:=600.0',
        'pre_extract_waist_pitch:=-0.785', 'hook_pose_x_offset:=-0.10', 'hook_approach_height:=0.22',
        'pre_extract_distance:=0.12', 'grasp_waist_pitch:=-1.10', 'carry_waist_lift:=0.25',
        'place_waist_lift:=0.21', 'carry_center_height:=0.94', 'place_center_height:=0.94',
        'place_on_floor_before_release:=true', 'dual_ik_attempts:=400',
        'align_with_extracted_center:=true', 'prefer_symmetric_arm_branch:=true',
        'rear_source_dock_x:=2.068', 'source_grasp_x:=0.0', 'validate_all_nine:=false',
        f'initial_positions_file:={initial}']
    scene, recorder = None, None
    node = None
    callbacks, measured = [], {}
    measured_lock = threading.Lock()
    failure = None
    try:
        scene = start(launch + common + ['start_transfer:=false', 'measurement_bridge:=false',
            'spawn_robot:=false', f'world_file:={video_world}'], 'scene.log')
        if not args.no_record:
            recorder = start(['python3', str(ROOT / 'scripts/record_pallet_bin_transfer.py'), 'record',
            '--output-dir', str(out), '--log', str(combined), '--fps', '30',
            '--sampling-time', 'simulation', '--speedup', '8', '--clean-video',
            '--finger-length-mm', '360', '--tcp-offset-mm', '300', '--floor-seated',
            '--waist-limit-deg', '-90', '--expected-transfers', '9', '--total-boxes', '9',
            '--target-model', 'pallet_box_r0_c0', '--max-seconds', '40000'], 'recorder.log')
        event('NINE_SEQUENCE_START one scene, nine physical source boxes, ' +
              ('unrecorded full-flow verification' if args.no_record else 'clean continuous recording'))
        from gz.transport13 import Node
        from gz.msgs10.pose_pb2 import Pose
        from gz.msgs10.entity_pb2 import Entity
        from gz.msgs10.boolean_pb2 import Boolean
        node = Node()
        for name in original_positions:
            def receive(message, name=name):
                p, q = message.position, message.orientation
                tilt = math.degrees(math.acos(max(-1.0, min(1.0, 1-2*(q.x*q.x+q.y*q.y)))))
                with measured_lock:
                    measured[name] = (time.monotonic(), [p.x,p.y,p.z], tilt)
            callbacks.append(receive)
            if not node.subscribe(Pose, f'/model/{name}/pose', receive):
                raise RuntimeError(f'Cannot observe {name}')
        time.sleep(60)
        for cycle, index in enumerate(ORDER):
            row, col = divmod(index, 3)
            name = f'pallet_box_r{row}_c{col}'
            next_index = ORDER[cycle+1] if cycle < 8 else None
            if next_index is None:
                return_pose = [-999.0, 0.0, 0.0]
            else:
                nr, nc = divmod(next_index, 3)
                return_pose = [2.068 if nr in (1,2) else 0.0,
                               YS[nc]-(0.10 if nr==1 else 0.12), math.pi if nr in (1,2) else 0.0]
            event('SEQUENCE_TARGET ' + name)
            options = ['reuse_gazebo:=true', 'transfer_start_delay:=2.0', f'target_model:={name}',
                f'full_transfer_box_index:={index}', f'source_row_only:={row}',
                f'completed_box_indices:={completed or [-1]}', 'completed_source_columns:=[-1]',
                f'cleared_first_row_count:={col if row in (0,2) else 0}',
                f'source_grasp_y:={return_pose[1]}', f'empty_return_pose:={return_pose}',
                f'return_to_pallet_after_transfer:={str(cycle<8).lower()}',
                f'pre_extract_enabled:={str(row in (0,2)).lower()}', f'pre_extract_box_index:={index}',
                f'rear_side_pull:={str(row==2).lower()}', f'retain_middle_row:={str(row==2).lower()}']
            if row == 1:
                options += ['side_push_execute:=true', 'rear_push_execute:=true',
                    'side_push_continue_transfer:=true', 'front_pull_execute:=false',
                    'side_push_distance:=0.324', 'side_push_overtravel:=0.020',
                    'side_push_lean:=0.90', 'side_push_waist_pitch:=-1.5707', 'side_push_waist_lift:=0.0']
            log_path = out / f'box_{index}.log'
            if args.navigation_check_only:
                options.append('navigation_check_only:=true')
            transfer = start(launch + common + options, log_path.name)
            offset, finished = 0, False
            deadline = time.monotonic() + 5000
            while time.monotonic() < deadline:
                time.sleep(1)
                with log_path.open(errors='replace') as handle:
                    handle.seek(offset)
                    chunk = handle.read()
                    offset = handle.tell()
                if chunk:
                    with combined.open('a') as handle:
                        handle.write(chunk)
                if '[FATAL]' in chunk or 'process has died' in chunk:
                    raise RuntimeError(f'{name}: motion failed; full uninterrupted recording rejected')
                if ('NAVIGATION_CHECK_SUCCESS' if args.navigation_check_only else 'Complete transfer finished') in chunk:
                    finished = True
                    break
                statuses = {'transfer':transfer.poll(), 'scene':scene.poll(), 'recorder':recorder.poll() if recorder is not None else None}
                if any(value is not None for value in statuses.values()):
                    raise RuntimeError(f'{name}: process exited {statuses}')
                with measured_lock:
                    sample = dict(measured)
                for other_index in range(9):
                    if other_index in completed or other_index == index:
                        continue
                    other = f'pallet_box_r{other_index//3}_c{other_index%3}'
                    if other not in sample or time.monotonic()-sample[other][0] > 3:
                        raise RuntimeError(f'{other}: source observation stale')
                    _, position, tilt = sample[other]
                    initial_position = original_positions[other]
                    if position[2] < 0.20 or tilt > 5 or math.dist(position[:2], initial_position[:2]) > 0.04:
                        raise RuntimeError(f'{other}: unhandled source box moved/tilted unexpectedly')
            if not finished:
                raise RuntimeError(f'{name}: transfer timeout')
            if args.navigation_check_only:
                stop(transfer)
                event('NAVIGATION_CHECK_SUCCESS no box transfer claimed')
                break
            content = log_path.read_text(errors='replace')
            for marker in ('Floor support verified BEFORE opening', 'Physical large-bin placement verified',
                           'return both arms home: execution completed', 'TOOL_BODY_GUARD'):
                if marker not in content:
                    raise RuntimeError(f'{name}: missing {marker}')
            if cycle < 8 and 'Empty robot returned to pallet; destination clearance now permitted' not in content:
                raise RuntimeError(f'{name}: empty return not verified')
            stop(transfer)
            completed.append(index)
            evidence.append({'target':name, 'index':index, 'floor_before_open':True, 'arms_home':True,
                             'empty_return':cycle<8, 'tool_body_guard':True})
            event(f'NINE_SEQUENCE_COMPLETED {len(completed)}/9 {name}')
            if cycle < 8:
                event('SEQUENCE_CLEAR_DESTINATION ' + name)
                request = Entity(name=name, type=Entity.MODEL)
                ok, response = node.request('/world/pallet_bin_15kg/remove', request, Entity, Boolean, 5000)
                if not ok or not response.data:
                    raise RuntimeError('Receiving-bin clearance failed')
                event('SEQUENCE_REMOVED ' + name)
                time.sleep(2)
        if recorder is not None:
            recorder.wait(timeout=45)
            recording = json.loads((out/'recording_summary.json').read_text())
            if recorder.returncode != 0 or not recording['passed'] or recording['recorded_completed_transfers'] != 9:
                raise RuntimeError(f'Recording not fully finalized, exit={recorder.returncode}')
        if not args.navigation_check_only:
            event('NINE_SEQUENCE_SUCCESS 9/9, no scene restart, no replay, no video splice')
    except BaseException as error:
        failure = str(error)
        event('NINE_SEQUENCE_FAILURE ' + failure)
        raise
    finally:
        (out/'sequence_result.json').write_text(json.dumps({
            'passed':failure is None and completed==ORDER, 'navigation_check_only':args.navigation_check_only,
            'navigation_passed':failure is None if args.navigation_check_only else None, 'order':ORDER, 'completed':completed,
            'rows_in_order':[0,2,1], 'single_continuous_scene':True,
            'scene_restarts':0, 'recording_restarts':0, 'resumed':False, 'stitched':False,
            'clean_video':not args.no_record, 'recording_enabled':not args.no_record, 'all_nine_initially_physical':True, 'evidence':evidence,
            'receiving_bin_clearance':'remove placed box only after verified empty return; never reset source boxes or robot',
            'failure':failure}, ensure_ascii=False, indent=2)+'\n')
        for process in reversed(processes):
            try:
                stop(process)
            except subprocess.TimeoutExpired:
                if process.poll() is None:
                    os.killpg(process.pid, signal.SIGKILL)
                    process.wait(timeout=5)
        if node is not None:
            for name in original_positions:
                node.unsubscribe(f'/model/{name}/pose')
        detached = subprocess.run(['pgrep','-f','gz sim -s.*'+re.escape(str(video_world))+r'(\s|$)'],capture_output=True,text=True)
        for value in detached.stdout.split():
            pid = int(value)
            command = subprocess.run(['ps','-p',str(pid),'-o','args='],capture_output=True,text=True).stdout.strip()
            if command.startswith('gz sim -s') and str(video_world) in command:
                os.kill(pid, signal.SIGTERM)
        for handle in handles:
            handle.close()
    if args.record_after_pass and failure is None and completed == ORDER:
        recorded_out = out.with_name(out.name + '_recorded')
        event('VERIFICATION_PASSED_START_FRESH_RECORDING ' + str(recorded_out))
        subprocess.run(['python3', str(Path(__file__).resolve()), '--output-dir', str(recorded_out)], check=True)


if __name__ == '__main__':
    main()
