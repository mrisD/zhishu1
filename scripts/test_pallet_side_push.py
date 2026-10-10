import os
import signal
import subprocess
import time
import argparse
import json
import re
import xml.etree.ElementTree as ET
from pathlib import Path

root = Path('/home/zhul/online/guohua_ws')
parser = argparse.ArgumentParser()
parser.add_argument('--side', action='store_true')
parser.add_argument('--side-x', type=float, default=1.028284)
parser.add_argument('--suffix', default='')
parser.add_argument('--physical', action='store_true')
parser.add_argument('--transfer', action='store_true')
parser.add_argument('--front-pull-probe', action='store_true')
parser.add_argument('--front-pull', action='store_true')
parser.add_argument('--col', type=int, choices=(0, 1, 2), default=2)
args = parser.parse_args()
out = root / (('output/pallet_middle_side_push_20261009_lattice' if args.side else 'output/pallet_middle_rear_push_20261009_lattice') + args.suffix)
out.mkdir(exist_ok=False)
ys = [-0.400617, 0.002017, 0.406220]
front_mode = args.front_pull_probe or args.front_pull
target_col = args.col if front_mode else (2 if args.side else 0)
target_y = ys[target_col]
world_file = root / 'output/pallet_middle_three_20261009_attempt3/video_world.sdf'
if front_mode:
    evidence = (root / 'output/pallet_middle_side_push_20261009_lattice_transfer2_refresh/probe.log').read_text(errors='replace')
    if 'Complete transfer finished' not in evidence:
        raise RuntimeError('Cannot skip c2 without verified completion evidence')
    world = ET.parse(world_file)
    scene_world = world.getroot().find('world')
    scene_world.remove(scene_world.find("model[@name='pallet_box_r1_c2']"))
    if target_col == 0:
        verified_c1 = []
        for candidate in (root / 'output').glob('pallet_middle_rear_push_20261009_lattice_c1_front_transfer*/probe.log'):
            content = candidate.read_text(errors='replace')
            if ('Complete transfer finished' in content and '[FATAL]' not in content and
                    'Starting complete pallet -> large-bin transfer with pallet_box_r1_c1' in content):
                verified_c1.append(str(candidate))
        if not verified_c1:
            raise RuntimeError('Cannot skip c1 without verified completion evidence')
        (out / 'skipped_c1_evidence.txt').write_text('\n'.join(verified_c1) + '\n')
        scene_world.remove(scene_world.find("model[@name='pallet_box_r1_c1']"))
    world_file = out / 'remaining_middle.sdf'
    world.write(world_file, encoding='utf-8', xml_declaration=True)
    import yaml
    positions = yaml.safe_load((root / 'src/zhishu_robot_moveit_config/config/pallet_bin_initial_positions.yaml').read_text())
    home = ET.parse(root / 'src/zhishu_robot_moveit_config/config/zhishu_robot.srdf').getroot().find("group_state[@name='both_arms_home']")
    for joint in home:
        positions['initial_positions'][joint.attrib['name']] = float(joint.attrib['value'])
    positions['initial_positions'].update(waist_up_down_joint=0.21, waist_pitch_joint=-0.20)
    (out / 'safe_park_initial.yaml').write_text(yaml.safe_dump(positions))
launch = ['ros2', 'launch', 'zhishu_robot_mtc_pick_place', 'pallet_bin_15kg_transfer.launch.py']
common = ['headless:=true', 'use_rviz:=false', 'source_row_only:=1',
          'tote_finger_length:=0.360', 'waist_pitch_min:=-1.57079632679',
          f'target_model:=pallet_box_r1_c{target_col}',
          f'full_transfer_box_index:={3+target_col}']
if args.front_pull_probe:
    common += ['front_pull_probe:=true', 'completed_source_columns:=[2]']
elif args.front_pull:
    common += ['front_pull_execute:=true', 'completed_source_columns:=[2,1]' if target_col == 0 else 'completed_source_columns:=[2]']
if front_mode:
    common += [f'initial_positions_file:={out}/safe_park_initial.yaml']
if args.transfer:
    common += ['side_push_continue_transfer:=true', 'side_push_distance:=0.324',
        'carry_waist_lift:=0.25', 'place_waist_lift:=0.21',
        'carry_center_height:=0.94', 'place_center_height:=0.94',
        'grasp_waist_pitch:=-1.10', 'dual_ik_attempts:=400',
        'base_max_linear:=0.08', 'base_max_angular:=0.12',
        'base_linear_acceleration:=0.06', 'base_angular_acceleration:=0.06',
        'base_motion_timeout:=600.0', 'place_on_floor_before_release:=true',
        'velocity_scaling:=0.12', 'acceleration_scaling:=0.08',
        'source_grasp_x:=0.0', 'align_with_extracted_center:=true',
        'prefer_symmetric_arm_branch:=true', 'validate_all_nine:=false']
processes = []
handles = []
monitor = None
telemetry = None
def pause_physics():
    from gz.transport13 import Node
    from gz.msgs10.world_control_pb2 import WorldControl
    from gz.msgs10.boolean_pb2 import Boolean
    node = Node()
    request = WorldControl(pause=True)
    ok, response = node.request('/world/pallet_bin_15kg/control', request, WorldControl, Boolean, 1000)
    print(f'EMERGENCY_PAUSE confirmed={bool(ok and response.data)}', flush=True)
def start(args, name):
    handle = (out / name).open('w')
    handles.append(handle)
    proc = subprocess.Popen(launch + common + args, stdout=handle,
                            stderr=subprocess.STDOUT, start_new_session=True)
    processes.append(proc)
    return proc
try:
    scene = start(['start_transfer:=false', 'measurement_bridge:=false',
                   'spawn_x:=0.0' if front_mode else (f'spawn_x:={args.side_x}' if args.side else 'spawn_x:=2.068'),
                   f'spawn_y:={target_y+0.10}' if front_mode else ('spawn_y:=1.034' if args.side else 'spawn_y:=-0.500617'),
                   'spawn_yaw:=0.0' if front_mode else ('spawn_yaw:=-1.57079632679' if args.side else 'spawn_yaw:=3.14159265359'),
                   'world_file:=' + str(world_file)], 'scene.log')
    time.sleep(60)
    if scene.poll() is not None:
        raise RuntimeError('Scene failed to start')
    probe = start(['reuse_gazebo:=true', f'execute:={str(args.physical).lower()}', 'pre_extract_enabled:=false',
                   'side_push_execute:=false' if args.front_pull else ('side_push_execute:=true' if args.physical else 'rear_push_probe:=true'),
                   'transfer_start_delay:=5.0'], 'probe.log')
    if args.physical:
        import rclpy
        from geometry_msgs.msg import PoseStamped
        rclpy.init()
        monitor = rclpy.create_node('side_push_physical_guard')
        telemetry = (out / 'box_measurements.csv').open('w')
        telemetry.write('wall_time,x,y,z,qx,qy,qz,qw\n')
        latest = [None]
        def measured(msg):
            latest[0] = msg
        subscription = monitor.create_subscription(PoseStamped, '/pallet_bin/measured_box_pose', measured, 10)
    deadline = time.monotonic() + (3000 if args.transfer else (800 if args.physical else 400))
    while time.monotonic() < deadline:
        log = (out / 'probe.log').read_text(errors='replace')
        if monitor:
            rclpy.spin_once(monitor, timeout_sec=0.2)
            if latest[0]:
                pose = latest[0].pose
                p, q = pose.position, pose.orientation
                telemetry.write(f'{time.time()},{p.x},{p.y},{p.z},{q.x},{q.y},{q.z},{q.w}\n')
                telemetry.flush()
                if 'Physical side push verified' not in log and (p.x > 1.058284 or p.x < 0.654284 or abs(p.y - target_y) > 0.06 or abs(p.z - 0.224) > 0.025):
                    pause_physics()
                    raise RuntimeError(f'Physical guard: off-axis/height deviation x={p.x:.5f} y={p.y:.5f} z={p.z:.5f}')
                if 'Physical side push verified' in log and 'Starting complete pallet -> large-bin transfer' not in log:
                    if abs(p.x - 0.704284) > 0.03 or abs(p.y - target_y) > 0.04 or abs(p.z - 0.224) > 0.02:
                        pause_physics()
                        raise RuntimeError(f'Post-push home/route guard x={p.x:.5f} y={p.y:.5f} z={p.z:.5f}')
        else:
            time.sleep(2)
        log = (out / 'probe.log').read_text(errors='replace')
        if '[FATAL]' in log or 'Rear push IK probe passed' in log or 'Complete transfer finished' in log or ('Physical side push verified' in log and not args.transfer):
            print(log[-6000:], flush=True)
            if '[FATAL]' in log:
                raise RuntimeError('Node reported fatal validation failure')
            (out / 'validation_result.json').write_text(json.dumps({
                'passed': True, 'target_model': f'pallet_box_r1_c{target_col}',
                'full_transfer_completed': 'Complete transfer finished' in log,
                'planning_probe_only': not args.physical,
                'skipped_verified_columns': [2, 1] if target_col == 0 and front_mode else ([2] if front_mode else []),
                'single_scene': True, 'front_pull': args.front_pull,
                'finger_mm': 360, 'tcp_offset_mm': 300, 'waist_limit_deg': -90,
            }, ensure_ascii=False, indent=2))
            break
        if probe.poll() is not None or scene.poll() is not None:
            raise RuntimeError('Process exited before verified completion')
    else:
        raise RuntimeError('Probe timeout')
finally:
    if monitor:
        monitor.destroy_node()
        rclpy.shutdown()
    if telemetry:
        telemetry.close()
    for proc in reversed(processes):
        if proc.poll() is None:
            os.killpg(proc.pid, signal.SIGINT)
            try:
                proc.wait(timeout=25)
            except subprocess.TimeoutExpired:
                os.killpg(proc.pid, signal.SIGTERM)
                proc.wait(timeout=15)
    for handle in handles:
        handle.close()
    if front_mode:
        # Gazebo can outlive ros2 launch after its wrapper exits. Only target
        # this run's uniquely generated world, never other simulation worlds.
        found = subprocess.run(['pgrep', '-f', r'gz sim -s.*' + re.escape(str(world_file)) + r'(\s|$)'],
                               capture_output=True, text=True)
        for value in found.stdout.split():
            pid = int(value)
            command = subprocess.run(['ps', '-p', str(pid), '-o', 'args='], capture_output=True, text=True).stdout
            if command.startswith('gz sim -s') and str(world_file) in command:
                try:
                    os.kill(pid, signal.SIGTERM)
                    time.sleep(1)
                    if subprocess.run(['ps', '-p', str(pid), '-o', 'args='], capture_output=True, text=True).stdout.startswith('gz sim -s'):
                        os.kill(pid, signal.SIGKILL)
                except ProcessLookupError:
                    pass
