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
parser.add_argument('--rear-tip', action='store_true')
parser.add_argument('--paused-init', action='store_true')
parser.add_argument('--preload-robot', action='store_true')
parser.add_argument('--startup-smoke', action='store_true')
parser.add_argument('--record', action='store_true')
parser.add_argument('--pushed-start-diagnostic', action='store_true',
                    help='Debug transfer only from an initialized pushed box; not full push-transfer evidence')
parser.add_argument('--push-lean', type=float, default=0.90)
parser.add_argument('--push-overtravel', type=float, default=0.0)
parser.add_argument('--push-waist-pitch', type=float, default=-1.5707)
parser.add_argument('--physics-engine', choices=('dartsim', 'bullet-featherstone'), default='dartsim')
parser.add_argument('--col', type=int, choices=(0, 1, 2), default=2)
parser.add_argument('--completed-source-columns', type=int, nargs='*', choices=(0, 1, 2), default=[])
args = parser.parse_args()
out = root / (('output/pallet_middle_side_push_20261009_lattice' if args.side else 'output/pallet_middle_rear_push_20261009_lattice') + args.suffix)
out.mkdir(exist_ok=False)
if args.rear_tip:
    import zlib
    os.environ['ROS_DOMAIN_ID'] = str(20 + zlib.crc32(out.name.encode()) % 180)
    os.environ['GZ_PARTITION'] = out.name
ys = [-0.400617, 0.002017, 0.406220]
front_mode = args.front_pull_probe or args.front_pull
target_col = args.col
target_y = ys[target_col]
world_file = root / 'output/pallet_middle_three_20261009_attempt3/video_world.sdf'
if not front_mode:
    diagnostic = ET.parse(world_file)
    diagnostic_world = diagnostic.getroot().find('world')
    for completed_col in args.completed_source_columns:
        if completed_col == target_col:
            raise ValueError('Cannot remove the current target')
        diagnostic_world.remove(diagnostic_world.find(f"model[@name='pallet_box_r1_c{completed_col}']"))
    diagnostic_world.find("plugin[@name='gz::sim::systems::Physics']/engine/filename").text = 'gz-physics-' + args.physics_engine + '-plugin'
    if args.pushed_start_diagnostic:
        diagnostic_world.find(f"model[@name='pallet_box_r1_c{target_col}']/pose").text = f'0.701848 {target_y} 0.224 0 0 0'
    for element in list(diagnostic_world):
        if element.get('name', '').startswith('video_') or element.get('name', '') == 'gz::sim::systems::Sensors':
            diagnostic_world.remove(element)
    world_file = out / 'physics_world.sdf'
    diagnostic.write(world_file, encoding='utf-8', xml_declaration=True)
    if args.record:
        # Observer sensors do not change physical geometry or robot control.
        sensors = ET.SubElement(diagnostic_world, 'plugin',
            filename='gz-sim-sensors-system', name='gz::sim::systems::Sensors')
        ET.SubElement(sensors, 'render_engine').text = 'ogre2'
        diagnostic.write(world_file, encoding='utf-8', xml_declaration=True)
        recorded_world = out / 'video_world.sdf'
        subprocess.run(['python3', str(root / 'scripts/record_pallet_bin_transfer.py'),
                        'world', '--source', str(world_file), '--output', str(recorded_world)], check=True)
        world_file = recorded_world
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
          'use_head_camera:=false',
          'tote_finger_length:=0.360', 'waist_pitch_min:=-1.57079632679',
          f'target_model:=pallet_box_r1_c{target_col}',
          f'full_transfer_box_index:={3+target_col}']
if args.completed_source_columns:
    common += [f'completed_source_columns:={args.completed_source_columns}']
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
if args.rear_tip and not args.pushed_start_diagnostic:
    common += ['rear_push_execute:=true', f'side_push_lean:={args.push_lean}',
               f'side_push_overtravel:={args.push_overtravel}',
               'managed_gazebo_shutdown:=true',
               f'side_push_waist_pitch:={args.push_waist_pitch}', 'side_push_waist_lift:=0.0']
if args.rear_tip:
    import yaml
    positions = yaml.safe_load((root / 'src/zhishu_robot_moveit_config/config/pallet_bin_initial_positions.yaml').read_text())
    home = ET.parse(root / 'src/zhishu_robot_moveit_config/config/zhishu_robot.srdf').getroot().find("group_state[@name='both_arms_home']")
    positions['initial_positions'].update({j.attrib['name']: float(j.attrib['value']) for j in home})
    positions['initial_positions'].update(waist_up_down_joint=0.21, waist_pitch_joint=-0.20)
    # Existing outward hand-eye park keeps the idle elbow clear of the camera.
    positions['initial_positions']['left_shoulder_roll_joint'] = -0.261799
    (out / 'safe_park_initial.yaml').write_text(yaml.safe_dump(positions))
    common += [f'initial_positions_file:={out}/safe_park_initial.yaml']
if args.preload_robot:
    if not args.rear_tip:
        raise RuntimeError('Preloaded model is configured only for the rear-tip test')
    urdf = subprocess.check_output([
        'xacro', str(root / 'src/zhishu_robot_moveit_config/config/zhishu_robot.urdf.xacro'),
        'use_gazebo:=true', 'use_fake_hardware:=false',
        f'initial_positions_file:={out}/safe_park_initial.yaml',
        'use_head_camera:=false', 'mobile_base_sim_enabled:=true',
        'mobile_base_sim_mass:=71.53', 'waist_pitch_min:=-1.57079632679',
        'tote_finger_length:=0.360'], text=True)
    ET.ElementTree(ET.fromstring(urdf)).write(out / 'preloaded_robot.urdf', encoding='utf-8', xml_declaration=True)
    converted = subprocess.check_output(['gz', 'sdf', '-p', str(out / 'preloaded_robot.urdf')], text=True)
    robot_model = ET.fromstring(converted).find('model')
    robot_model.set('name', 'zhishu_robot')
    pose = robot_model.find('pose')
    if pose is None:
        pose = ET.SubElement(robot_model, 'pose')
    pose.text = (f'0.0 {target_y} 0.004 0 0 0' if args.pushed_start_diagnostic else
                 f'2.068 {target_y - 0.10} 0.004 0 0 3.14159265359')
    preloaded = ET.parse(world_file)
    preloaded.getroot().find('world').append(robot_model)
    world_file = out / 'preloaded_world.sdf'
    preloaded.write(world_file, encoding='utf-8', xml_declaration=True)
processes = []
handles = []
monitor = None
telemetry = None
recorder = None
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
                   f'spawn_robot:={str(not args.preload_robot).lower()}',
                   *([f'paused:={str(args.paused_init).lower()}',
                      f'managed_gazebo_shutdown:={str(os.environ.get("PALLET_GZ_TRACE") == "1").lower()}'] if args.rear_tip else []),
                   'spawn_x:=0.0' if front_mode else (f'spawn_x:={args.side_x}' if args.side else 'spawn_x:=2.068'),
                   f'spawn_y:={target_y+0.10}' if front_mode else ('spawn_y:=1.034' if args.side else f'spawn_y:={target_y-0.10}'),
                   'spawn_yaw:=0.0' if front_mode else ('spawn_yaw:=-1.57079632679' if args.side else 'spawn_yaw:=3.14159265359'),
                   'world_file:=' + str(world_file)], 'scene.log')
    ready_deadline = time.monotonic() + 120
    unpaused = not (args.rear_tip and args.paused_init)
    while time.monotonic() < ready_deadline:
        if scene.poll() is not None:
            raise RuntimeError('Scene failed to start')
        scene_log = (out / 'scene.log').read_text(errors='replace')
        if re.search(r'\[ERROR\].*\[(?:gazebo|managed_gazebo)-\d+\]: process has died', scene_log):
            raise RuntimeError('Gazebo exited during initialization')
        if 'Could not contact service /controller_manager/list_controllers' in scene_log:
            raise RuntimeError('Controller manager initialization failed')
        if not unpaused and 'Entity creation successful' in scene_log:
            from gz.transport13 import Node
            from gz.msgs10.world_control_pb2 import WorldControl
            from gz.msgs10.boolean_pb2 import Boolean
            # Let the spawned model initialize its control plugin before free
            # dynamics; otherwise unconstrained initial joints can collapse.
            plugin_ready = 'gz_ros2_control' in scene_log
            request = WorldControl(pause=not plugin_ready, multi_step=0 if plugin_ready else 10)
            ok, response = Node().request('/world/pallet_bin_15kg/control', request, WorldControl, Boolean, 3000)
            if not ok or not response.data:
                raise RuntimeError('Cannot unpause initialized scene')
            unpaused = plugin_ready
        if 'Configured and activated right_gripper_controller' in scene_log:
            break
        time.sleep(1)
    else:
        raise RuntimeError('Scene controllers failed to become ready')
    if args.startup_smoke:
        import rclpy
        from rclpy.action import ActionClient
        from control_msgs.action import FollowJointTrajectory
        from trajectory_msgs.msg import JointTrajectoryPoint
        from sensor_msgs.msg import JointState
        rclpy.init()
        monitor = rclpy.create_node('gz_control_startup_smoke')
        joints = {}
        def observe_joints(msg):
            joints.update(zip(msg.name, msg.position))
        joint_sub = monitor.create_subscription(JointState, '/joint_states', observe_joints, 10)
        client = ActionClient(monitor, FollowJointTrajectory, '/right_gripper_controller/follow_joint_trajectory')
        if not client.wait_for_server(timeout_sec=10):
            raise RuntimeError('Gripper action unavailable after startup')
        samples = []
        for target in (0.02, 0.0, 0.04, 0.0):
            command = FollowJointTrajectory.Goal()
            command.trajectory.joint_names = ['gripper_left_finger_joint']
            point = JointTrajectoryPoint(positions=[target])
            point.time_from_start.sec = 2
            command.trajectory.points = [point]
            future = client.send_goal_async(command)
            rclpy.spin_until_future_complete(monitor, future, timeout_sec=10)
            if not future.done() or not future.result().accepted:
                raise RuntimeError('Smoke gripper command rejected or timed out')
            result = future.result().get_result_async()
            rclpy.spin_until_future_complete(monitor, result, timeout_sec=15)
            if not result.done() or result.result().result.error_code != 0:
                raise RuntimeError('Smoke gripper motion did not complete')
            settle = time.monotonic() + 2
            while time.monotonic() < settle:
                rclpy.spin_once(monitor, timeout_sec=0.1)
            master = joints.get('gripper_left_finger_joint')
            follower = joints.get('gripper_right_finger_joint')
            if master is None or follower is None or abs(master-target) > 0.005 or abs(follower+master) > 0.005:
                raise RuntimeError(f'Mimic tracking failed: target={target} master={master} follower={follower}')
            samples.append(dict(target=target, master=master, follower=follower))
        (out / 'startup_smoke_result.json').write_text(json.dumps(dict(
            passed=True, scope='startup and real gripper/mimic motion only',
            full_transfer_completed=False, physics_engine=args.physics_engine,
            samples=samples), indent=2))
        print('STARTUP_AND_MIMIC_SMOKE_PASSED', flush=True)
        raise SystemExit(0)
    if args.record:
        recording_log = (out / 'recording.log').open('w')
        handles.append(recording_log)
        recorder = subprocess.Popen(['python3', str(root / 'scripts/record_pallet_bin_transfer.py'),
            'record', '--output-dir', str(out / 'video'), '--log', str(out / 'probe.log'),
            '--finger-length-mm', '360', '--tcp-offset-mm', '300', '--waist-limit-deg', '-90',
            '--floor-seated', '--speedup', '8', '--max-seconds', '4600',
            '--target-model', f'pallet_box_r1_c{target_col}',
            *(['--rear-push'] if args.rear_tip else [])],
            stdout=recording_log, stderr=subprocess.STDOUT, start_new_session=True)
        recording_deadline = time.monotonic() + 90
        while time.monotonic() < recording_deadline:
            if recorder.poll() is not None:
                raise RuntimeError('Video recorder exited before receiving frames')
            if 'both video encoders ready' in (out / 'recording.log').read_text(errors='replace'):
                break
            time.sleep(1)
        else:
            raise RuntimeError('No rendered video frames; refusing an unrecorded test')
    probe = start(['reuse_gazebo:=true', f'execute:={str(args.physical).lower()}', 'pre_extract_enabled:=false',
                   'side_push_execute:=false' if args.front_pull or args.pushed_start_diagnostic else ('side_push_execute:=true' if args.physical else 'rear_push_probe:=true'),
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
    # Rendering adds wall time, without changing simulation trajectory speeds.
    deadline = time.monotonic() + ((4500 if args.record else 3000) if args.transfer else (800 if args.physical else 400))
    while time.monotonic() < deadline:
        log = (out / 'probe.log').read_text(errors='replace')
        if monitor:
            rclpy.spin_once(monitor, timeout_sec=0.2)
            if latest[0]:
                pose = latest[0].pose
                p, q = pose.position, pose.orientation
                telemetry.write(f'{time.time()},{p.x},{p.y},{p.z},{q.x},{q.y},{q.z},{q.w}\n')
                telemetry.flush()
                if not args.pushed_start_diagnostic and 'Physical side push verified' not in log and (p.x > 1.058284 or p.x < 0.654284 or abs(p.y - target_y) > 0.06 or abs(p.z - 0.224) > 0.025):
                    pause_physics()
                    raise RuntimeError(f'Physical guard: off-axis/height deviation x={p.x:.5f} y={p.y:.5f} z={p.z:.5f}')
                if 'Physical side push verified' in log and 'Starting complete pallet -> large-bin transfer' not in log:
                    if abs(p.x - 0.704284) > 0.03 or abs(p.y - target_y) > 0.04 or abs(p.z - 0.224) > 0.02:
                        pause_physics()
                        raise RuntimeError(f'Post-push home/route guard x={p.x:.5f} y={p.y:.5f} z={p.z:.5f}')
        else:
            time.sleep(2)
        log = (out / 'probe.log').read_text(errors='replace')
        if 'process has died' in log and 'pallet_bin_15kg_transfer-2' in log and '[FATAL]' not in log:
            raise RuntimeError('Transfer process exited abnormally')
        if '[FATAL]' in log or 'Rear push IK probe passed' in log or 'Complete transfer finished' in log or ('Physical side push verified' in log and not args.transfer):
            print(log[-6000:], flush=True)
            if '[FATAL]' in log:
                raise RuntimeError('Node reported fatal validation failure')
            (out / 'validation_result.json').write_text(json.dumps({
                'passed': True, 'target_model': f'pallet_box_r1_c{target_col}',
                'full_transfer_completed': 'Complete transfer finished' in log,
                'planning_probe_only': not args.physical,
                'skipped_verified_columns': args.completed_source_columns or ([2, 1] if target_col == 0 and front_mode else ([2] if front_mode else [])),
                'single_scene': True, 'front_pull': args.front_pull,
                'pushed_start_diagnostic': args.pushed_start_diagnostic,
                'finger_mm': 360, 'tcp_offset_mm': 300, 'waist_limit_deg': -90,
            }, ensure_ascii=False, indent=2))
            break
        if probe.poll() is not None or scene.poll() is not None:
            raise RuntimeError('Process exited before verified completion')
    else:
        raise RuntimeError('Probe timeout')
finally:
    if recorder is not None and recorder.poll() is None:
        # Keep the outcome visible and finalize the video even on validation failure.
        try:
            recorder.wait(timeout=10)
        except subprocess.TimeoutExpired:
            os.killpg(recorder.pid, signal.SIGINT)
            recorder.wait(timeout=20)
    if monitor:
        monitor.destroy_node()
        if rclpy.ok():
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
                try:
                    proc.wait(timeout=15)
                except subprocess.TimeoutExpired:
                    os.killpg(proc.pid, signal.SIGKILL)
                    proc.wait(timeout=5)
    for handle in handles:
        handle.close()
    if front_mode or args.rear_tip:
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
