#!/usr/bin/env python3
"""Run the nine-tote scene, transferring only its first tote with wrist feedback.

Creates a fresh, reproducible scene and keeps Gazebo open after completion.
The existing gz_ros2_control compatibility overlay must already be built.
"""
import argparse
import json
import os
from pathlib import Path
import signal
import subprocess
import time
import xml.etree.ElementTree as ET
import yaml

ROOT = Path(__file__).resolve().parents[1]
HELPER = ROOT / 'scripts/run_with_gz_control_safe_overlay.sh'


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output-dir', type=Path, required=True)
    parser.add_argument('--domain-id', type=int, default=182)
    parser.add_argument('--no-gui', action='store_true', help='Explicitly use headless mode (the default)')
    parser.add_argument('--clearance-only', action='store_true', help='Check protected docking and torso poses with an empty robot; first tote is already seated')
    parser.add_argument('--gui', action='store_true', help='Open the Gazebo GUI for this run')
    parser.add_argument('--keep-open', action='store_true', help='Keep the verified scene open until Ctrl+C')
    parser.add_argument('--record-video', action='store_true', help='Record only this first-box transfer from the actual Gazebo window')
    args = parser.parse_args()
    if args.gui and args.no_gui:
        parser.error('--gui and --no-gui cannot be combined')
    if not 0 <= args.domain_id <= 232:
        parser.error('--domain-id must be in 0..232')
    if not (ROOT / 'output/gz_control_safe_overlay/setup.bash').exists():
        parser.error('Build the existing compatibility overlay with scripts/build_gz_control_safe_overlay.sh first')
    out = args.output_dir.resolve()
    out.mkdir(parents=True, exist_ok=False)
    env = dict(os.environ, ROS_DOMAIN_ID=str(args.domain_id),
               GZ_PARTITION='first_box_' + out.name, ROS_HOME=str(out / 'ros_home'),
               ROS_LOG_DIR=str(out / 'roslog'), QT_QPA_PLATFORM='xcb', LIBGL_ALWAYS_SOFTWARE='1')
    for name in ('ros_home', 'roslog'):
        (out / name).mkdir()
    config = ROOT / 'src/zhishu_robot_moveit_config/config'
    initial = yaml.safe_load((config / 'pallet_bin_initial_positions.yaml').read_text())
    srdf = ET.parse(config / 'zhishu_robot.srdf').getroot()
    home = srdf.find("group_state[@name='both_arms_home']")
    initial['initial_positions'].update({j.get('name'): float(j.get('value')) for j in home})
    initial['initial_positions'].update(waist_up_down_joint=.21, waist_pitch_joint=-.20,
                                         left_shoulder_roll_joint=-.261799)
    initial_path = out / 'initial_positions.yaml'
    initial_path.write_text(yaml.safe_dump(initial))

    def capture(command):
        return subprocess.check_output(['bash', str(HELPER), *command], env=env, cwd=ROOT, text=True)

    urdf = capture(['xacro', str(config / 'zhishu_robot.urdf.xacro'), 'use_gazebo:=true',
                    'use_fake_hardware:=false', f'initial_positions_file:={initial_path}',
                    'use_head_camera:=false', 'mobile_base_sim_enabled:=true',
                    'mobile_base_sim_mass:=71.53', 'waist_pitch_min:=-1.57079632679',
                    'tote_finger_length:=0.360'])
    urdf_path = out / 'robot.urdf'
    urdf_path.write_text(urdf)
    tree = ET.parse(ROOT / 'src/zhishu_robot_gazebo/worlds/pallet_bin_15kg.sdf')
    world = tree.getroot().find('world')
    robot = ET.fromstring(capture(['gz', 'sdf', '-p', str(urdf_path)])).find('model')
    robot.set('name', 'zhishu_robot')
    pose = robot.find('pose')
    if pose is None:
        pose = ET.SubElement(robot, 'pose')
    pose.text = '0 -1.55 0.004 0 0 0' if args.clearance_only else '0 -0.520617 0.004 0 0 0'
    if args.clearance_only:
        world.find("model[@name='pallet_box_r0_c0']/pose").text = '1.03 -1.55 0.636 0 0 0'
    world.append(robot)
    world_path = out / 'world.sdf'
    tree.write(world_path, encoding='utf-8', xml_declaration=True)
    assert len([m for m in world.findall('model') if m.get('name', '').startswith('pallet_box_')]) == 9
    assert not any(s.get('type') == 'contact' for m in world.findall('model')
                   if m.get('name', '').startswith('pallet_box_') for s in m.findall('.//sensor'))
    manifest = {'workspace': str(ROOT), 'domain_id': args.domain_id, 'partition': env['GZ_PARTITION'],
                'target': 'pallet_box_r0_c0', 'finger_length_m': .36,
                'flow': 'side pull 120 mm -> dual-arm grasp -> lift -> carry -> wrist-admittance seating -> release',
                'bin_center_xy_m': [1.03, -1.55], 'world': str(world_path),
                'initial_box_xyz_m': {m.get('name'): [float(v) for v in m.findtext('pose').split()[:3]]
                                      for m in world.findall('model') if m.get('name', '').startswith('pallet_box_')}}
    (out / 'manifest.json').write_text(json.dumps(manifest, indent=2))
    processes, handles = [], []

    def start(command, log):
        handle = (out / log).open('w')
        handles.append(handle)
        p = subprocess.Popen(['bash', str(HELPER), *command], cwd=ROOT, env=env,
                             stdout=handle, stderr=subprocess.STDOUT, start_new_session=True)
        processes.append(p)
        return p

    launch = ['ros2', 'launch', 'zhishu_robot_mtc_pick_place', 'pallet_bin_15kg_transfer.launch.py',
              'headless:=true', 'use_rviz:=false', 'use_head_camera:=false', 'spawn_robot:=false',
              f'world_file:={world_path}', f'initial_positions_file:={initial_path}',
              'tote_finger_length:=0.360', 'strict_tool_body_guard:=true', 'tool_body_clearance:=0.003',
              'keep_all_source_rows:=true', 'validate_all_nine:=false', 'full_transfer_box_index:=0',
              'pre_extract_enabled:=true', 'pre_extract_box_index:=0', 'source_row_only:=0',
              'pre_extract_distance:=0.12', 'pre_extract_waist_pitch:=-0.785',
              'hook_pose_x_offset:=-0.10', 'hook_approach_height:=0.22',
              'align_with_extracted_center:=true', 'source_grasp_y:=-0.520617',
              'prefer_symmetric_arm_branch:=true',
              'dual_ik_attempts:=400', 'dual_ik_timeout:=0.05', 'grasp_waist_pitch:=-1.10',
              'carry_waist_lift:=0.25', 'carry_center_height:=0.94', 'place_waist_lift:=0.28',
              'place_center_height:=0.956', 'place_carry_reach:=0.65', 'place_waist_pitch:=-0.70', 'place_floor_waist_pitch:=-0.80', 'place_on_floor_before_release:=true',
              'bin_target_x:=1.03', 'bin_dock_x:=0.345', 'column_table_clearance:=0.200',
              'return_to_pallet_after_transfer:=false',
              'base_motion_timeout:=600.0', 'base_max_linear:=0.12', 'base_max_angular:=0.18',
              'base_linear_acceleration:=0.08', 'base_angular_acceleration:=0.10']
    if args.clearance_only:
        launch += ['column_clearance_check_only:=true']
        manifest['flow'] = 'empty robot: protected bin dock -> carry torso -> place torso; first tote pre-seated for this distance-only test'
    manifest['headless'] = not args.gui
    manifest['lift_stage_clearance_reference'] = 'waist_up_down_link visible outer envelope; collision box is checked independently'
    manifest['required_lift_stage_horizontal_gap_m'] = .200
    manifest['launch_arguments'] = launch
    if not args.clearance_only:
        from audit_pallet_placement_reach import audit
        reach = audit(urdf_path, dock_x=.345, lift=.28, pitch=-.80)
        (out / 'placement_reach.json').write_text(json.dumps(reach, indent=2))
        if not reach['existing_grip_can_reach']:
            raise RuntimeError('Existing straight-down grip cannot reach the bin centre at the protected 200 mm dock. No ROS/Gazebo process was started; revise grip/posture before the next transfer.')
    (out / 'manifest.json').write_text(json.dumps(manifest, indent=2))
    failed = False
    try:
        scene = start(launch, 'scene.log')
        video = None
        if args.gui:
            start(['gz', 'sim', '-g', '--render-engine-gui', 'ogre'], 'gui.log')
        if args.record_video:
            if not args.gui:
                raise RuntimeError('--record-video requires --gui')
            video = start(['python3', str(ROOT / 'scripts/record_first_box_window.py'),
                           '--output-dir', str(out / 'video'), '--log', str(out / 'scene.log')],
                          'video_recorder.log')
        start(['python3', str(ROOT / 'scripts/record_pallet_wrist_forces.py'),
               '--scene-log', str(out / 'scene.log'), '--output-dir', str(out)], 'recorder.log')
        start(['python3', str(ROOT / 'scripts/monitor_pallet_table_clearance.py'),
               '--urdf', str(urdf_path), '--scene-log', str(out / 'scene.log'),
               '--output-dir', str(out)], 'table_monitor.log')
        print('Gazebo first-box test started. Logs:', out, flush=True)
        finished = False
        while scene.poll() is None:
            content = (out / 'scene.log').read_text(errors='replace')
            if not finished and 'Complete transfer finished' in content:
                finished = True
                print('First-box placement completed.', flush=True)
                if video is not None:
                    video.wait(timeout=120)
                if args.keep_open:
                    while scene.poll() is None:
                        time.sleep(1)
                break
            if args.clearance_only and 'COLUMN_CLEARANCE_CHECK_SUCCESS' in content:
                print('Column clearance check completed; Gazebo remains open.', flush=True)
                if args.keep_open:
                    while scene.poll() is None:
                        time.sleep(1)
                break
            if '[FATAL]' in content:
                failed = True
                print('Transfer stopped by protection. Inspect scene.log.', flush=True)
                if args.keep_open:
                    while scene.poll() is None:
                        time.sleep(1)
                break
            time.sleep(1)
    except KeyboardInterrupt:
        pass
    finally:
        for p in reversed(processes):
            if p.poll() is None:
                os.killpg(p.pid, signal.SIGINT)
        for p in reversed(processes):
            try:
                p.wait(timeout=10)
            except subprocess.TimeoutExpired:
                os.killpg(p.pid, signal.SIGTERM)
        for h in handles:
            h.close()
    if failed:
        raise RuntimeError('Simulation stopped by protection; see scene.log')


if __name__ == '__main__':
    main()
