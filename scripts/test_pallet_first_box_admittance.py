#!/usr/bin/env python3
"""Wait for a verified MTC handoff, then test only pallet_box_r0_c0.

Run with the same ROS_DOMAIN_ID and GZ_PARTITION as the live nine-box scene.
The scene must have been started with admittance_handoff:=true.
"""
import argparse
from pathlib import Path
import subprocess
import time


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--scene-log', type=Path, required=True)
    parser.add_argument('--output-dir', type=Path, required=True)
    args = parser.parse_args()
    out = args.output_dir.resolve()
    out.mkdir(parents=True, exist_ok=True)
    deadline = time.monotonic() + 1200
    while time.monotonic() < deadline:
        content = args.scene_log.read_text(errors='replace')
        if '[FATAL]' in content:
            raise RuntimeError('MTC failed; admittance handoff rejected')
        if 'ADMITTANCE_HANDOFF_READY pallet_box_r0_c0' in content:
            break
        time.sleep(1)
    else:
        raise RuntimeError('No verified first-box handoff within 1200 seconds')

    def run(*command):
        print('RUN', ' '.join(command), flush=True)
        subprocess.run(command, check=True, timeout=120)

    import ament_index_python.packages
    share = ament_index_python.packages.get_package_share_directory('zhishu_robot_moveit_config')
    run('ros2', 'run', 'controller_manager', 'spawner', 'left_arm_controller',
        'right_arm_controller', '--inactive', '--controller-manager-timeout', '30',
        '--param-file', str(Path(share) / 'config/ros2_controllers.yaml'))
    with (out / 'servo_admittance.log').open('w') as log:
        process = subprocess.Popen(
            ['ros2', 'launch', 'zhishu_robot_admittance', 'pallet_first_box_admittance.launch.py'],
            stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
    (out / 'servo_admittance.pid').write_text(str(process.pid) + '\n')
    run('ros2', 'service', 'call', '/left_servo/switch_command_type',
        'moveit_msgs/srv/ServoCommandType', '{command_type: 1}')
    run('ros2', 'service', 'call', '/right_servo/switch_command_type',
        'moveit_msgs/srv/ServoCommandType', '{command_type: 1}')
    run('ros2', 'control', 'switch_controllers', '--deactivate', 'dual_arm_controller',
        '--activate', 'left_arm_controller', 'right_arm_controller', '--strict')
    print('ADMITTANCE_TEST_START target=pallet_box_r0_c0 distance=0.05', flush=True)
    # Use measured box displacement to stop. Servo/Gazebo timing can make
    # velocity integration differ from physical travel in this grasp posture.
    command = ['ros2', 'run', 'zhishu_robot_admittance', 'run_admittance_validation',
               '--ros-args', '-p', 'world_name:=pallet_bin_15kg',
               '-p', 'model_name:=pallet_box_r0_c0', '-p', 'linear_distance_m:=0.050',
               '-p', 'arm_linear_distance_m:=0.050', '-p', 'linear_speed_mps:=0.005',
               '-p', 'stop_on_measured_distance:=true',
               '-p', 'linear_distance_tolerance_m:=0.002',
               '-p', 'require_reaction_estimate:=true',
               '-p', 'linear_axis:=z', '-p', f'output_path:={out / "validation.json"}']
    with (out / 'validation.log').open('w') as log:
        result = subprocess.run(command, stdout=log, stderr=subprocess.STDOUT)
    print((out / 'validation.log').read_text(), flush=True)
    return result.returncode


if __name__ == '__main__':
    raise SystemExit(main())
