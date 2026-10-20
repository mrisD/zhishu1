"""Pin the executable and runtime configuration throughout validation/recording."""
import hashlib
import json
import os
from pathlib import Path

def manifest(paths):
    return {str(Path(p).absolute()):hashlib.sha256(Path(p).read_bytes()).hexdigest() for p in paths}

def verify(path):
    expected=json.loads(Path(path).read_text())['files']
    actual=manifest(expected)
    changed=[p for p in expected if actual[p]!=expected[p]]
    if changed:raise RuntimeError('Execution contract changed; stop before next motion: '+', '.join(changed))
    return hashlib.sha256(json.dumps(expected,sort_keys=True).encode()).hexdigest()

def verify_environment():
    path=os.environ.get('PALLET_RUN_CONTRACT')
    return verify(path) if path else None

def create(root,output):
    paths=[root/'install/zhishu_robot_mtc_pick_place/lib/zhishu_robot_mtc_pick_place/pallet_bin_15kg_transfer']
    for pattern in ['src/zhishu_robot_mtc_pick_place/launch/pallet_bin_15kg_transfer.launch.py',
                    'install/zhishu_robot_mtc_pick_place/share/zhishu_robot_mtc_pick_place/launch/pallet_bin_15kg_transfer.launch.py',
                    'src/zhishu_robot_mtc_pick_place/src/pallet_bin_15kg_transfer.cpp',
                    'src/zhishu_robot_moveit_config/config/*',
                    'install/zhishu_robot_moveit_config/share/zhishu_robot_moveit_config/config/*',
                    'src/zhishu_robot_description/urdf/**/*',
                    'src/zhishu_robot_gazebo/worlds/pallet_bin_15kg.sdf']:
        paths.extend(p for p in root.glob(pattern) if p.is_file())
    paths.extend(root/'scripts'/name for name in ['run_pallet_nine_continuous.py','run_pallet_first_row_sequence.py',
        'record_pallet_bin_transfer.py','pallet_run_contract.py','verify_then_record_pallet.py',
        'record_pallet_after_verified_nine.py','continue_pallet_process_validation.py','show_pallet_workflow_gui.py'])
    output.write_text(json.dumps({'files':manifest(sorted(set(paths)))},indent=2)+'\n')
    return verify(output)
