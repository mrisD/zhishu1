#!/usr/bin/env python3
"""Open a GUI with the exact network configuration of a pallet validation run."""
import argparse
import json
import os
from pathlib import Path

def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--run-dir',type=Path,required=True)
    args=parser.parse_args()
    config=json.loads((args.run_dir.resolve()/'network_config.json').read_text())
    for key in ('ROS_DOMAIN_ID','GZ_PARTITION','GZ_IP','GZ_RELAY',
                'ROS_AUTOMATIC_DISCOVERY_RANGE','RMW_IMPLEMENTATION','CYCLONEDDS_URI'):
        if not isinstance(config.get(key),str):
            raise ValueError('Missing network configuration: '+key)
        os.environ[key]=config[key]
    print('GAZEBO_GUI_CONNECT '+config['GZ_PARTITION']+' '+config['GZ_IP'],flush=True)
    os.execvp('gz',['gz','sim','-g'])

if __name__=='__main__':
    main()
