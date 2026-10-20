#!/usr/bin/env python3
"""Start a fresh nine-box test only after the named one-box integration audit passes."""
import argparse
import json
from pathlib import Path
import subprocess
import sys
import time

ROOT=Path(__file__).resolve().parents[1]

def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--pilot-result',type=Path,required=True)
    parser.add_argument('--pilot-box-index',type=int,choices=range(9),default=0)
    parser.add_argument('--output-dir',type=Path,required=True)
    args=parser.parse_args()
    result=args.pilot_result.resolve()
    if args.output_dir.exists(): parser.error('output directory already exists')
    print('PROCESS_VALIDATION_WAITING_FOR_PILOT '+str(result),flush=True)
    deadline=time.monotonic()+4*3600
    while not result.exists():
        if time.monotonic()>deadline: raise RuntimeError('Pilot result deadline expired; no full run started')
        time.sleep(2)
    pilot=json.loads(result.read_text())
    if not (pilot.get('diagnostic_only') and pilot.get('diagnostic_passed') is True and pilot.get('completed')==[args.pilot_box_index] and
            pilot.get('failure') is None and len(pilot.get('evidence',[]))==1 and pilot['evidence'][0].get('index')==args.pilot_box_index and
            pilot['evidence'][0].get('process_admittance',{}).get('passed')):
        raise RuntimeError('Pilot process-admittance evidence failed; no nine-box run started')
    print('PROCESS_VALIDATION_PILOT_PASSED waiting for scene cleanup',flush=True)
    time.sleep(30)
    print('PROCESS_VALIDATION_START_FRESH_NINE '+str(args.output_dir),flush=True)
    subprocess.run([sys.executable,str(ROOT/'scripts/run_pallet_nine_continuous.py'),
                    '--output-dir',str(args.output_dir),'--no-record','--process-admittance'],
                   cwd=ROOT,check=True)

if __name__=='__main__': main()
