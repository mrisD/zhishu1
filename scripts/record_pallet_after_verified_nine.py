#!/usr/bin/env python3
"""Record a fresh nine-box run only after an entire headless admittance run passes."""
import argparse
import json
from pathlib import Path
import subprocess
import sys
import time
from pallet_run_contract import verify_environment

ROOT=Path(__file__).resolve().parents[1]
ORDER=[0,1,2,6,7,8,5,4,3]

def verification_error(result):
    required=('passed','process_admittance_passed','all_nine_initially_physical','single_continuous_scene')
    if (not all(result.get(k) is True for k in required) or
        any(result.get(k) for k in ('diagnostic_only','second_row_diagnostic','third_row_diagnostic')) or
        result.get('failure') is not None or result.get('completed') != ORDER or
        any(result.get(k,False) for k in ('resumed','stitched')) or
        any(result.get(k,0) != 0 for k in ('scene_restarts','recording_restarts'))):
        return 'Full nine-box verification failed; recording not started'
    evidence=result.get('evidence',[])
    if (len(evidence)!=9 or [e.get('index') for e in evidence]!=ORDER or
        any(e.get('process_admittance',{}).get('passed') is not True for e in evidence)):
        return 'Nine ordered per-box audits required before recording'
    return None


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--verified-run',type=Path,required=True)
    p.add_argument('--output-dir',type=Path,required=True)
    p.add_argument('--release-hold-after-pass',action='store_true',help='Archive the recording hold only after full nine-box evidence passes')
    a=p.parse_args();run=a.verified_run.resolve();out=a.output_dir.resolve()
    if out.exists():p.error('recording output already exists')
    print('RECORDING_WAITING_FOR_FULL_NINE '+str(run),flush=True)
    deadline=time.monotonic()+12*3600
    while not (run/'sequence_result.json').exists():
        if time.monotonic()>deadline:raise RuntimeError('Verification deadline expired; recording not started')
        time.sleep(2)
    result=json.loads((run/'sequence_result.json').read_text())
    contract=verify_environment()
    if contract and result.get('execution_contract')!=contract:
        raise RuntimeError('Validation executable/configuration contract differs from recording')
    error=verification_error(result)
    if error: raise RuntimeError(error)
    hold=ROOT/'output/pallet_nine_recording_hold.json'
    if hold.exists():
        if not a.release_hold_after_pass:
            raise RuntimeError('Recording hold exists; recording not started')
        archive=run/'recording_hold_released.json'
        if archive.exists(): raise RuntimeError('Hold archive already exists')
        hold.rename(archive)
        print('RECORDING_HOLD_RELEASED_AFTER_FULL_NINE '+str(run),flush=True)
    print('ALL_NINE_VERIFIED_START_FRESH_RECORDING '+str(out),flush=True)
    time.sleep(30) # permit verification runner to finish scene cleanup
    subprocess.run([sys.executable,str(ROOT/'scripts/run_pallet_nine_continuous.py'),
                    '--output-dir',str(out),'--process-admittance'],cwd=ROOT,check=True)

if __name__=='__main__':main()
