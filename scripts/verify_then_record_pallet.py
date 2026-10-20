#!/usr/bin/env python3
"""Run one-box regression, fresh full-nine validation, then gated fresh recording."""
import argparse
from pathlib import Path
import subprocess
import sys
import os
from pallet_run_contract import create, verify_environment

ROOT=Path(__file__).resolve().parents[1]

def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output-dir',type=Path,required=True)
    parser.add_argument('--full-nine-first',action='store_true',help='Validate the full initial nine-box scene directly')
    parser.add_argument('--show-gui',action='store_true')
    args=parser.parse_args()
    root=args.output_dir.resolve()
    root.mkdir(parents=True,exist_ok=False)
    contract=root/'execution_contract.json'
    digest=create(ROOT,contract)
    os.environ['PALLET_RUN_CONTRACT']=str(contract)
    print('WORKFLOW_EXECUTION_CONTRACT '+digest,flush=True)
    def run(script,*arguments):
        verify_environment()
        print('WORKFLOW_START '+script+' '+' '.join(map(str,arguments)),flush=True)
        subprocess.run([sys.executable,str(ROOT/'scripts'/script),*map(str,arguments)],cwd=ROOT,check=True)
    if args.show_gui:
        subprocess.Popen([sys.executable,str(ROOT/'scripts/show_pallet_workflow_gui.py'),
                          '--workflow-dir',str(root),'--parent-pid',str(os.getpid())],cwd=ROOT,stdin=subprocess.DEVNULL)
    pilot=root/'pilot'
    full=root/'full_nine'
    recording=root/'recording'
    if args.full_nine_first:
        run('run_pallet_nine_continuous.py','--output-dir',full,'--no-record','--process-admittance')
    else:
        run('run_pallet_nine_continuous.py','--output-dir',pilot,'--box-limit','1','--no-record','--process-admittance')
        run('continue_pallet_process_validation.py','--pilot-result',pilot/'sequence_result.json','--output-dir',full)
    run('record_pallet_after_verified_nine.py','--verified-run',full,'--output-dir',recording,'--release-hold-after-pass')
    import json
    from record_pallet_after_verified_nine import verification_error
    result=json.loads((recording/'sequence_result.json').read_text())
    if result.get('execution_contract')!=verify_environment():
        raise RuntimeError('Recorded run execution contract mismatch')
    error=verification_error(result)
    if error: raise RuntimeError('Recorded run rejected: '+error)
    print('WORKFLOW_RECORDED_NINE_PASSED '+str(recording),flush=True)

if __name__=='__main__':main()
