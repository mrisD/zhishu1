#!/usr/bin/env python3
"""Start all three rear-row transfers only after all three middle-row audits pass."""
import argparse
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time

ROOT=Path(__file__).resolve().parents[1]

def middle_row_verified(result):
    evidence=result.get('evidence',[])
    return (result.get('second_row_diagnostic') is True and
            result.get('requested_order')==[5,4,3] and result.get('completed')==[5,4,3] and
            result.get('diagnostic_passed') is True and result.get('failure') is None and
            result.get('process_admittance_requested') is True and
            len(evidence)==3 and [e.get('index') for e in evidence]==[5,4,3] and
            all(e.get('process_admittance',{}).get('passed') is True for e in evidence))

def close_previous_gui(partition):
    for proc in Path('/proc').iterdir():
        if not proc.name.isdigit():continue
        try:
            env=(proc/'environ').read_bytes().split(b'\0')
            args=[a.decode(errors='replace') for a in (proc/'cmdline').read_bytes().split(b'\0') if a]
            if ('GZ_PARTITION='+partition).encode() in env and args and (
                    args[0].startswith('gz sim -g') or ('sim' in args and '-g' in args)):
                os.kill(int(proc.name),signal.SIGTERM)
        except (OSError,ValueError):pass

def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--verified-run',type=Path,required=True)
    parser.add_argument('--output-dir',type=Path,required=True)
    parser.add_argument('--show-gui',action='store_true')
    args=parser.parse_args()
    source=args.verified_run.resolve();out=args.output_dir.resolve()
    if out.exists():parser.error('output directory already exists')
    print('ROW_SEQUENCE_WAITING_FOR_SECOND_ROW '+str(source),flush=True)
    deadline=time.monotonic()+12*3600
    while not (source/'sequence_result.json').exists():
        if time.monotonic()>deadline:raise RuntimeError('Second-row verification deadline expired')
        time.sleep(2)
    result=json.loads((source/'sequence_result.json').read_text())
    if not middle_row_verified(result):
        raise RuntimeError('Second-row three-box audit did not pass; third row not started')
    print('SECOND_ROW_ALL_THREE_VERIFIED_WAITING_FOR_CLEANUP',flush=True)
    time.sleep(30)
    command=[sys.executable,str(ROOT/'scripts/run_pallet_nine_continuous.py'),
             '--output-dir',str(out),'--third-row-diagnostic','--box-limit','3',
             '--no-record','--process-admittance']
    print('ROW_SEQUENCE_START_FRESH_THIRD_ROW '+str(out),flush=True)
    run=subprocess.Popen(command,cwd=ROOT)
    try:
        if args.show_gui:
            wait_until=time.monotonic()+120
            while not (out/'network_config.json').exists():
                if run.poll() is not None:raise RuntimeError('Third-row runner exited during startup')
                if time.monotonic()>wait_until:raise RuntimeError('Third-row network configuration missing')
                time.sleep(.5)
            close_previous_gui(source.name)
            with (out/'gui.log').open('w') as log:
                subprocess.Popen([sys.executable,str(ROOT/'scripts/open_pallet_sim_gui.py'),
                    '--run-dir',str(out)],cwd=ROOT,stdout=log,stderr=subprocess.STDOUT,
                    start_new_session=True)
        code=run.wait()
        if code:raise RuntimeError('Third-row verification failed; inspect its result and log')
        third=json.loads((out/'sequence_result.json').read_text())
        if not (third.get('third_row_diagnostic') and third.get('completed')==[6,7,8]
                and third.get('diagnostic_passed') is True and third.get('failure') is None):
            raise RuntimeError('Third-row three-box verification incomplete')
        print('BOTH_ROWS_VERIFIED_SEPARATELY second=[5,4,3] third=[6,7,8]; no nine-box pass claimed',flush=True)
    finally:
        if run.poll() is None:
            run.send_signal(signal.SIGINT)
            try:run.wait(timeout=60)
            except subprocess.TimeoutExpired:run.terminate();run.wait(timeout=15)

if __name__=='__main__':main()
