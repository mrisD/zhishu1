#!/usr/bin/env python3
"""Summarize raw child-frame wrist F/T samples; forces N, torques N m."""
import argparse
import csv
import json
import math
from pathlib import Path

AXES=('fx','fy','fz','tx','ty','tz')

def summarize(path):
    stages={}
    invalid=malformed=0
    for row in csv.DictReader(Path(path).open()):
        try:
            time=float(row['time'])
            left=[float(row['left_'+axis]) for axis in AXES]
            right=[float(row['right_'+axis]) for axis in AXES]
            skew=abs(float(row['left_stamp'])-float(row['right_stamp']))
        except (KeyError,TypeError,ValueError):
            malformed+=1
            continue
        if not all(math.isfinite(x) for x in [time,skew,*left,*right]):
            invalid+=1
            continue
        stage=stages.setdefault(row['stage'],dict(samples=0,start_time=time,end_time=time,max_stamp_skew=0,
            left=dict(force_peak_N=0,torque_peak_Nm=0,force_sum=0,above_limits_samples=0),
            right=dict(force_peak_N=0,torque_peak_Nm=0,force_sum=0,above_limits_samples=0)))
        stage['samples']+=1; stage['end_time']=time
        stage['max_stamp_skew']=max(stage['max_stamp_skew'],skew)
        for name,values in [('left',left),('right',right)]:
            item=stage[name]
            force=math.sqrt(sum(x*x for x in values[:3])); torque=math.sqrt(sum(x*x for x in values[3:]))
            item['force_peak_N']=max(item['force_peak_N'],force)
            item['torque_peak_Nm']=max(item['torque_peak_Nm'],torque)
            item['force_sum']+=force
            item['above_limits_samples']+=int(force>450 or torque>100)
            item['end_raw_F_N_T_Nm']=values
            item['end_force_norm_N']=force
    for stage in stages.values():
        for name in ('left','right'):
            stage[name]['force_mean_N']=stage[name].pop('force_sum')/stage['samples']
    return dict(source=str(Path(path).resolve()),axis_order=list(AXES),
                coordinate_frame='raw wrist sensor child frames; norms are not per-arm weight shares',
                nonfinite_rows=invalid,malformed_rows=malformed,stages=stages,
                note='Recorded sample peaks; 50 Hz sampling does not establish continuous-time force bounds')

if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('csv',type=Path)
    parser.add_argument('--output',type=Path)
    args=parser.parse_args()
    text=json.dumps(summarize(args.csv),ensure_ascii=False,indent=2)+'\n'
    if args.output: args.output.write_text(text)
    else: print(text,end='')
