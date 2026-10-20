#!/usr/bin/env python3
"""Read Gazebo world statistics; print RTF without changing simulator state."""
import argparse
import json
import math
import os
from pathlib import Path
import time

def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--run-dir',type=Path,required=True)
    p.add_argument('--seconds',type=float,default=5)
    a=p.parse_args()
    if not 2<=a.seconds<=15:p.error('seconds must be 2..15')
    config=json.loads((a.run_dir/'network_config.json').read_text())
    for key in ('GZ_PARTITION','GZ_IP','GZ_RELAY'):os.environ[key]=config[key]
    from gz.transport13 import Node
    from gz.msgs10.world_stats_pb2 import WorldStatistics
    samples=[]
    def receive(msg):
        samples.append((time.monotonic(),msg.sim_time.sec+msg.sim_time.nsec*1e-9,
                        float(msg.real_time_factor),bool(msg.paused)))
    node=Node();topic='/world/pallet_bin_15kg/stats'
    if not node.subscribe(WorldStatistics,topic,receive):raise RuntimeError('Statistics subscription failed')
    time.sleep(a.seconds)
    node.unsubscribe(topic)
    good=[s for s in samples if math.isfinite(s[2]) and s[2]>=0]
    if len(good)<2:
        print(json.dumps({'available':False,'reason':'No fresh Gazebo statistics'}));return 1
    first,last=good[0],good[-1]
    result={'available':True,'reported_rtf':last[2],
            'sample_window_rtf':(last[1]-first[1])/(last[0]-first[0]),
            'wall_window_seconds':last[0]-first[0], 'samples':len(good),'paused':last[3],
            'age_seconds':time.monotonic()-last[0]}
    if result['age_seconds']>2:
        result['available']=False;result['reason']='Statistics stopped updating'
    print(json.dumps(result))
    return 0 if result['available'] else 1

if __name__=='__main__':raise SystemExit(main())
