#!/usr/bin/env python3
"""Read-only MTC planning evidence recorder for one isolated validation run."""
import argparse
import json
import os
from pathlib import Path
import time
import zlib


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--run-dir',type=Path,required=True)
    args=parser.parse_args(); run=args.run_dir.resolve()
    os.environ['ROS_DOMAIN_ID']=str(20+zlib.crc32(str(run).encode())%180)
    os.environ['ROS_AUTOMATIC_DISCOVERY_RANGE']='LOCALHOST'
    import rclpy
    from rclpy.qos import QoSProfile,DurabilityPolicy,ReliabilityPolicy
    from rclpy.serialization import serialize_message
    from moveit_task_constructor_msgs.msg import Solution,TaskDescription
    out=run/'planning_solutions';out.mkdir(exist_ok=True)
    rclpy.init();node=rclpy.create_node('pallet_readonly_planning_recorder')
    for _ in range(6):rclpy.spin_once(node,timeout_sec=.5)
    count={'solution':0,'description':0}
    index=(out/'index.jsonl').open('a')
    subscriptions=[]
    def callback(kind,topic):
        def receive(message):
            count[kind]+=1
            name=f'{kind}_{count[kind]:05d}.cdr'
            (out/name).write_bytes(serialize_message(message))
            index.write(json.dumps(dict(kind=kind,file=name,topic=topic,
                                        task_id=getattr(message,'task_id',None),received_wall=time.time()))+'\n')
            index.flush()
        return receive
    for topic,message,kind in [('/solution',Solution,'solution'),('/description',TaskDescription,'description')]:
        publishers=node.get_publishers_info_by_topic(topic)
        durable=any(p.qos_profile.durability==DurabilityPolicy.TRANSIENT_LOCAL for p in publishers)
        qos=QoSProfile(depth=20,reliability=ReliabilityPolicy.RELIABLE,
                       durability=DurabilityPolicy.TRANSIENT_LOCAL if durable else DurabilityPolicy.VOLATILE)
        subscriptions.append(node.create_subscription(message,topic,callback(kind,topic),qos))
    print('READONLY_PLANNING_RECORDER_READY '+str(out),flush=True)
    deadline=time.monotonic()+8*3600
    try:
        while rclpy.ok() and time.monotonic()<deadline and not (run/'sequence_result.json').exists():
            rclpy.spin_once(node,timeout_sec=1)
    finally:
        index.close();node.destroy_node();rclpy.shutdown()
        print('READONLY_PLANNING_RECORDER_FINISHED '+json.dumps(count),flush=True)

if __name__=='__main__':main()
