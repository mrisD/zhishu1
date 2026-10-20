#!/usr/bin/env python3
"""Check live clocks, joint feedback, wrist sensors and Gazebo scene discovery."""
import argparse
import json
import math
import os
from pathlib import Path
import time

def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--run-dir',type=Path,required=True)
    parser.add_argument('--seconds',type=float,default=15)
    args=parser.parse_args()
    if not 5<=args.seconds<=60:parser.error('seconds must be between 5 and 60')
    run=args.run_dir.resolve()
    config=json.loads((run/'network_config.json').read_text())
    for key in ('ROS_DOMAIN_ID','GZ_PARTITION','GZ_IP','GZ_RELAY',
                'ROS_AUTOMATIC_DISCOVERY_RANGE','RMW_IMPLEMENTATION','CYCLONEDDS_URI'):
        os.environ[key]=config[key]
    import rclpy
    from rclpy.qos import qos_profile_sensor_data
    from rosgraph_msgs.msg import Clock
    from sensor_msgs.msg import JointState
    from geometry_msgs.msg import WrenchStamped
    from gz.transport13 import Node
    from gz.msgs10.empty_pb2 import Empty
    from gz.msgs10.scene_pb2 import Scene
    rclpy.init()
    node=rclpy.create_node('pallet_feedback_health_check')
    samples={name:{'count':0,'first':None,'last':None,'received':None,'finite':True}
             for name in ('/clock','/joint_states','/left_wrist_ft','/right_wrist_ft')}
    def callback(name,msg):
        stamp=msg.clock if name=='/clock' else msg.header.stamp
        value=stamp.sec+stamp.nanosec*1e-9
        s=samples[name];s['count']+=1
        if s['first'] is None:s['first']=value
        s['last']=value;s['received']=time.monotonic()
        if name=='/joint_states':
            s['finite']=s['finite'] and bool(msg.position) and all(math.isfinite(v) for v in msg.position)
        elif name!='/clock':
            w=msg.wrench
            s['finite']=s['finite'] and all(math.isfinite(v) for v in
                (w.force.x,w.force.y,w.force.z,w.torque.x,w.torque.y,w.torque.z))
    subscriptions=[]
    for name,kind in (('/clock',Clock),('/joint_states',JointState),
                      ('/left_wrist_ft',WrenchStamped),('/right_wrist_ft',WrenchStamped)):
        subscriptions.append(node.create_subscription(kind,name,
            lambda msg,name=name:callback(name,msg),qos_profile_sensor_data))
    try:
        deadline=time.monotonic()+args.seconds
        while time.monotonic()<deadline:rclpy.spin_once(node,timeout_sec=.1)
        now=time.monotonic()
        for s in samples.values():
            received=s.pop('received')
            s['age_seconds']=None if received is None else now-received
            s['passed']=s['count']>=10 and s['finite'] and s['last']>s['first'] and s['age_seconds']<2
        transport=Node()
        ok,scene=transport.request('/world/pallet_bin_15kg/scene/info',Empty(),Empty,Scene,3000)
        models=[m.name for m in scene.model] if ok else []
        result={'passed':all(s['passed'] for s in samples.values()) and ok and 'zhishu_robot' in models,
                'network':{k:config[k] for k in ('GZ_IP','GZ_RELAY','GZ_PARTITION')},
                'topics':samples,'gazebo_scene_service':ok,'robot_present':'zhishu_robot' in models}
        (run/'communication_health.json').write_text(json.dumps(result,indent=2)+'\n')
        print(json.dumps(result,indent=2),flush=True)
        return 0 if result['passed'] else 1
    finally:
        node.destroy_node();rclpy.shutdown()

if __name__=='__main__':raise SystemExit(main())
