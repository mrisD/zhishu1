#!/usr/bin/env python3
"""Back an empty robot away from the table using real body mesh clearance."""
import argparse,itertools,json,math,struct,time
from pathlib import Path
import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped,Twist
from sensor_msgs.msg import JointState
from moveit_msgs.srv import GetPositionFK
from rclpy.qos import qos_profile_sensor_data


def matrix(p):
    x,y,z,w=p.orientation.x,p.orientation.y,p.orientation.z,p.orientation.w
    a=np.eye(4);a[:3,:3]=[[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],[2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],[2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]]
    a[:3,3]=[p.position.x,p.position.y,p.position.z]
    return a


def mesh_vertices(path):
    raw=Path(path).read_bytes();count=struct.unpack_from('<I',raw,80)[0]
    return np.frombuffer(raw,offset=84,count=count,dtype=[('n','<f4',3),('v','<f4',(3,3)),('a','<u2')])['v'].reshape(-1,3).astype(float)


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--output-dir',type=Path,required=True);p.add_argument('--gap',type=float,default=.220);a=p.parse_args()
    if a.gap < .200: p.error('Required horizontal clearance is at least 0.200 m')
    a.output_dir.mkdir(parents=True,exist_ok=True)
    root=Path(__file__).resolve().parents[1]
    rclpy.init();n=rclpy.create_node('retreat_column_from_table');state={};pub=n.create_publisher(Twist,'/cmd_vel',10)
    def receive(key,msg):state[key]=(msg,time.monotonic())
    n.create_subscription(JointState,'/joint_states',lambda m:receive('joints',m),qos_profile_sensor_data)
    n.create_subscription(PoseStamped,'/pallet_bin/measured_base_pose',lambda m:receive('base',m),10)
    n.create_subscription(PoseStamped,'/pallet_bin/measured_box_pose',lambda m:receive('box',m),10)
    deadline=time.monotonic()+15
    while len(state)<3 and time.monotonic()<deadline:rclpy.spin_once(n,timeout_sec=.05)
    if len(state)<3:raise RuntimeError('Missing measured pose or joint states')
    joints=dict(zip(state['joints'][0].name,state['joints'][0].position))
    if max(abs(joints[k]) for k in ('left_gripper_left_finger_joint','gripper_left_finger_joint'))>.001:raise RuntimeError('Retreat requires empty open jaws')
    if abs(state['box'][0].pose.position.z-.636)>.002:raise RuntimeError('First tote must remain seated before empty retreat')
    start_waist=[joints[k] for k in ('waist_up_down_joint','waist_pitch_joint')]
    q=GetPositionFK.Request();q.header.frame_id='base_footprint';q.fk_link_names=['waist_up_down_link','waist_pitch_link'];q.robot_state.joint_state=state['joints'][0]
    c=n.create_client(GetPositionFK,'/compute_fk');c.wait_for_service(timeout_sec=5);f=c.call_async(q);rclpy.spin_until_future_complete(n,f,timeout_sec=10)
    base_points={}
    for name,pose in zip(q.fk_link_names,f.result().pose_stamped):
        v=mesh_vertices(root/'src/zhishu_robot_description/meshes'/f'{name}.STL')
        # Include the collision proxy as well as the visible outer surface.
        if name=='waist_up_down_link':center=np.array([-.000002,0,.25]);half=[.11,.11,.25]
        else:center=np.array([.015,-.000342,.25]);half=[.135,.14,.25]
        corners=np.array(list(itertools.product(*[[-h,h] for h in half])))+center
        v=np.vstack([v,corners]);tf=matrix(pose.pose);base_points[name]=v@tf[:3,:3].T+tf[:3,3]
    def measure():
        b=matrix(state['base'][0].pose)
        return {k:float(.655-(v@b[0,:3]+b[0,3]).max()) for k,v in base_points.items()}
    p0=state['base'][0].pose;q0=p0.orientation;yaw0=math.atan2(2*(q0.w*q0.z+q0.x*q0.y),1-2*(q0.y*q0.y+q0.z*q0.z))
    if abs(yaw0)>.03:raise RuntimeError('Straight retreat requires the inspected forward-facing dock')
    before={'gaps_m':measure(),'base_x':p0.position.x,'base_y':p0.position.y}
    (a.output_dir/'retreat_before.json').write_text(json.dumps(before,indent=2));print('Before',before,flush=True)
    trace=[];deadline=time.monotonic()+120
    try:
        while time.monotonic()<deadline:
            rclpy.spin_once(n,timeout_sec=.025)
            if any(time.monotonic()-received>.5 for _,received in state.values()):raise RuntimeError('Stale retreat feedback')
            current=dict(zip(state['joints'][0].name,state['joints'][0].position))
            if max(abs(current[k]-v) for k,v in zip(('waist_up_down_joint','waist_pitch_joint'),start_waist))>.0005:raise RuntimeError('Body posture changed during the empty retreat')
            gaps=measure();gap=min(gaps.values());pnow=state['base'][0].pose
            trace.append({'base_x':pnow.position.x,'gaps_m':gaps})
            if gap>=a.gap:break
            qn=pnow.orientation;yaw=math.atan2(2*(qn.w*qn.z+qn.x*qn.y),1-2*(qn.y*qn.y+qn.z*qn.z))
            cmd=Twist();cmd.linear.x=-min(.05,max(.006,.7*(a.gap-gap)));cmd.angular.z=max(-.03,min(.03,2*(yaw0-yaw)));pub.publish(cmd)
        else:raise RuntimeError('Empty retreat timed out')
        pub.publish(Twist())
        until=time.monotonic()+2
        while time.monotonic()<until:pub.publish(Twist());rclpy.spin_once(n,timeout_sec=.05)
        gaps=measure();pnow=state['base'][0].pose
        after={'gaps_m':gaps,'minimum_gap_m':min(gaps.values()),'base_x':pnow.position.x,'base_y':pnow.position.y,'required_gap_m':.2,'passed':min(gaps.values())>=.2}
        (a.output_dir/'retreat_after.json').write_text(json.dumps(after,indent=2));print('After',after,flush=True)
        if not after['passed']:raise RuntimeError('Measured body clearance is below 200 mm')
    finally:
        pub.publish(Twist());(a.output_dir/'retreat_trace.json').write_text(json.dumps(trace));n.destroy_node();rclpy.shutdown()


if __name__=='__main__':main()
