#!/usr/bin/env python3
"""Necessary reach check for the retained straight-finger placement (no motion)."""
import argparse,json,math
from pathlib import Path
import xml.etree.ElementTree as ET
import numpy as np


def rot(axis,angle):
    axis=np.asarray(axis,dtype=float);axis/=np.linalg.norm(axis)
    x,y,z=axis;k=np.array([[0,-z,y],[z,0,-x],[-y,x,0]])
    return np.eye(3)+math.sin(angle)*k+(1-math.cos(angle))*(k@k)


def pose(xyz,rpy):
    t=np.eye(4);t[:3,3]=xyz;r,p,y=rpy
    t[:3,:3]=rot([0,0,1],y)@rot([0,1,0],p)@rot([1,0,0],r)
    return t


def audit(urdf,dock_x=.16,lift=.28,pitch=-.2,left_roll=-math.pi/2,right_roll=-math.pi/2,tcp_pitch=0):
    robot=ET.parse(urdf).getroot();joints=robot.findall('joint')
    values={'waist_up_down_joint':lift,'waist_pitch_joint':pitch}
    tf={'base_footprint':np.eye(4)};parents={}
    pending=list(joints)
    while pending:
        progressed=False
        for j in list(pending):
            parent=j.find('parent').get('link');child=j.find('child').get('link')
            if parent not in tf:continue
            o=j.find('origin');xyz=[float(v) for v in o.get('xyz','0 0 0').split()] if o is not None else [0,0,0]
            rpy=[float(v) for v in o.get('rpy','0 0 0').split()] if o is not None else [0,0,0]
            local=pose(xyz,rpy);motion=np.eye(4);v=values.get(j.get('name'),0.)
            axis=[float(v) for v in j.find('axis').get('xyz').split()] if j.find('axis') is not None else [1,0,0]
            if j.get('type')=='prismatic':motion[:3,3]=np.array(axis)*v
            elif j.get('type') in ('revolute','continuous'):motion[:3,:3]=rot(axis,v)
            tf[child]=tf[parent]@local@motion;parents[child]=(parent,float(np.linalg.norm(xyz)))
            pending.remove(j);progressed=True
        if not progressed:raise ValueError('Disconnected robot tree')
    result={'dock_x_m':dock_x,'waist_lift_m':lift,'waist_pitch_rad':pitch,'box_center_world_m':[1.03,-1.55,.636],
            'scope':'Necessary geometric bound for existing straight-down tool orientations; does not prove IK or collision feasibility.'}
    for side,hand,tcp,yaw,roll in [('left','L_hand_link','left_gripper_tcp',math.pi/2,left_roll),('right','R_hand_link','right_gripper_tcp',-math.pi/2,right_roll)]:
        shoulder=side+'_shoulder_pitch_link';bound=0.;link=hand
        while link!=shoulder:
            link,length=parents[link];bound+=length
        hand_to_tcp=np.linalg.inv(tf[hand])@tf[tcp]
        tcp_target=pose([1.03-dock_x, .195 if side=='left' else -.195, .636+.108-.0033267],[roll,tcp_pitch,yaw])
        hand_target=tcp_target@np.linalg.inv(hand_to_tcp)
        distance=float(np.linalg.norm(hand_target[:3,3]-tf[shoulder][:3,3]))
        result[side]={'shoulder_to_hand_required_m':distance,'maximum_chain_length_m':bound,'shoulder_xyz_base_m':tf[shoulder][:3,3].tolist(),'target_hand_xyz_base_m':hand_target[:3,3].tolist(),'hand_to_tcp_xyz_hand_m':hand_to_tcp[:3,3].tolist(),'passes_necessary_reach_bound':distance<=bound}
    result['existing_grip_can_reach']=all(result[s]['passes_necessary_reach_bound'] for s in ('left','right'))
    return result


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--urdf',type=Path,required=True);p.add_argument('--output',type=Path,required=True);p.add_argument('--lift',type=float,default=.28);p.add_argument('--pitch',type=float,default=-.2);p.add_argument('--left-roll',type=float,default=-math.pi/2);p.add_argument('--right-roll',type=float,default=-math.pi/2);a=p.parse_args()
    result=audit(a.urdf,lift=a.lift,pitch=a.pitch,left_roll=a.left_roll,right_roll=a.right_roll);a.output.write_text(json.dumps(result,indent=2));print(json.dumps(result,indent=2))


if __name__=='__main__':main()
