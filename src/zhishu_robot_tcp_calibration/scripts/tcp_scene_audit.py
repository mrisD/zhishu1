#!/usr/bin/env python3
"""Read actual Gazebo geometry and compare rendered needle apex to its TCP."""
import subprocess
import numpy as np
from google.protobuf import text_format, json_format
from gz.msgs10.scene_pb2 import Scene
from gz.msgs10.pose_pb2 import Pose
import json


def transform(pose):
    q = pose.orientation
    x, y, z, w = q.x, q.y, q.z, q.w
    norm = np.linalg.norm([x, y, z, w])
    if norm:
        x, y, z, w = np.array([x, y, z, w])/norm
    else:
        w = 1.0
    result = np.eye(4)
    result[:3, :3] = [
        [1-2*y*y-2*z*z, 2*x*y-2*z*w, 2*x*z+2*y*w],
        [2*x*y+2*z*w, 1-2*x*x-2*z*z, 2*y*z-2*x*w],
        [2*x*z-2*y*w, 2*y*z+2*x*w, 1-2*x*x-2*y*y]]
    result[:3, 3] = [pose.position.x, pose.position.y, pose.position.z]
    return result


def main():
    result = subprocess.run([
        'gz', 'service', '-s', '/world/empty/scene/info', '--reqtype', 'gz.msgs.Empty',
        '--reptype', 'gz.msgs.Scene', '--timeout', '3000', '--req', ''],
        capture_output=True, text=True, check=True, timeout=8)
    scene = text_format.Parse(result.stdout, Scene())
    current = subprocess.run(['gz', 'topic', '-e', '-n', '1', '--json-output',
                              '-t', '/world/empty/pose/info'],
                             capture_output=True, text=True, check=True, timeout=8)
    current_poses = {p['id']: json_format.ParseDict(p, Pose())
                     for p in json.loads(current.stdout)['pose']}
    def matrix(entity):
        return transform(current_poses.get(entity.id, entity.pose))
    models = {m.name: m for m in scene.model}
    robot = models['zhishu_robot']
    fixture = models['tcp_pivot_fixture']
    tips = {}
    for link in robot.link:
        for visual in link.visual:
            if 'needle_sharp_tip' not in visual.name:
                continue
            side = 'right' if 'right_needle' in visual.name else 'left'
            flange_matrix = matrix(robot)@matrix(link)
            apex = flange_matrix@matrix(visual)@np.array([0, 0, visual.geometry.mesh.scale.z, 1])
            tcp = flange_matrix@np.array([0.015, 0, -0.30, 1])
            error = np.linalg.norm(apex[:3]-tcp[:3])*1000
            print('%s rendered apex / model TCP error: %.9f mm' % (side, error))
            tips[side] = apex[:3]
            if error > 0.001:
                raise RuntimeError('Rendered apex is not at the configured TCP')
    link = fixture.link[0]
    visual = next(v for v in link.visual if v.name == 'tip')
    fixed_apex = matrix(fixture)@matrix(link)@matrix(visual)@np.array(
        [0, 0, visual.geometry.mesh.scale.z, 1])
    print('Right tool / fixed needle tip separation: %.6f mm' %
          (np.linalg.norm(tips['right']-fixed_apex[:3])*1000))


if __name__ == '__main__':
    main()
