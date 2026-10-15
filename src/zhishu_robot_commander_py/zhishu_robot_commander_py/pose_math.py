"""ABB-style pure target math, in ROS SI units (metres/radians, quaternion xyzw).

Offs operates in the target's object coordinates. RelTool translates in the
ORIGINAL target tool axes, then rotates about X, NEW Y, NEW Z, per ABB TRM.
Neither function queries current robot pose or changes any coordinate registry.
"""
import copy
from dataclasses import replace
import math


def _checked(point, values):
    from .commander import Commander
    message = Commander._pose_message(point)
    numbers = [float(v) for v in values]
    if not all(math.isfinite(v) for v in numbers):
        raise ValueError('offsets and angles must be finite')
    p, q = message.pose.position, message.pose.orientation
    norm = math.hypot(q.x, q.y, q.z, q.w)
    return (p.x, p.y, p.z), (q.x/norm, q.y/norm, q.z/norm, q.w/norm), numbers


def _result(point, position, quaternion=None):
    from .commander import Commander, PoseTarget
    if isinstance(point, PoseTarget):
        changes = dict(zip(('x', 'y', 'z'), position))
        if quaternion is not None:
            changes.update(zip(('qx', 'qy', 'qz', 'qw'), quaternion))
        result = replace(point, **changes)
    else:
        result = copy.deepcopy(point)
        result.pose.position.x, result.pose.position.y, result.pose.position.z = position
        if quaternion is not None:
            (result.pose.orientation.x, result.pose.orientation.y,
             result.pose.orientation.z, result.pose.orientation.w) = quaternion
    Commander._pose_message(result)  # Also rejects arithmetic overflow.
    return result


def _multiply(a, b):
    x, y, z, w = a
    X, Y, Z, W = b
    return (w*X+x*W+y*Z-z*Y, w*Y-x*Z+y*W+z*X,
            w*Z+x*Y-y*X+z*W, w*W-x*X-y*Y-z*Z)


def Offs(point, dx_m, dy_m, dz_m):
    """Return a new target translated along its workobject axes; attitude unchanged."""
    p, _, delta = _checked(point, (dx_m, dy_m, dz_m))
    return _result(point, tuple(a+b for a, b in zip(p, delta)))


def RelTool(point, dx_m, dy_m, dz_m, rx_rad=0., ry_rad=0., rz_rad=0.):
    """Return a new target translated/rotated in its local tool coordinates."""
    p, q, values = _checked(point, (dx_m, dy_m, dz_m, rx_rad, ry_rad, rz_rad))
    dx, dy, dz, rx, ry, rz = values
    rotated = _multiply(_multiply(q, (dx, dy, dz, 0.)), (-q[0], -q[1], -q[2], q[3]))
    qx = (math.sin(rx/2), 0., 0., math.cos(rx/2))
    qy = (0., math.sin(ry/2), 0., math.cos(ry/2))
    qz = (0., 0., math.sin(rz/2), math.cos(rz/2))
    result_q = _multiply(_multiply(_multiply(q, qx), qy), qz)
    norm = math.sqrt(sum(v*v for v in result_q))
    return _result(point, tuple(p[i]+rotated[i] for i in range(3)),
                   tuple(v/norm for v in result_q))


offs = Offs
rel_tool = RelTool
