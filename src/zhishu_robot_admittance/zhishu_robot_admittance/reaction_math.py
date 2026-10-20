"""Quasi-static vertical support from world-frame wrist forces.

Gazebo reports the load's force ON each wrist. Tool mass is downstream of
the sensor, so it must be added back; force norms cannot be summed here.
"""
import math


def bottom_reaction(left_fz, right_fz, box_mass=15.0, tool_mass=0.56, gravity=9.81):
    values = (left_fz, right_fz, box_mass, tool_mass, gravity)
    if not all(math.isfinite(v) for v in values):
        raise ValueError('Nonfinite force or mass')
    if box_mass <= 0.0 or tool_mass < 0.0 or gravity <= 0.0:
        raise ValueError('Invalid mass or gravity')
    return (box_mass + tool_mass) * gravity + left_fz + right_fz
