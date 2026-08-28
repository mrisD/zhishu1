"""Dependency-free math used by the realtime-ish Python control loop."""

import math


def clamp(value, lower, upper):
    if not all(math.isfinite(item) for item in (value, lower, upper)):
        raise ValueError("clamp arguments must be finite")
    if lower > upper:
        raise ValueError("clamp lower bound cannot exceed upper bound")
    return max(lower, min(upper, value))


def add(a, b):
    return [x + y for x, y in zip(a, b)]


def subtract(a, b):
    return [x - y for x, y in zip(a, b)]


def scale(values, factor):
    return [factor * value for value in values]


def cross(a, b):
    return [
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    ]


def rotate_vector(vector, quaternion):
    """Rotate vector by quaternion [x, y, z, w]."""
    if not all(math.isfinite(value) for value in list(vector) + list(quaternion)):
        raise ValueError("vector and quaternion values must be finite")
    qx, qy, qz, qw = quaternion
    q_norm = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
    if q_norm < 1e-12:
        raise ValueError("zero-length quaternion")
    qx, qy, qz, qw = (value / q_norm for value in quaternion)
    q_vector = [qx, qy, qz]
    uv = cross(q_vector, vector)
    uuv = cross(q_vector, uv)
    return add(vector, add(scale(uv, 2.0 * qw), scale(uuv, 2.0)))


def apply_deadband(value, deadband):
    if abs(value) <= deadband:
        return 0.0
    return math.copysign(abs(value) - deadband, value)


def admittance_step(error, mass, damping, stiffness, position, velocity, dt):
    acceleration = (error - damping * velocity - stiffness * position) / mass
    new_velocity = velocity + acceleration * dt
    new_position = position + new_velocity * dt
    return new_position, new_velocity
