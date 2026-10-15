"""Frame construction from measured points only; no simulator truth inputs."""
import numpy as np


def rotation_rpy(roll, pitch, yaw):
    cr, sr = np.cos(roll), np.sin(roll)
    cp, sp = np.cos(pitch), np.sin(pitch)
    cy, sy = np.cos(yaw), np.sin(yaw)
    return np.array([[cy*cp, cy*sp*sr-sy*cr, cy*sp*cr+sy*sr],
                     [sy*cp, sy*sp*sr+cy*cr, sy*sp*cr-cy*sr],
                     [-sp, cp*sr, cp*cr]])


def matrix_from_quaternion(q):
    q = np.asarray(q, dtype=float)
    if q.shape != (4,) or not np.isfinite(q).all() or np.linalg.norm(q) < 1e-12:
        raise ValueError('Invalid xyzw quaternion')
    x, y, z, w = q/np.linalg.norm(q)
    return np.array([[1-2*y*y-2*z*z, 2*x*y-2*z*w, 2*x*z+2*y*w],
                     [2*x*y+2*z*w, 1-2*x*x-2*z*z, 2*y*z-2*x*w],
                     [2*x*z-2*y*w, 2*y*z+2*x*w, 1-2*x*x-2*y*y]])


def quaternion_from_matrix(R):
    R = np.asarray(R, dtype=float)
    if R.shape != (3, 3) or not np.isfinite(R).all() or not np.allclose(R.T@R, np.eye(3), atol=1e-7) or np.linalg.det(R) < 0:
        raise ValueError('Expected a proper rotation matrix')
    tr = np.trace(R)
    if tr > 0:
        s = 2*np.sqrt(1+tr)
        q = [(R[2, 1]-R[1, 2])/s, (R[0, 2]-R[2, 0])/s, (R[1, 0]-R[0, 1])/s, s/4]
    else:
        i = int(np.argmax(np.diag(R)))
        j, k = (i+1) % 3, (i+2) % 3
        s = 2*np.sqrt(1+R[i, i]-R[j, j]-R[k, k])
        q = np.zeros(4)
        q[i], q[j], q[k], q[3] = s/4, (R[j, i]+R[i, j])/s, (R[k, i]+R[i, k])/s, (R[k, j]-R[j, k])/s
    q = np.asarray(q)/np.linalg.norm(q)
    return q if q[3] >= 0 else -q


def pose_matrix(position, rotation):
    T = np.eye(4)
    rotation = np.asarray(rotation, dtype=float)
    T[:3, :3] = matrix_from_quaternion(rotation) if rotation.shape == (4,) else rotation
    T[:3, 3] = position
    return T


def transform_point(T, point):
    return T[:3, :3]@np.asarray(point)+T[:3, 3]


def solve_three_points(points, origin_mode='axis_intersection'):
    p = np.asarray(points, dtype=float)
    if p.shape != (3, 3) or not np.isfinite(p).all():
        raise ValueError('Three finite measured TCP points required')
    x1, x2, y1 = p
    baseline = np.linalg.norm(x2-x1)
    if baseline < 0.04:
        raise ValueError('X1/X2 baseline is less than 40 mm')
    x = (x2-x1)/baseline
    projected = x1+x*np.dot(y1-x1, x)
    yvec = y1-projected
    height = np.linalg.norm(yvec)
    if height < 0.025 or height/max(np.linalg.norm(y1-x1), 1e-12) < 0.15:
        raise ValueError('Y1 is too close to the X-axis; degenerate frame')
    y = yvec/height
    z = np.cross(x, y)
    y = np.cross(z, x)
    if origin_mode == 'axis_intersection':
        origin = projected
    elif origin_mode == 'first_point':
        origin = x1
    else:
        raise ValueError('Unknown origin mode')
    return pose_matrix(origin, np.column_stack((x, y, z))), {
        'x_baseline_mm': float(baseline*1000), 'y_height_mm': float(height*1000)}


def rotation_error_deg(a, b):
    return float(np.degrees(np.arccos(np.clip((np.trace(a.T@b)-1)/2, -1, 1))))
