"""Pure geometry helpers used by both ROS nodes and unit tests."""

from dataclasses import dataclass
from typing import Optional

import numpy as np


@dataclass(frozen=True)
class CameraIntrinsics:
    fx: float
    fy: float
    cx: float
    cy: float


@dataclass(frozen=True)
class BottleEstimate:
    center: np.ndarray
    size: np.ndarray
    grasp_point: np.ndarray
    yaw: float
    table_height: float
    point_count: int


def depth_to_meters(depth: np.ndarray, encoding: str = "") -> np.ndarray:
    """Normalize Orbbec/Gazebo depth images to float32 metres.

    Orbbec normally publishes ``16UC1`` millimetres while Gazebo's depth
    camera publishes ``32FC1`` metres. The dtype fallback keeps bag files with
    incomplete encoding metadata usable.
    """
    array = np.asarray(depth)
    normalized_encoding = encoding.upper()
    if normalized_encoding in {"16UC1", "MONO16"} or array.dtype == np.uint16:
        return array.astype(np.float32) * 0.001
    return array.astype(np.float32)


def project_masked_depth(
    depth_m: np.ndarray,
    mask: np.ndarray,
    intrinsics: CameraIntrinsics,
    stride: int = 2,
    min_depth: float = 0.10,
    max_depth: float = 4.0,
) -> np.ndarray:
    """Back-project valid masked pixels into the optical camera frame."""
    if depth_m.shape[:2] != mask.shape[:2]:
        raise ValueError("depth and mask dimensions must match")
    step = max(1, int(stride))
    rows, cols = np.nonzero(mask[::step, ::step])
    rows = rows * step
    cols = cols * step
    if rows.size == 0:
        return np.empty((0, 3), dtype=np.float64)
    z = depth_m[rows, cols].astype(np.float64)
    valid = np.isfinite(z) & (z >= min_depth) & (z <= max_depth)
    rows, cols, z = rows[valid], cols[valid], z[valid]
    x = (cols.astype(np.float64) - intrinsics.cx) * z / intrinsics.fx
    y = (rows.astype(np.float64) - intrinsics.cy) * z / intrinsics.fy
    return np.column_stack((x, y, z))


def quaternion_matrix(x: float, y: float, z: float, w: float) -> np.ndarray:
    """Return a 3x3 rotation matrix for a normalized xyzw quaternion."""
    q = np.asarray([x, y, z, w], dtype=np.float64)
    norm = float(np.dot(q, q))
    if norm < 1e-16:
        return np.eye(3)
    q *= np.sqrt(2.0 / norm)
    q = np.outer(q, q)
    return np.array(
        [
            [1.0 - q[1, 1] - q[2, 2], q[0, 1] - q[2, 3], q[0, 2] + q[1, 3]],
            [q[0, 1] + q[2, 3], 1.0 - q[0, 0] - q[2, 2], q[1, 2] - q[0, 3]],
            [q[0, 2] - q[1, 3], q[1, 2] + q[0, 3], 1.0 - q[0, 0] - q[1, 1]],
        ]
    )


def transform_points(
    points: np.ndarray, translation: np.ndarray, quaternion_xyzw: np.ndarray
) -> np.ndarray:
    """Transform Nx3 camera-frame points into the target frame."""
    points = np.asarray(points, dtype=np.float64).reshape((-1, 3))
    rotation = quaternion_matrix(*np.asarray(quaternion_xyzw, dtype=np.float64))
    return points @ rotation.T + np.asarray(translation, dtype=np.float64)


def _trimmed(points: np.ndarray, low: float = 2.0, high: float = 98.0) -> np.ndarray:
    if len(points) < 20:
        return points
    lower = np.percentile(points, low, axis=0)
    upper = np.percentile(points, high, axis=0)
    keep = np.all((points >= lower) & (points <= upper), axis=1)
    return points[keep]


def estimate_bottle_geometry(
    object_points_base: np.ndarray,
    table_points_base: Optional[np.ndarray] = None,
    minimum_points: int = 80,
    minimum_height: float = 0.04,
    maximum_height: float = 0.60,
) -> BottleEstimate:
    """Estimate upright bottle dimensions and a side-grasp point.

    The surrounding depth ring provides the support-plane height. This is
    important for an overhead camera: the bottom of an opaque bottle is not
    directly visible, so using only object pixels systematically shortens it.
    """
    points = np.asarray(object_points_base, dtype=np.float64).reshape((-1, 3))
    points = points[np.all(np.isfinite(points), axis=1)]
    if len(points) < minimum_points:
        raise ValueError(f"not enough bottle points: {len(points)} < {minimum_points}")
    points = _trimmed(points)

    if table_points_base is not None:
        table = np.asarray(table_points_base, dtype=np.float64).reshape((-1, 3))
        table = table[np.all(np.isfinite(table), axis=1)]
    else:
        table = np.empty((0, 3), dtype=np.float64)

    if len(table) >= 30:
        table_height = float(np.median(table[:, 2]))
    else:
        table_height = float(np.percentile(points[:, 2], 2.0))

    top_height = float(np.percentile(points[:, 2], 98.0))
    height = float(np.clip(top_height - table_height, minimum_height, maximum_height))

    xy = points[:, :2]
    xy_center = np.median(xy, axis=0)
    centered = xy - xy_center
    covariance = np.cov(centered, rowvar=False)
    eigenvalues, eigenvectors = np.linalg.eigh(covariance)
    order = np.argsort(eigenvalues)[::-1]
    axes = eigenvectors[:, order]
    projected = centered @ axes
    lower = np.percentile(projected, 2.0, axis=0)
    upper = np.percentile(projected, 98.0, axis=0)
    spans = np.maximum(upper - lower, 0.015)
    yaw = float(np.arctan2(axes[1, 0], axes[0, 0]))

    center = np.array([xy_center[0], xy_center[1], table_height + 0.5 * height])
    grasp_height = float(np.clip(table_height + 0.55 * height,
                                 table_height + 0.035,
                                 table_height + height - 0.025))
    grasp_point = np.array([xy_center[0], xy_center[1], grasp_height])
    size = np.array([float(spans[0]), float(spans[1]), height])
    return BottleEstimate(center, size, grasp_point, yaw, table_height, len(points))
