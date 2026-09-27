import numpy as np

from zhishu_robot_bottle_grasp.geometry import (
    CameraIntrinsics,
    depth_to_meters,
    estimate_bottle_geometry,
    project_masked_depth,
    transform_points,
)


def test_depth_units_and_projection():
    depth = np.full((5, 5), 1000, dtype=np.uint16)
    mask = np.zeros((5, 5), dtype=bool)
    mask[2, 2] = True
    meters = depth_to_meters(depth, "16UC1")
    points = project_masked_depth(meters, mask, CameraIntrinsics(100, 100, 2, 2), stride=1)
    np.testing.assert_allclose(points, [[0.0, 0.0, 1.0]])


def test_rigid_transform():
    points = np.array([[1.0, 0.0, 0.0]])
    half = np.sqrt(0.5)
    transformed = transform_points(points, [1.0, 2.0, 3.0], [0.0, 0.0, half, half])
    np.testing.assert_allclose(transformed, [[1.0, 3.0, 3.0]], atol=1e-8)


def test_unknown_bottle_dimensions_use_table_plane():
    rng = np.random.default_rng(7)
    angles = rng.uniform(0.0, 2.0 * np.pi, 3000)
    radii = 0.035 * np.sqrt(rng.uniform(0.0, 1.0, 3000))
    z = rng.uniform(0.72, 0.94, 3000)
    bottle = np.column_stack((0.45 + radii * np.cos(angles),
                              -0.10 + radii * np.sin(angles), z))
    table = np.column_stack((rng.uniform(0.30, 0.60, 500),
                             rng.uniform(-0.25, 0.05, 500),
                             rng.normal(0.70, 0.001, 500)))
    estimate = estimate_bottle_geometry(bottle, table)
    assert abs(estimate.size[2] - 0.24) < 0.015
    assert 0.055 < estimate.size[0] < 0.080
    assert 0.055 < estimate.size[1] < 0.080
    np.testing.assert_allclose(estimate.center[:2], [0.45, -0.10], atol=0.006)
