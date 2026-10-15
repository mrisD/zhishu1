import sys
import unittest
from pathlib import Path
import numpy as np
sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'scripts'))
from workobject_math import (solve_three_points, rotation_rpy, quaternion_from_matrix,
                             matrix_from_quaternion, pose_matrix, transform_point, rotation_error_deg)


class WorkobjectTests(unittest.TestCase):
    def setUp(self):
        self.truth = np.eye(4)
        self.truth[:3, :3] = rotation_rpy(-.04, .06, .28)
        self.truth[:3, 3] = [.55, -.225, .53]
        self.local = [[-.06, 0, 0], [.06, 0, 0], [0, .06, 0]]
        self.points = [transform_point(self.truth, p) for p in self.local]

    def test_axis_intersection_not_x1(self):
        result, _ = solve_three_points(self.points)
        np.testing.assert_allclose(result, self.truth, atol=1e-12)
        self.assertAlmostEqual(np.linalg.norm(result[:3, 3]-self.points[0]), .06)

    def test_first_point_origin(self):
        result, _ = solve_three_points(self.points, 'first_point')
        np.testing.assert_allclose(result[:3, 3], self.points[0])
        np.testing.assert_allclose(result[:3, :3], self.truth[:3, :3], atol=1e-12)

    def test_holdout_and_object_offset(self):
        user, _ = solve_three_points(self.points)
        offset = np.eye(4)
        offset[:3, 3] = [.02, .03, .01]
        offset[:3, :3] = rotation_rpy(0, 0, .7)
        p = [.01, -.02, .05]
        np.testing.assert_allclose(transform_point(user @ offset, p),
                                   transform_point(user, transform_point(offset, p)))

    def test_degenerate_points_rejected(self):
        for points in ([[0, 0, 0], [.001, 0, 0], [0, .05, 0]],
                       [[0, 0, 0], [.1, 0, 0], [.05, 0, 0]],
                       [[0, 0, 0], [.1, 0, 0], [1., .03, 0]]):
            with self.assertRaises(ValueError):
                solve_three_points(points)

    def test_invalid_inputs(self):
        for points in ([], [[np.nan]*3]*3, [[0]*3]*2):
            with self.assertRaises(ValueError):
                solve_three_points(points)
        with self.assertRaises(ValueError):
            solve_three_points(self.points, 'unknown')

    def test_quaternion_roundtrip(self):
        for rpy in ((np.pi, 0, 0), (0, np.pi, 0), (0, 0, np.pi), (-.04, .06, .28)):
            rotation = rotation_rpy(*rpy)
            np.testing.assert_allclose(matrix_from_quaternion(quaternion_from_matrix(rotation)), rotation, atol=1e-12)
            result = pose_matrix([1, 2, 3], quaternion_from_matrix(rotation))
            np.testing.assert_allclose(result[:3, :3], rotation, atol=1e-12)
            np.testing.assert_allclose(result[:3, 3], [1, 2, 3])

    def test_measurement_noise(self):
        points = np.asarray(self.points)+np.random.default_rng(42).normal(0, .0001, (3, 3))
        result, _ = solve_three_points(points)
        self.assertLess(np.linalg.norm(result[:3, 3]-self.truth[:3, 3]), .0005)
        self.assertLess(rotation_error_deg(result[:3, :3], self.truth[:3, :3]), .3)


if __name__ == '__main__':
    unittest.main()
