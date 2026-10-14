"""Standalone mathematics cross-check for the 6-unknown pivot formulation.

Run: python3 -m unittest discover -s src/zhishu_robot_tcp_calibration/test
"""
import unittest
import numpy as np


def estimate(poses):
    A = np.concatenate([np.concatenate((R, -np.eye(3)), axis=1)
                        for R, _ in poses], axis=0)
    b = np.concatenate([-p for _, p in poses], axis=0)
    if np.linalg.matrix_rank(A) != 6:
        raise ValueError('degenerate rotation samples')
    result = np.linalg.lstsq(A, b, rcond=None)[0]
    return result[:3], result[3:]


def rot(axis, angle):
    axis = np.asarray(axis, dtype=float)
    axis /= np.linalg.norm(axis)
    K = np.array([[0, -axis[2], axis[1]],
                  [axis[2], 0, -axis[0]],
                  [-axis[1], axis[0], 0]])
    return np.eye(3) + np.sin(angle)*K + (1 - np.cos(angle))*K@K


def frame_from_elongators(reference, tcp, z_sample, x_sample):
    def tip_position(sample):
        R_flange, p_flange = sample
        return p_flange + R_flange @ tcp

    reference_tip = tip_position(reference)
    z_axis = tip_position(z_sample) - reference_tip
    x_axis = tip_position(x_sample) - reference_tip
    z_axis /= np.linalg.norm(z_axis)
    x_axis -= z_axis * np.dot(z_axis, x_axis)
    if np.linalg.norm(x_axis) < 1e-8:
        raise ValueError('degenerate X/Z elongator directions')
    x_axis /= np.linalg.norm(x_axis)
    y_axis = np.cross(z_axis, x_axis)
    y_axis /= np.linalg.norm(y_axis)
    x_axis = np.cross(y_axis, z_axis)
    R_base_tool = np.column_stack((x_axis, y_axis, z_axis))
    return reference[0].T @ R_base_tool


class TestPivot(unittest.TestCase):
    def test_four_point_exact(self):
        t = np.array([0.015, 0.0, 0.16])
        c = np.array([0.4, -0.4, 0.8])
        matrices = [np.eye(3), rot([1, 0, 0], 0.32),
                    rot([0, 1, 0], 0.32), rot([0, 0, 1], 0.32)]
        tcp, pivot = estimate([(R, c - R @ t) for R in matrices])
        np.testing.assert_allclose(tcp, t, atol=1e-12)
        np.testing.assert_allclose(pivot, c, atol=1e-12)

    def test_degeneracy(self):
        t = np.array([0.015, 0.0, 0.16])
        c = np.array([0.4, -0.4, 0.8])
        with self.assertRaises(ValueError):
            estimate([(np.eye(3), c-t)]*10)

    def test_noisy_12_point(self):
        t = np.array([0.015, 0.0, 0.16])
        c = np.array([0.4, -0.4, 0.8])
        rng = np.random.default_rng(42)
        poses = []
        for i in range(12):
            R = rot([np.cos(i), np.sin(i), 0.3], 0.2 + 0.03 * i)
            poses.append((R, c - R@t + rng.normal(scale=0.0002, size=3)))
        tcp, _ = estimate(poses)
        self.assertLess(np.linalg.norm(tcp-t), 0.003)

    def test_abb_tcp_z_x_orientation(self):
        tcp = np.array([0.015, -0.004, 0.16])
        pivot = np.array([0.4, -0.4, 0.8])
        flange_reference = rot([1, 2, -1], 0.4)
        tool_truth = rot([0.3, -0.5, 1], 0.6)
        length = 0.10
        z_sample = (flange_reference, pivot + length * flange_reference @ tool_truth[:, 2]
                    - flange_reference @ tcp)
        x_sample = (flange_reference, pivot + length * flange_reference @ tool_truth[:, 0]
                    - flange_reference @ tcp)
        tool_estimate = frame_from_elongators(
            (flange_reference, pivot-flange_reference@tcp), tcp, z_sample, x_sample)
        np.testing.assert_allclose(tool_estimate, tool_truth, atol=1e-12)

    def test_axis_reference_can_move_away_from_fixed_fixture(self):
        tcp = np.array([0.015, 0.0, -0.30])
        pivot = np.array([0.55, -0.225, 0.53])
        reference_tip = pivot + np.array([0.0, 0.0, 0.20])
        flange = rot([1, 2, -1], 0.4)
        tool = rot([0.3, -0.5, 1], 0.6)
        reference = (flange, reference_tip-flange@tcp)
        z = (flange, reference_tip+0.05*flange@tool[:, 2]-flange@tcp)
        x = (flange, reference_tip+0.05*flange@tool[:, 0]-flange@tcp)
        np.testing.assert_allclose(frame_from_elongators(reference, tcp, z, x), tool, atol=1e-12)

    def test_abb_tcp_z_x_rejects_collinear_axes(self):
        tcp = np.zeros(3)
        reference = np.eye(3)
        pivot = np.zeros(3)
        sample = (np.eye(3), np.array([0.0, 0.0, 0.1]))
        with self.assertRaises(ValueError):
            frame_from_elongators((reference, pivot), tcp, sample, sample)


if __name__ == '__main__':
    unittest.main()
