import numpy as np

from zhishu_robot_bottle_grasp.voxel_map import TemporalVoxelMap


def test_voxels_require_repeated_observations_and_decay():
    model = TemporalVoxelMap(voxel_size=0.1, minimum_hits=2, decay_seconds=1.0)
    points = np.array([[0.01, 0.02, 0.03], [0.04, 0.02, 0.01]])
    assert len(model.update(points, 0.0)) == 0
    occupied = model.update(points, 0.2)
    np.testing.assert_allclose(occupied, [[0.05, 0.05, 0.05]])
    assert len(model.update(np.empty((0, 3)), 1.3)) == 0


def test_clock_rewind_clears_previous_reconstruction():
    model = TemporalVoxelMap(voxel_size=0.1, minimum_hits=1, decay_seconds=10.0)
    assert len(model.update([[0.0, 0.0, 0.0]], 5.0)) == 1
    rewound = model.update([[1.0, 0.0, 0.0]], 1.0)
    np.testing.assert_allclose(rewound, [[1.05, 0.05, 0.05]])
