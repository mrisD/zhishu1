import numpy as np

from zhishu_robot_bottle_grasp.tracker import AlphaBetaTracker


def test_constant_velocity_prediction_converges():
    tracker = AlphaBetaTracker(alpha=0.75, beta=0.25)
    for index in range(20):
        tracker.update([0.1 * index, 0.0, 0.5], 0.1 * index)
    prediction = tracker.predict(2.4)
    np.testing.assert_allclose(prediction, [2.4, 0.0, 0.5], atol=0.04)


def test_large_gap_resets_velocity():
    tracker = AlphaBetaTracker(max_gap=0.5)
    tracker.update([0.0, 0.0, 0.0], 0.0)
    tracker.update([1.0, 0.0, 0.0], 1.0)
    np.testing.assert_allclose(tracker.state.velocity, [0.0, 0.0, 0.0])
    assert tracker.state.observations == 1
