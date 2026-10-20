import pytest
from types import SimpleNamespace

from zhishu_robot_admittance.validation_runner import ValidationRunner, trapezoid_position
from zhishu_robot_admittance import validation_runner


def test_trapezoid_profile_reaches_requested_distance():
    duration = 10.5
    ramp = 0.5
    peak = 0.2 / (duration - ramp)

    assert trapezoid_position(0.0, duration, peak, ramp) == 0.0
    assert trapezoid_position(duration, duration, peak, ramp) == pytest.approx(0.2)


def test_arm_and_waist_profiles_remain_synchronized():
    duration = 10.5
    ramp = 0.5
    arm_peak = 0.04 / (duration - ramp)
    waist_peak = 0.16 / (duration - ramp)

    for elapsed in (0.1, 0.5, 3.0, 10.0, 10.5):
        combined = trapezoid_position(elapsed, duration, arm_peak, ramp)
        combined += trapezoid_position(elapsed, duration, waist_peak, ramp)
        expected = trapezoid_position(elapsed, duration, 0.02, ramp)
        assert combined == pytest.approx(expected)


def test_waist_profile_starts_at_current_lift():
    sent = []
    runner = SimpleNamespace(current_waist_pitch=-0.1, current_waist_lift=0.25,
                             body_publisher=SimpleNamespace(
                                 get_subscription_count=lambda: 1,
                                 publish=sent.append))
    ValidationRunner.publish_waist_profile(runner, 2.5, 0.02)
    assert sent[0].points[0].positions[0] >= 0.25
    assert sent[0].points[-1].positions[0] == pytest.approx(0.27)
    assert all(point.positions[1] == -0.1 for point in sent[0].points)


def test_waist_profile_rejects_absolute_limit_violation():
    runner = SimpleNamespace(current_waist_pitch=-0.1, current_waist_lift=0.25)
    with pytest.raises(RuntimeError, match='limit'):
        ValidationRunner.publish_waist_profile(runner, 2.5, 0.1)


def test_arm_only_profile_does_not_command_waist():
    ValidationRunner.publish_waist_profile(SimpleNamespace(), 2.5, 0.0)


def test_slow_but_advancing_simulation_does_not_time_out(monkeypatch):
    wall = [0.0]
    sent = []
    runner = SimpleNamespace(sim_time=0.0,
                             publisher=SimpleNamespace(publish=sent.append),
                             _sample_tcp_distance=lambda: None)

    def spin(node, timeout_sec):
        wall[0] += 10.0
        node.sim_time += 0.01

    monkeypatch.setattr(validation_runner.rclpy, 'spin_once', spin)
    monkeypatch.setattr(validation_runner.time, 'monotonic', lambda: wall[0])
    ValidationRunner.publish_profile(runner, 0.03)
    assert wall[0] >= 30.0
    assert sent


def test_stopped_simulation_clock_times_out(monkeypatch):
    wall = [0.0]
    runner = SimpleNamespace(sim_time=0.0)

    def spin(node, timeout_sec):
        wall[0] = 6.0

    monkeypatch.setattr(validation_runner.rclpy, 'spin_once', spin)
    monkeypatch.setattr(validation_runner.time, 'monotonic', lambda: wall[0])
    with pytest.raises(RuntimeError, match='did not advance'):
        ValidationRunner.publish_profile(runner, 1.0)


def test_measured_distance_stops_without_waiting_for_open_loop_duration(monkeypatch):
    runner = SimpleNamespace(sim_time=0.0)
    pose = SimpleNamespace(x=0.0, y=0.0, z=0.050, sample_time=0.0)
    start = SimpleNamespace(x=0.0, y=0.0, z=0.0)
    monkeypatch.setattr(validation_runner.rclpy, 'spin_once', lambda node, timeout_sec: None)
    monkeypatch.setattr(validation_runner.time, 'monotonic', lambda: 0.0)
    ValidationRunner.publish_profile(
        runner, 10.5, axis='z', peak=0.005,
        pose_tracker=SimpleNamespace(latest=lambda: pose), line_start=start,
        measured_distance_target=0.050)
