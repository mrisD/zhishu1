import pytest

from zhishu_robot_admittance.validation_runner import trapezoid_position


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
