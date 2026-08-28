import math

import pytest

from zhishu_robot_admittance.admittance_math import admittance_step
from zhishu_robot_admittance.admittance_math import clamp
from zhishu_robot_admittance.admittance_math import cross
from zhishu_robot_admittance.admittance_math import rotate_vector


def test_cross_product():
    assert cross([1.0, 0.0, 0.0], [0.0, 1.0, 0.0]) == [0.0, 0.0, 1.0]


def test_quaternion_rotation():
    half = 0.5 * math.pi / 2.0
    result = rotate_vector([1.0, 0.0, 0.0], [0.0, 0.0, math.sin(half), math.cos(half)])
    assert abs(result[0]) < 1e-9
    assert abs(result[1] - 1.0) < 1e-9
    assert abs(result[2]) < 1e-9


def test_admittance_accelerates_in_force_direction():
    position, velocity = admittance_step(10.0, 5.0, 20.0, 0.0, 0.0, 0.0, 0.01)
    assert position > 0.0
    assert velocity > 0.0


@pytest.mark.parametrize("value", [math.nan, math.inf, -math.inf])
def test_clamp_rejects_non_finite_values(value):
    with pytest.raises(ValueError):
        clamp(value, -1.0, 1.0)


def test_rotation_rejects_non_finite_input():
    with pytest.raises(ValueError):
        rotate_vector([math.nan, 0.0, 0.0], [0.0, 0.0, 0.0, 1.0])
