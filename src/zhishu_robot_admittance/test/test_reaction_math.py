import math
import pytest
from zhishu_robot_admittance.reaction_math import bottom_reaction


def test_free_carried_box_has_zero_floor_reaction():
    assert bottom_reaction(-100.0, -(15.56 * 9.81 - 100.0)) == pytest.approx(0.0)


def test_fully_supported_box_leaves_only_tool_weight_on_wrists():
    assert bottom_reaction(-0.28 * 9.81, -0.28 * 9.81) == pytest.approx(15.0 * 9.81)


def test_partial_support_and_unequal_load_sharing():
    assert bottom_reaction(20.0, -100.0) == pytest.approx(15.56 * 9.81 - 80.0)


def test_negative_estimate_is_not_hidden_by_clamping():
    assert bottom_reaction(-200.0, -100.0) < 0.0


@pytest.mark.parametrize('value', [math.nan, math.inf, -math.inf])
def test_invalid_sensor_force_rejected(value):
    with pytest.raises(ValueError):
        bottom_reaction(value, 0.0)
