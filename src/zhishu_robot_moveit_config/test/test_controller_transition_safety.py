from pathlib import Path

import yaml


PACKAGE_ROOT = Path(__file__).resolve().parents[1]


def test_arm_trajectory_transitions_start_from_measured_state():
    config = yaml.safe_load(
        (PACKAGE_ROOT / "config" / "ros2_controllers.yaml").read_text(
            encoding="utf-8"
        )
    )

    for controller in (
        "left_arm_controller",
        "right_arm_controller",
        "dual_arm_controller",
    ):
        params = config[controller]["ros__parameters"]
        assert params["interpolate_from_desired_state"] is False

    dual_params = config["dual_arm_controller"]["ros__parameters"]
    assert (
        dual_params["set_last_command_interface_value_as_state_on_activation"]
        is False
    )
