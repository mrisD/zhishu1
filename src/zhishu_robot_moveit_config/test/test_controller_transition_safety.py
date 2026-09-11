from pathlib import Path

import yaml


PACKAGE_ROOT = Path(__file__).resolve().parents[1]

ARM_JOINTS = (
    "left_shoulder_pitch_joint",
    "left_shoulder_roll_joint",
    "left_shoulder_yaw_joint",
    "left_elbow_pitch_joint",
    "left_wrist_yaw_joint",
    "left_wrist_pitch_joint",
    "left_wrist_roll_joint",
    "right_shoulder_pitch_joint",
    "right_shoulder_roll_joint",
    "right_shoulder_yaw_joint",
    "right_elbow_pitch_joint",
    "right_wrist_yaw_joint",
    "right_wrist_pitch_joint",
    "right_wrist_roll_joint",
)


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


def test_ruckig_smoothing_runs_after_time_parameterization():
    config = yaml.safe_load(
        (PACKAGE_ROOT / "config" / "ompl_planning.yaml").read_text(
            encoding="utf-8"
        )
    )

    adapters = config["response_adapters"]
    time_parameterization = (
        "default_planning_response_adapters/AddTimeOptimalParameterization"
    )
    ruckig_smoothing = (
        "default_planning_response_adapters/AddRuckigTrajectorySmoothing"
    )

    assert adapters.index(ruckig_smoothing) == adapters.index(time_parameterization) + 1


def test_every_arm_joint_has_a_conservative_jerk_limit():
    config = yaml.safe_load(
        (PACKAGE_ROOT / "config" / "joint_limits.yaml").read_text(
            encoding="utf-8"
        )
    )

    limits = config["joint_limits"]
    for joint in ARM_JOINTS:
        assert limits[joint]["has_jerk_limits"] is True
        assert limits[joint]["max_jerk"] == 1.0
