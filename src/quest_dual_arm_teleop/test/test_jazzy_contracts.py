from pathlib import Path

import yaml


PACKAGE_ROOT = Path(__file__).resolve().parents[1]


def test_servo_nodes_receive_kinematics():
    launch_text = (
        PACKAGE_ROOT / "launch" / "quest_dual_arm_teleop.launch.py"
    ).read_text(encoding="utf-8")
    # One builder call plus one parameter entry for each Servo node.
    assert launch_text.count("robot_description_kinematics") >= 3
    assert "start_servo" not in launch_text


def test_collision_thresholds_are_not_humble_legacy_values():
    for arm in ("left", "right"):
        config = yaml.safe_load(
            (PACKAGE_ROOT / "config" / f"{arm}_servo.yaml").read_text(
                encoding="utf-8"
            )
        )
        assert config["check_collisions"] is True
        assert config["self_collision_proximity_threshold"] <= 0.01
        assert config["scene_collision_proximity_threshold"] <= 0.02


def test_smoke_tests_use_named_command_constants():
    joint_script = (
        PACKAGE_ROOT / "scripts" / "servo_joint_smoke_test.py"
    ).read_text(encoding="utf-8")
    twist_script = (
        PACKAGE_ROOT / "scripts" / "servo_smoke_test.py"
    ).read_text(encoding="utf-8")
    assert "ServoCommandType.Request.JOINT_JOG" in joint_script
    assert "ServoCommandType.Request.TWIST" in twist_script
