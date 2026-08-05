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


def test_trigger_toggles_both_gripper_controllers_on_press_edge():
    source = (
        PACKAGE_ROOT / "src" / "quest_dual_arm_teleop_node.cpp"
    ).read_text(encoding="utf-8")
    launch_text = (
        PACKAGE_ROOT / "launch" / "quest_dual_arm_teleop.launch.py"
    ).read_text(encoding="utf-8")

    assert "trigger_press_threshold" in source
    assert "trigger_release_threshold" in source
    assert "state.closed = !state.closed" in source
    assert "state.trigger_pressed" in source
    assert source.count("processGripper(") >= 3
    assert "left_gripper_controller" in launch_text
    assert "right_gripper_controller" in launch_text

    expected = {
        "left": "left_gripper_left_finger_joint",
        "right": "gripper_left_finger_joint",
    }
    for side, joint in expected.items():
        config = yaml.safe_load(
            (PACKAGE_ROOT / "config" / f"{side}_gripper_controller.yaml").read_text(
                encoding="utf-8"
            )
        )
        assert config["/**"]["ros__parameters"]["joints"] == [joint]
