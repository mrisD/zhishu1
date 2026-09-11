import importlib.util
from pathlib import Path


SCRIPT = (
    Path(__file__).resolve().parents[1]
    / "scripts"
    / "analyze_pdo_diagnostics.py"
)
SPEC = importlib.util.spec_from_file_location("analyze_pdo_diagnostics", SCRIPT)
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


def row(target, actual, sequence, timestamp=0, **extra):
    result = {
        "monotonic_ns": str(timestamp),
        "target_position_607a": str(target),
        "actual_position_6064": str(actual),
        "goal_update_sequence": str(sequence),
    }
    result.update({key: str(value) for key, value in extra.items()})
    return result


def test_detects_repeated_and_skipped_roscontrol_updates():
    stats = MODULE.MotorStats()
    stats.add(row(100, 100, 10, timestamp=0))
    stats.add(row(100, 101, 10, timestamp=1_000_000))
    stats.add(row(103, 102, 12, timestamp=2_000_000))
    stats.add(row(104, 104, 13, timestamp=3_000_000))

    summary = stats.summary()
    assert stats.pdo_repeat_cycles == 1
    assert stats.pdo_multi_update_cycles == 1
    assert stats.skipped_goal_updates == 1
    assert summary[8] == 3
    assert summary[9] == 3


def test_static_target_with_feedback_motion_points_to_drive_or_mechanics():
    stats = MODULE.MotorStats()
    for index, actual in enumerate((100, 101, 105, 99)):
        stats.add(row(100, actual, index, timestamp=index * 1_000_000))

    assert "驱动器伺服参数/机械系统" in stats.summary()[-1]


def test_collects_timing_pdo_and_drive_health_fields():
    stats = MODULE.MotorStats()
    stats.add(
        row(
            100,
            90,
            1,
            timestamp=1_000_000,
            period_jitter_ns=-20_000,
            wake_lateness_ns=30_000,
            execution_ns=120_000,
            deadline_miss_total=2,
            dropped_cycles_total=3,
            domain_wc_state=1,
            actual_velocity_606c_raw=-45,
            actual_torque_6077_raw=321,
            status_word_6041="0x0027",
            control_word_6040="0x000F",
            error_code_valid=1,
            error_code_603f="0x1234",
        )
    )

    assert stats.period_jitter_ns == [20_000]
    assert stats.deadline_miss_total == 2
    assert stats.dropped_cycles_total == 3
    assert stats.wc_incomplete_cycles == 1
    assert stats.wc_regression_cycles == 0
    assert stats.velocity_abs_max_raw == 45
    assert stats.torque_abs_max_raw == 321
    assert stats.status_words == {0x0027}
    assert stats.control_words == {0x000F}
    assert stats.error_codes == {0x1234}


def test_detects_working_counter_regression_after_complete():
    stats = MODULE.MotorStats()
    stats.add(row(0, 0, 1, domain_wc_state=1))
    stats.add(row(0, 0, 2, domain_wc_state=2))
    stats.add(row(0, 0, 3, domain_wc_state=1))

    assert stats.wc_incomplete_cycles == 2
    assert stats.wc_regression_cycles == 1
    assert "WC 回退" in stats.summary()[-1]
