#!/usr/bin/env python3

import argparse
import csv
import math
from collections import defaultdict


class MotorStats:
    def __init__(self, moving_velocity_threshold_raw=10):
        self.moving_velocity_threshold_raw = moving_velocity_threshold_raw
        self.rows = 0
        self.stable_rows = 0
        self.moving_rows = 0
        self.first_ns = None
        self.last_ns = None
        self.target_min = None
        self.target_max = None
        self.actual_min = None
        self.actual_max = None
        self.last_target = None
        self.last_actual = None
        self.last_goal_sequence = None
        self.target_changes = 0
        self.actual_changes = 0
        self.pdo_repeat_cycles = 0
        self.pdo_multi_update_cycles = 0
        self.skipped_goal_updates = 0
        self.errors = []
        self.position_errors_rad = []
        self.target_steps = []
        self.period_jitter_ns = []
        self.wake_lateness_ns = []
        self.execution_ns = []
        self.deadline_miss_total = 0
        self.dropped_cycles_total = 0
        self.wc_incomplete_cycles = 0
        self.wc_regression_cycles = 0
        self.seen_wc_complete = False
        self.velocity_abs_max_raw = 0
        self.torque_abs_max_raw = 0
        self.moving_velocity_abs_max_raw = 0
        self.moving_torque_abs_max_raw = 0
        self.moving_velocity_abs_max_rad_s = None
        self.moving_effort_abs_max_nm = None
        self.status_words = set()
        self.control_words = set()
        self.error_codes = set()

    def add(self, row):
        timestamp_ns = int(row["monotonic_ns"])
        target = int(row["target_position_607a"])
        actual = int(row["actual_position_6064"])
        goal_sequence = int(row["goal_update_sequence"])
        actual_velocity = int(row.get("actual_velocity_606c_raw", 0))
        actual_torque = int(row.get("actual_torque_6077_raw", 0))
        joint_velocity = row.get("joint_velocity_rad_s")
        joint_effort = row.get("joint_effort_nm")
        position_error_rad = row.get("position_error_joint_rad")
        joint_velocity = (
            float(joint_velocity) if joint_velocity not in (None, "") else None
        )
        joint_effort = float(joint_effort) if joint_effort not in (None, "") else None
        if position_error_rad not in (None, ""):
            self.position_errors_rad.append(abs(float(position_error_rad)))
        target_delta = int(
            row.get(
                "target_delta_counts",
                0 if self.last_target is None else target - self.last_target,
            )
        )

        self.period_jitter_ns.append(abs(int(row.get("period_jitter_ns", 0))))
        self.wake_lateness_ns.append(max(0, int(row.get("wake_lateness_ns", 0))))
        self.execution_ns.append(max(0, int(row.get("execution_ns", 0))))
        self.deadline_miss_total = max(
            self.deadline_miss_total, int(row.get("deadline_miss_total", 0))
        )
        self.dropped_cycles_total = max(
            self.dropped_cycles_total, int(row.get("dropped_cycles_total", 0))
        )
        wc_complete = int(row.get("domain_wc_state", 2)) == 2
        if wc_complete:
            self.seen_wc_complete = True
        else:
            self.wc_incomplete_cycles += 1
            if self.seen_wc_complete:
                self.wc_regression_cycles += 1
        self.velocity_abs_max_raw = max(
            self.velocity_abs_max_raw,
            abs(actual_velocity),
        )
        self.torque_abs_max_raw = max(
            self.torque_abs_max_raw,
            abs(actual_torque),
        )
        status_word_text = row.get("status_word_6041")
        status_word = int(status_word_text, 0) if status_word_text is not None else None
        if status_word is not None:
            self.status_words.add(status_word)
        operation_enabled = status_word is None or (status_word & 0x006F) == 0x0027
        if wc_complete and operation_enabled:
            self.stable_rows += 1
            moving = (
                target_delta != 0
                or abs(actual_velocity) >= self.moving_velocity_threshold_raw
            )
            if moving:
                self.moving_rows += 1
                self.moving_velocity_abs_max_raw = max(
                    self.moving_velocity_abs_max_raw, abs(actual_velocity)
                )
                self.moving_torque_abs_max_raw = max(
                    self.moving_torque_abs_max_raw, abs(actual_torque)
                )
                if joint_velocity is not None:
                    self.moving_velocity_abs_max_rad_s = max(
                        self.moving_velocity_abs_max_rad_s or 0.0,
                        abs(joint_velocity),
                    )
                if joint_effort is not None:
                    self.moving_effort_abs_max_nm = max(
                        self.moving_effort_abs_max_nm or 0.0,
                        abs(joint_effort),
                    )
        if "control_word_6040" in row:
            self.control_words.add(int(row["control_word_6040"], 0))
        if int(row.get("error_code_valid", 0)):
            error_code = int(row.get("error_code_603f", "0"), 0)
            if error_code:
                self.error_codes.add(error_code)

        self.rows += 1
        self.first_ns = timestamp_ns if self.first_ns is None else self.first_ns
        self.last_ns = timestamp_ns
        self.target_min = target if self.target_min is None else min(self.target_min, target)
        self.target_max = target if self.target_max is None else max(self.target_max, target)
        self.actual_min = actual if self.actual_min is None else min(self.actual_min, actual)
        self.actual_max = actual if self.actual_max is None else max(self.actual_max, actual)
        if self.last_target is not None and target != self.last_target:
            self.target_changes += 1
        if self.last_target is not None:
            self.target_steps.append(abs(target - self.last_target))
        if self.last_actual is not None and actual != self.last_actual:
            self.actual_changes += 1
        if self.last_goal_sequence is not None:
            sequence_delta = goal_sequence - self.last_goal_sequence
            if sequence_delta == 0:
                self.pdo_repeat_cycles += 1
            elif sequence_delta > 1:
                self.pdo_multi_update_cycles += 1
                self.skipped_goal_updates += sequence_delta - 1
        self.last_target = target
        self.last_actual = actual
        self.last_goal_sequence = goal_sequence
        self.errors.append(abs(target - actual))

    @staticmethod
    def percentile(values, fraction):
        if not values:
            return 0
        ordered = sorted(values)
        index = min(len(ordered) - 1, math.ceil(fraction * len(ordered)) - 1)
        return ordered[index]

    def summary(self):
        elapsed = 0.0
        if self.first_ns is not None and self.last_ns is not None:
            elapsed = (self.last_ns - self.first_ns) * 1e-9
        target_span = self.target_max - self.target_min
        actual_span = self.actual_max - self.actual_min
        target_change_ratio = self.target_changes / max(1, self.rows - 1)
        actual_change_ratio = self.actual_changes / max(1, self.rows - 1)
        repeat_ratio = self.pdo_repeat_cycles / max(1, self.rows - 1)

        if self.wc_regression_cycles > 0:
            verdict = "EtherCAT 进入 COMPLETE 后发生 WC 回退：优先检查链路和从站状态"
        elif self.deadline_miss_total > 0:
            verdict = "检测到 1 kHz deadline miss：主机实时调度可能参与抖动"
        elif self.skipped_goal_updates > 0:
            verdict = (
                "检测到 1 kHz 命令被覆盖/跳过：ros2_control 与 EtherCAT "
                "周期相位可能参与抖动"
            )
        elif target_span <= 2 and actual_span >= 5:
            verdict = "目标基本静止但实际位置波动：优先检查驱动器伺服参数/机械系统"
        elif self.stable_rows > 0 and self.moving_rows < 10:
            verdict = "未捕获明确运动阶段：请将 PDO 记录与 MOVE_START 同步"
        elif target_change_ratio > 0.01:
            verdict = "目标本身持续变化：需结合轨迹阶段检查 ros2_control/驱动下发"
        else:
            verdict = "波动很小或样本不足：建议延长记录并覆盖明显抖动时段"

        return (
            elapsed,
            target_span,
            actual_span,
            target_change_ratio,
            actual_change_ratio,
            repeat_ratio,
            self.pdo_multi_update_cycles,
            self.skipped_goal_updates,
            self.percentile(self.target_steps, 0.95),
            max(self.target_steps, default=0),
            self.percentile(self.errors, 0.95),
            max(self.errors, default=0),
            self.percentile(self.period_jitter_ns, 0.99) / 1000.0,
            self.percentile(self.wake_lateness_ns, 0.99) / 1000.0,
            self.percentile(self.execution_ns, 0.99) / 1000.0,
            verdict,
        )


def rows_from_file(path):
    with open(path, "r", encoding="utf-8", newline="") as stream:
        data_lines = (line for line in stream if not line.startswith("#"))
        yield from csv.DictReader(data_lines)


def main():
    parser = argparse.ArgumentParser(
        description="Summarize exact EtherCAT PDO diagnostic CSV files."
    )
    parser.add_argument("csv", nargs="+", help="one or more pdo_master*.csv files")
    parser.add_argument(
        "--moving-velocity-threshold-raw",
        type=int,
        default=10,
        help="absolute 0x606C raw threshold used to classify motion (default: 10)",
    )
    args = parser.parse_args()
    if args.moving_velocity_threshold_raw < 0:
        parser.error("--moving-velocity-threshold-raw must be non-negative")

    stats = defaultdict(
        lambda: MotorStats(args.moving_velocity_threshold_raw)
    )
    for path in args.csv:
        for row in rows_from_file(path):
            key = (int(row["master_index"]), row["motor_name"])
            stats[key].add(row)

    print(
        "master motor samples seconds target_span actual_span target_change% "
        "actual_change% repeat% multi_update_cycles skipped_updates "
        "target_step_p95 target_step_max error_p95 error_max "
        "position_error_p95_rad position_error_max_rad "
        "jitter_p99_us wake_late_p99_us execution_p99_us deadline_misses "
        "dropped wc_startup_incomplete wc_regressions stable_samples moving_samples moving% "
        "velocity_abs_max_raw torque_abs_max_raw moving_velocity_abs_max_raw "
        "moving_torque_abs_max_raw moving_velocity_abs_max_rad_s "
        "moving_effort_abs_max_nm "
        "status_words control_words error_codes verdict"
    )
    for (master, motor), values in sorted(stats.items()):
        (
            elapsed,
            target_span,
            actual_span,
            target_change_ratio,
            actual_change_ratio,
            repeat_ratio,
            multi_update_cycles,
            skipped_goal_updates,
            target_step_p95,
            target_step_max,
            error_p95,
            error_max,
            jitter_p99_us,
            wake_late_p99_us,
            execution_p99_us,
            verdict,
        ) = values.summary()
        moving_velocity_si = (
            "-"
            if values.moving_velocity_abs_max_rad_s is None
            else f"{values.moving_velocity_abs_max_rad_s:.6f}"
        )
        moving_effort_si = (
            "-"
            if values.moving_effort_abs_max_nm is None
            else f"{values.moving_effort_abs_max_nm:.6f}"
        )
        position_error_p95_rad = (
            "-"
            if not values.position_errors_rad
            else f"{values.percentile(values.position_errors_rad, 0.95):.9f}"
        )
        position_error_max_rad = (
            "-"
            if not values.position_errors_rad
            else f"{max(values.position_errors_rad):.9f}"
        )
        print(
            f"{master} {motor} {values.rows} {elapsed:.3f} "
            f"{target_span} {actual_span} "
            f"{target_change_ratio * 100:.2f} {actual_change_ratio * 100:.2f} "
            f"{repeat_ratio * 100:.2f} {multi_update_cycles} "
            f"{skipped_goal_updates} {target_step_p95} {target_step_max} "
            f"{error_p95} {error_max} "
            f"{position_error_p95_rad} {position_error_max_rad} "
            f"{jitter_p99_us:.3f} "
            f"{wake_late_p99_us:.3f} {execution_p99_us:.3f} "
            f"{values.deadline_miss_total} {values.dropped_cycles_total} "
            f"{values.wc_incomplete_cycles} {values.wc_regression_cycles} "
            f"{values.stable_rows} {values.moving_rows} "
            f"{values.moving_rows / max(1, values.stable_rows) * 100:.2f} "
            f"{values.velocity_abs_max_raw} "
            f"{values.torque_abs_max_raw} "
            f"{values.moving_velocity_abs_max_raw} "
            f"{values.moving_torque_abs_max_raw} "
            f"{moving_velocity_si} {moving_effort_si} "
            f"{','.join(f'0x{x:04X}' for x in sorted(values.status_words)) or '-'} "
            f"{','.join(f'0x{x:04X}' for x in sorted(values.control_words)) or '-'} "
            f"{','.join(f'0x{x:04X}' for x in sorted(values.error_codes)) or '-'} "
            f"{verdict}"
        )


if __name__ == "__main__":
    main()
