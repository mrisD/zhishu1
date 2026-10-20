#!/usr/bin/env python3
"""Reject enabled-only logs: require completed process stages and commanded corrections."""
import argparse
import json
import re
from pathlib import Path


def audit(content, target, row):
    ends = []
    for match in re.finditer(r'PROCESS_ADMITTANCE_END stage=(.*?) cycles=(\d+) max_abs_offset=([\d.eE+-]+) corrected_points=(\d+) projected_points=(\d+)', content):
        ends.append(dict(stage=match[1], cycles=int(match[2]), max_abs_offset=float(match[3]),
                         corrected_points=int(match[4]), projected_points=int(match[5])))
    pull = ('physical side push without attachment' if row == 1 else
            ('left-arm mirrored pre-extraction ' if row == 2 else 'right-arm pre-extraction ') + target)
    required = ['empty long-finger coordinated approach', pull,
                'full-transfer pick ' + target, 'coordinated level tote posture',
                'large-bin insertion ' + target]
    missing = [stage for stage in required if not any(e['stage'] == stage and e['cycles'] > 0 for e in ends)]
    corrected = [e['stage'] for e in ends if e['corrected_points'] > 0 and e['max_abs_offset'] > 1e-8]
    carry = 'PROCESS_ADMITTANCE_CARRY_SAMPLE' in content
    support = 'Floor support verified BEFORE opening' in content
    placed = 'Physical large-bin placement verified' in content
    failed = '[FATAL]' in content or 'PROCESS_ADMITTANCE_CARRY_STOP' in content
    return dict(passed=not missing and bool(corrected) and carry and support and placed and not failed,
                missing_stages=missing, completed_stages=ends, corrected_stages=corrected,
                carry_feedback_observed=carry, floor_support_before_release=support,
                physical_placement_verified=placed, failure_observed=failed,
                limitation='Functional simulation evidence only; no controlled external-disturbance robustness claim')


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('log', type=Path)
    parser.add_argument('--target', required=True)
    parser.add_argument('--row', type=int, choices=(0, 1, 2), required=True)
    args = parser.parse_args()
    result = audit(args.log.read_text(errors='replace'), args.target, args.row)
    print(json.dumps(result, ensure_ascii=False, indent=2))
    raise SystemExit(0 if result['passed'] else 1)
