#!/usr/bin/env python3
"""Forward kinematics helper for the DH-oriented Zhishu robot arms.

The MDH convention used here is:

    T_i = Rx(alpha_i) * Tx(a_i) * Rz(theta_offset_i + q_i) * Tz(d_i)

The total transform from waist_pitch_link to the hand tool frame is:

    T = T_waist_to_arm_dh_base * T_1 * ... * T_7 * T_wrist_roll_to_hand

Run example:

    python3 scripts/dh_forward_kinematics.py --side left --q 0 0 0 0 0 0 0
"""

import argparse
import csv
import math
from pathlib import Path

import numpy as np


def rx(a):
    c, s = math.cos(a), math.sin(a)
    T = np.eye(4)
    T[:3, :3] = [[1, 0, 0], [0, c, -s], [0, s, c]]
    return T


def rz(a):
    c, s = math.cos(a), math.sin(a)
    T = np.eye(4)
    T[:3, :3] = [[c, -s, 0], [s, c, 0], [0, 0, 1]]
    return T


def tx(a):
    T = np.eye(4)
    T[0, 3] = a
    return T


def tz(d):
    T = np.eye(4)
    T[2, 3] = d
    return T


def rpy_to_matrix(roll, pitch, yaw):
    return rz(yaw) @ _ry(pitch) @ rx(roll)


def _ry(a):
    c, s = math.cos(a), math.sin(a)
    T = np.eye(4)
    T[:3, :3] = [[c, 0, s], [0, 1, 0], [-s, 0, c]]
    return T


def xyz_rpy_to_matrix(xyz, rpy):
    T = rpy_to_matrix(*rpy)
    T[:3, 3] = xyz
    return T


def load_base_transform(config_dir, side):
    path = config_dir / 'arm_dh_base_transforms.csv'
    with path.open(newline='') as f:
        for row in csv.DictReader(f):
            if row['side'] == side:
                xyz = [float(row['xyz_x_m']), float(row['xyz_y_m']), float(row['xyz_z_m'])]
                rpy = [float(row['rpy_roll_rad']), float(row['rpy_pitch_rad']), float(row['rpy_yaw_rad'])]
                return xyz_rpy_to_matrix(xyz, rpy)
    raise RuntimeError(f'No base transform for side={side}')


def load_tool_transform(config_dir, side):
    path = config_dir / 'arm_dh_tool_transforms.csv'
    with path.open(newline='') as f:
        for row in csv.DictReader(f):
            if row['side'] == side:
                xyz = [float(v) for v in row['xyz_m'].split()]
                rpy = [float(v) for v in row['rpy_rad'].split()]
                return xyz_rpy_to_matrix(xyz, rpy)
    raise RuntimeError(f'No tool transform for side={side}')


def load_mdh_rows(config_dir, side):
    path = config_dir / 'arm_dh_mdh_parameters.csv'
    rows = []
    with path.open(newline='') as f:
        for row in csv.DictReader(f):
            if row['side'] == side:
                rows.append(row)
    rows.sort(key=lambda r: int(r['index']))
    return rows


def forward_kinematics(config_dir, side, q):
    rows = load_mdh_rows(config_dir, side)
    if len(q) != len(rows):
        raise ValueError(f'{side} arm expects {len(rows)} joint values, got {len(q)}')

    T = load_base_transform(config_dir, side)
    for row, qi in zip(rows, q):
        a = float(row['a_m'])
        alpha = float(row['alpha_rad'])
        d = float(row['d_m'])
        theta_offset = float(row['theta_offset_rad'])
        T = T @ rx(alpha) @ tx(a) @ rz(theta_offset + qi) @ tz(d)
    T = T @ load_tool_transform(config_dir, side)
    return T


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--side', choices=['left', 'right'], required=True)
    parser.add_argument('--q', type=float, nargs=7, required=True, help='Seven joint values in radians')
    parser.add_argument('--config-dir', default=str(Path(__file__).resolve().parents[1] / 'config'))
    args = parser.parse_args()

    T = forward_kinematics(Path(args.config_dir), args.side, args.q)
    np.set_printoptions(precision=6, suppress=True)
    print(T)


if __name__ == '__main__':
    main()
