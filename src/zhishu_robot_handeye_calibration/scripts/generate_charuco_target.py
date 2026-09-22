#!/usr/bin/env python3
"""Generate the Gazebo-safe geometric and PNG forms of the hand-eye target."""

from pathlib import Path

import cv2
import numpy as np


SQUARES_X = 7
SQUARES_Y = 5
SQUARE_LENGTH = 0.030
MARKER_LENGTH = 0.022
WHITE_MARGIN = 0.010
MARKER_BITS = 4
BORDER_BITS = 1


def add_box(lines, index, y, z, size_y, size_z):
    lines.extend(
        [
            f'    <visual name="handeye_black_{index}">',
            f'      <origin xyz="-0.00175 {y:.7f} {z:.7f}" rpy="0 0 0"/>',
            f'      <geometry><box size="0.0005 {size_y:.7f} {size_z:.7f}"/></geometry>',
            '      <material name="handeye_black"><color rgba="0 0 0 1"/></material>',
            '    </visual>',
        ]
    )


def generate_xacro(path, dictionary):
    board_width = SQUARES_X * SQUARE_LENGTH
    board_height = SQUARES_Y * SQUARE_LENGTH
    backing_width = board_width + 2.0 * WHITE_MARGIN
    backing_height = board_height + 2.0 * WHITE_MARGIN
    marker_margin = (SQUARE_LENGTH - MARKER_LENGTH) / 2.0
    marker_cells = MARKER_BITS + 2 * BORDER_BITS
    bit_length = MARKER_LENGTH / marker_cells

    lines = [
        '<?xml version="1.0"?>',
        '<robot xmlns:xacro="http://www.ros.org/wiki/xacro">',
        '  <!-- Generated 7x5 DICT_4X4_50 ChArUco board:',
        '       square_length=0.030 m, marker_length=0.022 m.',
        '       The pattern is mirrored onto local -X because that is the camera-facing side. -->',
        '  <xacro:macro name="handeye_target_pattern">',
        '    <visual name="handeye_board_backing">',
        f'      <geometry><box size="0.003 {backing_width:.7f} {backing_height:.7f}"/></geometry>',
        '      <material name="handeye_white"><color rgba="1 1 1 1"/></material>',
        '    </visual>',
    ]

    visual_index = 0
    marker_id = 0
    for row in range(SQUARES_Y):
        for column in range(SQUARES_X):
            square_left = column * SQUARE_LENGTH
            square_top = row * SQUARE_LENGTH
            if (row + column) % 2 == 0:
                y = board_width / 2.0 - square_left - SQUARE_LENGTH / 2.0
                z = board_height / 2.0 - square_top - SQUARE_LENGTH / 2.0
                add_box(lines, visual_index, y, z, SQUARE_LENGTH, SQUARE_LENGTH)
                visual_index += 1
                continue

            marker = cv2.aruco.drawMarker(
                dictionary, marker_id, marker_cells, borderBits=BORDER_BITS
            )
            marker_id += 1
            for bit_row in range(marker_cells):
                bit_column = 0
                while bit_column < marker_cells:
                    if marker[bit_row, bit_column] != 0:
                        bit_column += 1
                        continue
                    run_start = bit_column
                    while bit_column < marker_cells and marker[bit_row, bit_column] == 0:
                        bit_column += 1
                    run_length = bit_column - run_start
                    offset_y = marker_margin + (run_start + run_length / 2.0) * bit_length
                    offset_z = marker_margin + (bit_row + 0.5) * bit_length
                    y = board_width / 2.0 - square_left - offset_y
                    z = board_height / 2.0 - square_top - offset_z
                    add_box(
                        lines,
                        visual_index,
                        y,
                        z,
                        run_length * bit_length,
                        bit_length,
                    )
                    visual_index += 1

    if marker_id != 17:
        raise RuntimeError(f"expected 17 markers, generated {marker_id}")
    lines.extend(['  </xacro:macro>', '</robot>', ''])
    path.write_text("\n".join(lines), encoding="utf-8")


def generate_png(path, dictionary):
    pixels_per_meter = 10000
    board_width = int(round(SQUARES_X * SQUARE_LENGTH * pixels_per_meter))
    board_height = int(round(SQUARES_Y * SQUARE_LENGTH * pixels_per_meter))
    margin = int(round(WHITE_MARGIN * pixels_per_meter))
    board = cv2.aruco.CharucoBoard_create(
        SQUARES_X,
        SQUARES_Y,
        SQUARE_LENGTH,
        MARKER_LENGTH,
        dictionary,
    )
    pattern = board.draw((board_width, board_height), marginSize=0, borderBits=BORDER_BITS)
    image = np.full((board_height + 2 * margin, board_width + 2 * margin), 255, np.uint8)
    image[margin : margin + board_height, margin : margin + board_width] = pattern
    if not cv2.imwrite(str(path), image):
        raise RuntimeError(f"failed to write {path}")


def main():
    package = Path(__file__).resolve().parents[1]
    source_root = package.parent
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    generate_xacro(
        source_root / "zhishu_robot_description" / "urdf" / "handeye_target_pattern.xacro",
        dictionary,
    )
    generate_png(package / "models" / "handeye_target" / "aruco_grid.png", dictionary)


if __name__ == "__main__":
    main()
