# Zhishu robot Eye-to-Hand calibration

This package simulates calibration between the movable robot arm and the RGB
camera mounted under `head_camera_link`.  During sample collection the head is
treated as fixed relative to `base_link`.

## Start simulation

```bash
ros2 launch zhishu_robot_handeye_calibration handeye_gazebo.launch.py
```

The calibration launch enables the head camera, attaches the 3x4 ArUco board to
`left_gripper_tcp`, and hides only the rendered mesh of `head_pitch_link`.

In the HandEyeCalibration RViz display select Eye-to-Hand and configure:

- sensor frame: `head_camera_optical_frame`
- object frame: `handeye_target`
- end-effector frame: `left_gripper_tcp`
- robot base frame: `base_link`
- image: `/camera_head/color/image_raw`
- camera info: `/camera_head/camera_info`
- ArUco dictionary: `DICT_5X5_250`
- board: 3 x 4, marker size 0.06 m, separation 0.015 m

The calibration launch fixes the head at yaw `0.5` rad and pitch `0.15` rad.
Those values must remain unchanged for every sample and for the final solve.

Before sampling, verify the image and marker detector:

```bash
ros2 run zhishu_robot_handeye_calibration check_camera_target.py
```

The command saves an annotated frame to `/tmp/handeye_camera_check.png` and
reports how many markers are visible. Six or more is the expected simulation
result across the supplied pose sequence.

Run the interactive, collision-checked sample sequence in another terminal:

```bash
ros2 run zhishu_robot_handeye_calibration handeye_pose_sequence
```

Do not collect a sample if `handeye_head_lock_guard` reports that either head
joint moved outside its 0.002 rad tolerance.

For an SSH-only machine without an active display, start the same simulation
through Xvfb:

```bash
xvfb-run -a -s '-screen 0 1280x720x24' \
  ros2 launch zhishu_robot_handeye_calibration handeye_gazebo.launch.py \
  headless:=true use_rviz:=false
```
