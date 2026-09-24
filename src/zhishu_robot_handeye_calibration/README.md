# Zhishu robot Eye-to-Hand calibration

This package simulates calibration between the movable robot arm and the RGB
camera mounted under `head_camera_link`.  During sample collection the head is
treated as fixed relative to `base_link`.

## Real Gemini Pro camera preparation

The legacy Gemini Pro is driven by the official `orbbec_camera` ROS 2 wrapper.
The launch publishes `640x480@30` RGB and registered depth, hardware D2C, a
depth point cloud, and a colored point cloud under `/camera_head`.  RGB uses the
robot-owned `head_camera_optical_frame`, so the camera driver does not publish a
duplicate TF.  An interactive ChArUco intrinsic calibration tool writes a
standard CameraInfo YAML and a separate checker validates that the image,
calibration, resolution, timestamps, and frame ID agree.

See [`docs/GEMINI_PRO_CAMERA.zh-CN.md`](docs/GEMINI_PRO_CAMERA.zh-CN.md) for the
camera-only installation, launch, calibration, and verification procedure.  It
does not start robot hardware or motion planning.

### Gemini Pro RGB intrinsic calibration

Start the camera in the first terminal:

```bash
cd /home/zhul/online/guohua_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch zhishu_robot_handeye_calibration gemini_pro_camera.launch.py
```

Run the interactive ChArUco calibration tool in a second terminal:

```bash
cd /home/zhul/online/guohua_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 run zhishu_robot_handeye_calibration calibrate_charuco_intrinsics.py
```

Use `Space` to capture a valid view, `R` to remove the latest view, `C` to
calculate without saving, `S` to calculate and save, and `Q` or `Esc` to quit.
Collect 30--50 sharp views covering the image center, corners, edges, several
distances, and different board tilts.

The calibration tool writes its working result and quality report to
`~/.ros/camera_info/`.  The production camera launch deliberately loads the
version-controlled calibration from
`config/gemini_pro_rgb_640x480.yaml` through the package share directory.  After
accepting a new calibration, copy the generated CameraInfo YAML into that
source path and rebuild this package:

```bash
cp ~/.ros/camera_info/gemini_pro_rgb_640x480.yaml \
  src/zhishu_robot_handeye_calibration/config/gemini_pro_rgb_640x480.yaml
colcon build --symlink-install --packages-select zhishu_robot_handeye_calibration
source install/setup.bash
```

The checked-in calibration was produced from 50 samples at `640x480`; its RMS
reprojection error is `0.2375 px`.

## Fully automatic simulation and validation

Install declared system dependencies, then build the package and its workspace
dependencies.  The manifest includes the MoveIt hand-eye GUI, Eigen, OpenCV,
Python OpenCV, and RViz visualization dependencies used by this package:

```bash
cd /home/zhul/online/guohua_ws
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install --packages-up-to zhishu_robot_handeye_calibration
source install/setup.bash
```

Run the complete headless pipeline:

```bash
ros2 launch zhishu_robot_handeye_calibration handeye_auto.launch.py
```

This single command starts Gazebo and MoveIt, executes all 12 collision-checked
poses, detects the ArUco board from the rendered camera images, runs every
OpenCV hand-eye solver, validates the selected result against the simulation
truth, writes `/tmp/handeye_calibration_result.yaml` plus the native MoveIt
sample file `/tmp/handeye_moveit_samples.yaml`, and then stops the launch.
The calibration node returns success only when at least five image samples are
valid and both the truth and AX=XB consistency errors are within their configured
tolerances. For scripts, use the YAML `status` field as the authoritative result;
the enclosing `ros2 launch` process may not preserve a child process exit code.

Useful launch overrides:

```bash
ros2 launch zhishu_robot_handeye_calibration handeye_auto.launch.py \
  output_file:=/tmp/result.yaml \
  image_directory:=/tmp/samples \
  moveit_samples_file:=/tmp/handeye_moveit_samples.yaml \
  minimum_markers:=3 \
  sample_attempts_per_pose:=3 \
  translation_tolerance:=0.03 \
  rotation_tolerance:=0.05 \
  reprojection_translation_tolerance:=0.02 \
  reprojection_rotation_tolerance:=0.03
```

The default of three visible markers keeps all supplied diverse poses. Raise
`minimum_markers` to six when stronger per-frame PnP quality is more important
than retaining the full motion set.

The automatic node checks the head joint lock before calibration and at every
sample, retries transient frame/detection failures up to three times per pose,
and writes a `FAIL` YAML on runtime exceptions whenever the output path is writable.

The Chinese validation guide and the recorded 2026-09-14 result are in
[`docs/AUTOMATIC_VALIDATION.zh-CN.md`](docs/AUTOMATIC_VALIDATION.zh-CN.md).

Simulation truth is read from the coincident
`head_camera_optical_truth` frame. The ordinary
`head_camera_optical_frame` is intentionally excluded from truth validation
because the RViz HandEye Context publishes its current camera-pose guess on
that frame before and after solving.

For a visible automatic run that leaves Gazebo and RViz open, start the
interactive stack first and run `handeye_auto_calibration` in a second terminal.
RViz then shows every accepted target pose on
`/handeye_calibration/sample_markers`, the live count, and the annotated image
on `/handeye_calibration/annotated_image`. When the solve completes, the
automatic node publishes `/handeye_calibration/auto_load_samples`; the patched
MoveIt HandEye panel reloads the native sample file after every accepted pose,
so its own `Pose samples` tree grows during acquisition. It selects the winning
solver, switches to `Calibrate`, solves again, and
shows translation, quaternion, and reprojection error in that panel.

## Interactive simulation

```bash
ros2 launch zhishu_robot_handeye_calibration handeye_gazebo.launch.py
```

Use this command for Gazebo simulation.  The similarly named
`zhishu_robot_bringup zhishu_robot.handeye.launch.py` entry point is for a
physical Gemini Pro camera; it cannot publish an image when no USB camera is
connected.  In that case RViz reports `No Image` and no target transform is
available for `Take sample`.

The launch runs the Gazebo server separately from a GUI-only OGRE client.  This
avoids the Mesa/EGL crash seen when OGRE2 is combined with forced software
rendering.  RViz opens with `HandEye Sample Poses`, `Head Camera RGB`,
`HandEye Target Detection`, `MotionPlanning`, and `HandEyeCalibration`
enabled.

The real-robot `zhishu_robot.handeye.launch.py` also overrides the ordinary
MoveIt RViz layout with this hand-eye layout.  Target and Context values are
therefore restored automatically rather than requiring a manually-added
display.  The plugin subscribes to the configured image topic immediately,
even when the camera publisher appears after RViz starts.

### RK3588 bringup with RViz on a separate PC

Run the robot, camera, MoveIt, and calibration support nodes on the RK3588,
but disable its RViz process:

```bash
ros2 launch zhishu_robot_bringup zhishu_robot.handeye.launch.py use_rviz:=false
```

On the local PC, source a build of the same workspace revision and start only
the hand-eye RViz client:

```bash
ros2 launch zhishu_robot_handeye_calibration handeye_rviz.launch.py
```

This local launch does not start hardware, controllers, a camera driver,
`robot_state_publisher`, or `move_group`. It loads the hand-eye RViz layout and
local planner-selection parameters, then receives the matching robot model
(including the calibration plate on the left flange), joint states, planning
scene, camera image, services, and actions from the RK3588. The Gemini Pro must
be connected to the RK3588 for its hand-eye launch to publish the camera
topics. Both machines must use the same `ROS_DOMAIN_ID`, compatible RMW/DDS
settings, and be able to discover each other on the network.

The launch also keeps a CameraInfo relay alive from
`/camera_head/camera_info` to `/camera_head/color/camera_info`, matching the
topic convention derived by the MoveIt image transport subscriber.

The calibration launch enables the head camera and replaces the rendered left
gripper with the flange-mounted 7x5 ChArUco plate.  The plate is attached
directly to `L_hand_link`; its printed face points along the free `+Z` side of
the wrist flange toward the head camera.  The left-gripper links and joints stay
in the robot model, but their visual and collision geometry is suppressed only
in hand-eye mode so the existing SRDF and ros2_control configuration remain
valid.  Ordinary Gazebo and MoveIt launches keep the complete left gripper.

The simulated carrier is a photo-derived approximation of the supplied plate:
240 x 220 x 3 mm, 0.43 kg, with four 5.5 mm visual alignment holes.  Replace
these dimensions and the hole coordinates with measured CAD values before using
the model for mechanical clearance or load calculations; they do not change the
known printed pattern dimensions.

In the HandEyeCalibration RViz display select Eye-to-Hand and configure:

- sensor frame: `head_camera_optical_frame`
- object frame: `handeye_target`
- end-effector frame: `L_hand_link`
- robot base frame: `base_link`
- image: `/camera_head/color/image_raw`
- camera info used by the RViz plugin: `/camera_head/color/camera_info`
- target type: `HandEyeTarget/Charuco`
- ArUco dictionary: `DICT_4X4_50`
- board: 7 x 5 squares, square length 0.030 m, marker length 0.022 m

The RViz layout contains separate image displays for the raw RGB stream and
the interactive plugin output `/handeye_calibration/target_detection`.
`/handeye_calibration/annotated_image` is kept disabled by default because it
is published only by the automatic calibration executable.

The calibration launch fixes the head at yaw `0.5` rad and pitch `0.15` rad.
Those values must remain unchanged for every sample and for the final solve.

Before sampling, verify the image and marker detector:

```bash
ros2 run zhishu_robot_handeye_calibration check_camera_target.py
```

The command saves an annotated frame to `/tmp/handeye_camera_check.png` and
reports how many markers and interpolated ChArUco corners are visible.  The
supplied pose sequence was generated for the direct flange mounting and keeps
the plate front inside the head-camera view while MoveIt collision checking
remains enabled; the default acceptance threshold is three markers.

Run the interactive, collision-checked sample sequence in another terminal:

```bash
ros2 run zhishu_robot_handeye_calibration handeye_pose_sequence
```

Do not collect a sample if `handeye_head_lock_guard` reports that either head
joint moved outside its 0.002 rad tolerance.

For an SSH-only machine without an active display, use `headless:=true`. The
automatic launch does not require RViz or Xvfb. For the legacy interactive
launch, Xvfb can be used when it is installed:

```bash
xvfb-run -a -s '-screen 0 1280x720x24' \
  ros2 launch zhishu_robot_handeye_calibration handeye_gazebo.launch.py \
  headless:=true use_rviz:=false
```
