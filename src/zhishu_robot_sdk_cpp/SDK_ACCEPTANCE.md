# C++ SDK Acceptance Checklist

This procedure validates the package build, registered unit tests and shared-library packaging. It does **not** prove real-robot safety or complete Excel workflow functionality.

## Automated acceptance

From the workspace root on Ubuntu 24.04 with ROS 2 Jazzy and package dependencies installed:

```bash
./src/zhishu_robot_sdk_cpp/scripts/validate_sdk_acceptance.sh "$PWD"
```

The script builds the package and its workspace dependencies, runs registered gtests, checks test results, verifies `libzhishu_robot_sdk.so`, checks that `ldd` has no `not found` entries, and confirms a public `RobotSdk` symbol is exported. It does not command robot motion.

## Manual Fake Hardware smoke test

The package installs `test/commander_fake_validation.launch.py` plus its YAML fixtures. Run it only in an isolated ROS domain; it is configured to refuse any hardware plugin other than `mock_components/GenericSystem`.

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch zhishu_robot_sdk_cpp test/commander_fake_validation.launch.py
```

Verify that `robot_state_publisher`, `ros2_control_node`, the controller spawners, `move_group`, and the SDK Commander start without startup errors. This launch does not send motion automatically. Stop with Ctrl-C after checking startup logs. If the installed launch file cannot resolve its fixture YAMLs or any node fails, record the complete launch output and treat the smoke test as failed.

## Required checks before real hardware

- [ ] Automated acceptance script passes.
- [ ] Fake Hardware smoke test passes with only `GenericSystem`.
- [ ] Robot group names, joint order, TCP frames and joint limits are confirmed against the deployed URDF/SRDF.
- [ ] E-stop, motor-power isolation and recovery procedures are available.
- [ ] One operator supervises the first low-speed motion test in a cleared workspace.
- [ ] Stability-loop examples are not run against real hardware without a separately approved test plan.

## Known feature status

The following public APIs remain placeholders and return `kNotReady`: software Jog/teaching, trajectory sampling and JSON persistence, trajectory playback, scan Topic/Service monitoring, user Home persistence, speed-scale setter/getter, coordinate-frame CRUD, IO, controller version/error mapping, and vacuum. A passing build/test gate does not mean these features are implemented.

## Concurrency and feedback regressions

The registered `test_motion_task` tests exercise queued cancellation, completion while a stop
callback is still running, stop-callback failure, and backend exceptions. A task becomes
terminal only after the operation and its pending stop callback have finished. Failed stop
requests return false and do not forge a cancelled result.

With `BUILD_TESTING=ON`, a separate `sdk_simulation_regression` executable is built. After
starting only GenericSystem in an isolated domain, run:

```bash
ROS_DOMAIN_ID=169 ROS_LOCALHOST_ONLY=1 ros2 launch zhishu_robot_sdk_cpp sdk_simulation_regression.launch.py
```

This checks rejected inputs, 800 concurrent sync/async calls with a nonexistent named target,
and required-joint feedback when one optional TCP is unavailable. It intentionally sends no
valid motion target and is separate from hardware-independent CTest. Follow it with the joint
loop in the same GenericSystem domain for motion regression.

SDK motion admission and worker reaping are serialized. Busy calls still return `kBusy`;
accepted synchronous motion does not hold the admission lock while executing. Gripper targets
outside `[0, 0.06]` metres, including NaN/infinity, are rejected before worker creation. Optional
feedback failures only clear their own validity flags; required joint feedback retains its
separate result. State reads retain the existing Commander locking behavior and are not a
guarantee of a simultaneous or real-time snapshot.
