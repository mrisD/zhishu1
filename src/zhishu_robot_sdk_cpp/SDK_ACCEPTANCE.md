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
