# zhishu_robot_sdk_cpp — Excel API Contract (Skeleton)

This document freezes the public API contract for the 12 unique feature rows in
`AR3双臂功能程序说明.xlsx`. The original `zhishu_robot_commander_cpp` package remains unchanged.

## 1. Connection and disconnection

- `connect()` is intended to check ROS context and MoveIt/controller readiness only; `disconnect()` is intended to release SDK-side connection/resources only.
- Neither API may switch motor power, servo enable, operating mode, or safety state.
- Current status: `connect()` and `disconnect()` remain placeholders returning `ErrorCode::kNotReady`; `isConnected()` remains false.
- `getConnectionStatus().ros_context_available` reports only whether a node and live ROS context are available. It does not mean MoveIt action servers/controllers are ready.
- Until a real backend probe exists, `motion_interface_ready` and `SdkStatus::ready` must remain false. A constructed `Commander` pointer alone is not proof of readiness.

## 2. Software teaching / Jog

- `startDragTeach(..., kSoftwareJog)` enables a future software-teaching session.
- `jogCartesian()` sends one bounded relative XYZABC increment in the robot base frame.
- XYZ use millimetres; ABC use degrees.
- `jogJoints()` accepts seven joint deltas in radians for one arm.
- `stopJog()` stops the software Jog task when implemented.
- `kPhysicalButton` is reserved and is not implemented in this phase.

## 3. Unified dual-arm Home

- `setHomePose()` captures the current left and right end-effector poses (arm tips
  `L_hand_link` / `R_hand_link`) at one instant into a single unified Home record. Both arms
  always come from that one capture and one file.
- `moveHome()` replays the saved record through `moveBothPoses()`. If no unified record exists it
  falls back to the configured SRDF named target `both_arms_home`.
- It must never silently combine one arm's saved pose with the other arm's SRDF pose; a saved but
  unreadable record is reported as an error instead of being replaced by the fallback.
- The record is JSON version 1 with `captured_at` and the `left` / `right` end-effector poses
  (`frame_id`, `position`, `orientation`). The storage path can be set through
  `RobotSdkOptions::home_pose_file` and defaults to
  `$HOME/.ros/zhishu_robot_sdk_cpp/home_pose.json`.
- Current status: implemented. The `test_home_pose` unit test covers the record format and file
  handling; motion execution still needs a live MoveIt/controller backend.

## 4. Recording and AR3-compatible JSON

- Default sampling rate: 30 Hz; the rate can be configured in `RecordingOptions`.
- A single-arm trajectory is saved as `traj_L_<name>.json` or `traj_R_<name>.json`.
- A synchronized dual-arm recording produces both files with a shared time origin.
- Required top-level fields remain `dof`, `hz`, `arm`, `name`, `created`, and `points_data`.
- Each `points_data` entry contains `t` (elapsed seconds) and `q` (seven joint positions in radians).
- Multiple recorded segments are concatenated in chronological order into `points_data`, matching the AR3
  player expectation. Optional SDK metadata may preserve segment boundaries; legacy fields are unchanged.
- Existing same-name files must not be overwritten unless `overwrite=true`.
- The trajectory directory is configurable through `RobotSdkOptions::trajectory_directory`.

## 5. Playback and speed

- `playTrajectory()` starts ordinary playback without scan monitoring.
- `playTrajectoryWithScan()` starts playback with scan monitoring.
- These APIs are designed as task-start requests; status is polled through
  `getTrajectoryPlaybackStatus()`.
- `speed_percent` and `setSpeedScale()` use percentage units `[0, 100]`; non-finite and out-of-range values must return `kInvalidArgument`.
- Effective MoveIt velocity scaling is defined as `configured_max_velocity_scaling * speed_percent / 100`. Thus 100% means the configured SDK/Commander cap, not an unconditional MoveIt scale of 1.0. Acceleration remains bounded by the configured acceleration cap unless a separately documented API is added.
- A playback speed of `0` cannot launch motion and must be rejected; use stop/pause controls instead. The percentage is not directly compatible with AR3 raw values such as `200`/`500`.
- Current status: speed-setting APIs are placeholders returning `kNotReady` for otherwise valid inputs.
- With `arms=kBoth`, the implementation pairs the `traj_L_<name>.json` and `traj_R_<name>.json` files.

## 6. Scan transport

- Topic type: `std_msgs/msg/String`; a non-empty string is a scan result.
- Service type: `std_srvs/srv/Trigger`; `response.success=true` means scan success and `response.message` may
  contain the scanned code.
- Endpoint names are configurable via `RobotSdkOptions::scan_topic_name` and
  `RobotSdkOptions::scan_service_name`. A per-call `ScanOptions::endpoint_name` overrides the selected
  endpoint; `topic_name` / `service_name` can also override the configured defaults. No names are hard-coded.
- If playback finishes without a matching scan, playback status reports `ScanOutcome::kNotDetected`.
- On successful scan, the eventual implementation requests motion stop and reports `stopped_by_scan=true`.

## 7. Trajectory management

- Prefer `Result listTrajectories(std::vector<TrajectoryInfo>&)` because it can distinguish an unavailable backend from a valid empty directory. It clears the output and returns `kNotReady` until implemented.
- The legacy vector-only `listTrajectories()` overload is retained for source compatibility; an empty result from it is ambiguous and must not be treated as proof that the directory is empty.
- `deleteTrajectory(name, arms)` deletes one named trajectory for the selected arm(s).
- `clearAllTrajectories(confirmed)` must refuse to clear files unless `confirmed=true`.

## 8. XYZABC step

- `moveStep()` performs a bounded relative increment for one arm only; `ArmGroup::kBoth` is not valid for this API.
- XYZ values are millimetres; ABC rotations are degrees in the robot base frame.
- A zero vector should be rejected by the eventual implementation.

## 9. Vacuum interface

- `vacuumOn()` accepts target vacuum percentage and acquisition timeout.
- Status should distinguish supported, active, workpiece secured, leak, measured vacuum, and fault code.
- `vacuumOff()` releases vacuum.
- No real driver/protocol is currently available; all three APIs remain placeholders and must not report success.

## 10. Additional common API placeholders

- `setSpeedScale()` / `getSpeedScale()` use percentage units `[0, 100]`.
- Tool/workobject CRUD APIs use normalized ROS geometry types and the existing YAML coordinate registry format;
  persistent active-frame selection is not introduced in this phase.
- `getRobotVersion()` and `getErrorCode()` return normalized types instead of copying vendor protocol structures.
- `readIO()` / `writeIO()` use `IOAddress` with module, type, direction and channel. Channel mapping is not frozen
  until the real hardware backend and wiring map are confirmed.
- These vendor-inspired additions are interface placeholders only and return `ErrorCode::kNotReady`.

## 11. Implementation status

The existing `RobotSdk`/`Commander` motion implementation is retained. The public entry points validate
joint-vector sizes, finite values, non-empty named targets, finite Cartesian poses and finite gripper commands
before entering MoveIt. `readState()` requires valid arm/torso-head joint feedback and separately reports
validity flags for optional gripper and pose feedback. `SdkStatus::ready` is false until the backend readiness
probe is implemented; it is never inferred from a non-null Commander pointer. `Commander::clearTargets()`
clears target state for arms, torso/head and grippers.

`robot_sdk.hpp` declares the unified `RobotSdk` class and its Excel workflows/common extension APIs. Most
`robot_sdk_extensions.cpp` functions are still linkable placeholders returning `ErrorCode::kNotReady` or an
explicit default status; the unified dual-arm Home pair (`setHomePose()` / `moveHome()`) is implemented on top
of `moveBothPoses()` plus the internal `home_pose` record module. The separate wrapper class has been removed so
callers use only `RobotSdk`. No new Jog, trajectory recording/playback, scan transport, IO backend, coordinate
CRUD or vacuum hardware behavior is implemented yet. Existing `RobotSdk` movement functions remain implemented.

## 12. Acceptance gates

1. Configure-time version gate: CMake project version, `package.xml`, and `version.hpp` must all be `0.3.1` for this source snapshot. CMake fails if `package.xml` differs from the CMake project version; unit tests verify the public version constants.
2. Unit/API gate: all registered gtests must pass. This includes public API compile checks, version/type contracts, coordinate transforms, pose math, unified Home record handling, and stability-recording unit tests.
3. Binary gate: `libzhishu_robot_sdk.so` must exist after build, resolve runtime dependencies (`ldd` has no `not found` entries), and export `RobotSdk` symbols.
4. Fake-hardware smoke gate: run the installed `test/commander_fake_validation.launch.py` in a dedicated ROS domain. Verify that only `mock_components/GenericSystem` is configured and that the MoveIt/Commander stack starts without sending motion commands. Stop it with Ctrl-C after checking logs.
5. Real-hardware gate: do not run stability loops or motion tests until Fake Hardware passes, joint/group names and limits are confirmed, E-stop and motor-power procedures are available, and a human operator approves the test. This patch does not claim real-hardware validation.
6. Feature gate: Excel workflow APIs (Jog, recording/playback, scan, IO and vacuum) still return `kNotReady` until their backends are implemented. Tests must not count placeholder APIs as implemented functionality. The unified dual-arm Home pair is implemented and covered by `test_home_pose`; its motion path is still validated only through the existing dual-arm motion tests.
