^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package zhishu_robot_sdk_cpp
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.3.1 (2026-10-21)
------------------
* Implement the unified dual-arm Home pair: ``setHomePose()`` writes one JSON Home record with the
  14 arm joints and both end-effector poses, and ``moveHome()`` replays the joints through
  ``moveJoints()`` so the posture matches exactly; legacy pose-only records use
  ``moveBothPoses()`` and no record falls back to the built-in ``both_arms_home`` group state.
* Add the internal ``home_pose`` record module with atomic writes and strict validation, covered by
  the new ``test_home_pose`` unit test.
* Refuse to capture a Home record when arm joint feedback is incomplete (``kNotReady``) instead of
  writing an all-zero record; document that Gazebo runs need ``use_sim_time:=true``.
* Add ``sdk_home_example`` for Home pre-flight checks, capture and replay; it stays in no-motion
  mode unless ``execute_motion:=true`` is set.
* Serialize motion admission and worker reaping under concurrent sync/async calls.
* Complete cancellation only after its stop callback returns; handle callback failure without escaping.
* Isolate optional feedback failures from required joints and reject gripper bounds before queueing.
* Add deterministic motion-task tests and a separate GenericSystem concurrency regression consumer.
* Add public-library symbol visibility macros and normalized robot status / IO types.
* Add placeholder APIs for speed scaling, tool/workobject CRUD, version/error query, and IO access.
* Strengthen synchronous and asynchronous SDK input validation for joint vectors, Cartesian poses,
  named targets and gripper positions.
* Report incomplete joint feedback as ``kNotReady`` rather than a successful state snapshot.
* Remove a duplicate MoveIt target-clear call.
* Keep new Excel/vendor-inspired feature APIs as explicit ``kNotReady`` stubs.

0.3.0 (2026-09-10)
------------------
* Add true ``left_gripper_tcp`` / ``right_gripper_tcp`` pose APIs with RobotModel-based rigid transform conversion to the arm IK tips.
* Add structured ``zhishu_robot_interfaces/msg/CommanderStatus`` results on ``~/status_structured`` while preserving the existing String ``~/status`` topic.
* Add TCP pose topics for single-arm, dual-arm, and Cartesian commands.
* Add a customer-facing Python Commander SDK.
* Add named-pose and joint/EtherCAT loop stability test executables with timing/failure summaries and safe cancellation.
* Include TCP poses in ``RobotState`` snapshots.

0.2.0 (2026-08-30)
------------------
* Add structured SDK results and stable error codes.
* Add the ``RobotSdk`` facade with synchronous and asynchronous motion APIs.
* Add cancellable ``MotionHandle`` tasks, timeout waits and status snapshots.
* Include arm, gripper and end-effector feedback in the unified robot state snapshot.
* Validate SDK configuration and reject non-finite joint and pose targets.
* Keep all 0.1 ``Commander`` boolean APIs source compatible.

0.1.0
-----
* Initial MoveIt Commander library and ROS 2 topic bridge.
