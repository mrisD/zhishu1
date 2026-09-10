^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package zhishu_robot_commander_cpp
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

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
