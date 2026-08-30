^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package zhishu_robot_commander_cpp
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

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
