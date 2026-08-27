# zhishu_robot_commander_cpp

智枢双臂机器人 MoveIt 2 C++ SDK，提供机械臂、夹爪的 C++ 调用接口和 ROS 2
话题桥接节点。Commander 复用已经运行的 `move_group`，不会重复启动控制器，节点
启动时也不会自行发送运动命令。

## 功能

- 左/右臂：命名目标、7 关节目标、末端位姿目标、笛卡尔目标
- 双臂：命名目标、14 关节目标、左右末端双目标同步规划
- 夹爪：左/右独立控制、双夹爪同步控制和打开/关闭快捷命令
- C++ SDK 动态库：`libzhishu_robot_commander_cpp.so`
- ROS 2 话题桥接节点：`commander_node`

## 编译与启动

```bash
cd /home/zhul/online/guohua_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-up-to zhishu_robot_commander_cpp
source install/setup.bash
```

先启动机器人、控制器和 MoveIt：

```bash
ros2 launch zhishu_robot_bringup zhishu_robot.moveit.launch.py \
  use_fake_hardware:=true use_rviz:=true \
  enable_kernel_ethercat_log:=false
```

再启动 Commander。`use_fake_hardware` 和 `enable_motors_on_activate` 必须与
bringup 保持一致：

```bash
ros2 launch zhishu_robot_commander_cpp commander.launch.py \
  use_fake_hardware:=true enable_motors_on_activate:=true
```

## ROS 2 话题接口

所有接口位于 `/zhishu_robot_commander` 命名空间。执行结果发布到：

```bash
ros2 topic echo /zhishu_robot_commander/status
```

结果格式为 `SUCCESS: <command>` 或 `FAILED: <command>`。

### 命名目标

```bash
ros2 topic pub --once /zhishu_robot_commander/left_arm/named_target \
  std_msgs/msg/String "{data: left_arm_home}"

ros2 topic pub --once /zhishu_robot_commander/right_arm/named_target \
  std_msgs/msg/String "{data: right_arm_home}"

ros2 topic pub --once /zhishu_robot_commander/both_arms/named_target \
  std_msgs/msg/String "{data: both_arms_home}"
```

可用命名目标：

- 左臂：`left_arm_home`、`left_arm_pose1`、`left_arm_crate_pre`
- 右臂：`right_arm_home`、`right_arm_pose1`、`right_arm_crate_pre`
- 双臂：`both_arms_home`、`both_arms_pose1`、`both_arms_crate_safe`、
  `both_arms_crate_pre`

### 关节目标

单臂消息包含 7 个关节，双臂消息按左臂 7 轴、右臂 7 轴排列，单位 rad：

```bash
ros2 topic pub --once /zhishu_robot_commander/left_arm/joint_command \
  std_msgs/msg/Float64MultiArray \
  "{data: [0.0, -0.2, 0.0, 0.8, 0.0, 0.0, 0.0]}"

ros2 topic pub --once /zhishu_robot_commander/both_arms/joint_command \
  std_msgs/msg/Float64MultiArray \
  "{data: [0.0, -0.2, 0.0, 0.8, 0.0, 0.0, 0.0, 0.0, 0.2, 0.0, 0.8, 0.0, 0.0, 0.0]}"
```

### 位姿目标

默认末端为 `L_hand_link` 和 `R_hand_link`：

```bash
ros2 topic pub --once /zhishu_robot_commander/left_arm/pose_command \
  geometry_msgs/msg/PoseStamped \
  "{header: {frame_id: base_link}, pose: {position: {x: 0.35, y: 0.30, z: 0.75}, orientation: {w: 1.0}}}"
```

右臂使用 `/right_arm/pose_command`。笛卡尔直线目标使用：

- `/zhishu_robot_commander/left_arm/cartesian_pose_command`
- `/zhishu_robot_commander/right_arm/cartesian_pose_command`

双臂同步位姿使用 `/both_arms/pose_command`，消息类型为
`geometry_msgs/msg/PoseArray`；`poses[0]` 是左臂目标，`poses[1]` 是右臂目标。

### 夹爪

夹爪参数单位为米，`0.0` 为打开、`0.06` 为闭合。SDK 实际控制 MoveIt 与
controller 共同使用的主关节 `left_gripper_left_finger_joint` 和
`gripper_left_finger_joint`：

```bash
ros2 topic pub --once /zhishu_robot_commander/left_gripper/command \
  std_msgs/msg/Float64 "{data: 0.03}"

ros2 topic pub --once /zhishu_robot_commander/right_gripper/command \
  std_msgs/msg/Float64 "{data: 0.06}"

ros2 topic pub --once /zhishu_robot_commander/both_grippers/command \
  std_msgs/msg/Float64MultiArray "{data: [0.0, 0.0]}"
```

快捷命令为 `open_left`、`close_left`、`open_right`、`close_right`、
`open_both` 和 `close_both`：

```bash
ros2 topic pub --once /zhishu_robot_commander/grippers/named_command \
  std_msgs/msg/String "{data: open_both}"
```

### 停止 MoveIt 执行

```bash
ros2 topic pub --once /zhishu_robot_commander/stop std_msgs/msg/Empty "{}"
```

这只会请求 MoveIt 停止当前软件轨迹，不能代替实体急停或安全回路。

## C++ SDK

```cpp
#include <zhishu_robot_commander_cpp/commander.hpp>

// node 必须由 MultiThreadedExecutor 持续 spin。
zhishu_robot_commander_cpp::Commander commander(node);
commander.moveBothArmsNamed("both_arms_home");
commander.moveLeftArmJoints({0.0, -0.2, 0.0, 0.8, 0.0, 0.0, 0.0});
commander.openBothGrippers();
```

依赖本 SDK 的包需要添加：

```cmake
find_package(zhishu_robot_commander_cpp REQUIRED)
ament_target_dependencies(your_target zhishu_robot_commander_cpp)
```

```xml
<depend>zhishu_robot_commander_cpp</depend>
```

普通 MoveIt bringup 激活的是覆盖双臂 14 轴的 `dual_arm_controller`，不要同时
激活左右单臂控制器，以免争用相同的 position command interface。
