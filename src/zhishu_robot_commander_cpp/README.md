# zhishu_robot_commander_cpp

智枢双臂机器人 MoveIt 2 C++ SDK，提供机械臂、夹爪的 C++ 调用接口和 ROS 2
话题桥接节点。Commander 复用已经运行的 `move_group`，不会重复启动控制器，节点
启动时也不会自行发送运动命令。

## 功能

- 左/右臂：命名目标、7 关节目标、Hand 位姿目标、真实夹爪 TCP 位姿目标、笛卡尔目标
- 双臂：命名目标、14 关节目标、左右末端双目标同步规划
- 夹爪：左/右独立控制、双夹爪同步控制和打开/关闭快捷命令
- C++ SDK 动态库：`libzhishu_robot_commander_cpp.so`
- ROS 2 话题桥接节点：`commander_node`
- 结构化状态：`zhishu_robot_interfaces/msg/CommanderStatus`
- 稳定性测试：命名姿态循环、14 轴 Joint/EtherCAT 循环

## 编译与启动

```bash
cd /home/pi/Documents/zhul/guohua_ws
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

结果格式继续保持 `SUCCESS: <command>` 或 `FAILED: <command>`，用于兼容现有
`receiving_sim` 等调用端。新增结构化结果：

```bash
ros2 topic echo /zhishu_robot_commander/status_structured
```

消息类型为 `zhishu_robot_interfaces/msg/CommanderStatus`，包含 `command`、`success`、
`error_code`、`error_name`、`native_code` 和 `message`。

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

### 真实夹爪 TCP 位姿目标

原 `/pose_command` 仍表示 `L_hand_link` / `R_hand_link` 目标。抓取任务可直接给
`left_gripper_tcp` / `right_gripper_tcp` 目标；Commander 从 MoveIt RobotModel 动态读取
TCP 与 Hand 的固定变换，不硬编码夹爪长度。

```bash
ros2 topic pub --once /zhishu_robot_commander/left_arm/tcp_pose_command \
  geometry_msgs/msg/PoseStamped \
  "{header: {frame_id: base_link}, pose: {position: {x: 0.35, y: 0.30, z: 0.75}, orientation: {w: 1.0}}}"
```

新增话题：

- `/left_arm/tcp_pose_command`、`/right_arm/tcp_pose_command`
- `/both_arms/tcp_pose_command`（`PoseArray`）
- `/left_arm/cartesian_tcp_pose_command`、`/right_arm/cartesian_tcp_pose_command`

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

推荐新代码使用 `RobotSdk`。它提供结构化错误、同步/异步运动、单任务互斥、状态查询、
等待和取消；原有 `Commander` 的 `bool` 接口继续保留，REST、MCP 和旧程序无需立即
修改。

### 同步调用

```cpp
#include <zhishu_robot_commander_cpp/sdk.hpp>

// node 必须由 MultiThreadedExecutor 持续 spin。
zhishu_robot_commander_cpp::RobotSdk robot(node);

auto result = robot.moveNamed(
  zhishu_robot_commander_cpp::ArmGroup::kBoth,
  "both_arms_home");
if (!result) {
  RCLCPP_ERROR(
    node->get_logger(), "motion failed [%s]: %s",
    zhishu_robot_commander_cpp::toString(result.code),
    result.message.c_str());
}
```

`Result.code` 可以区分：

- `kInvalidArgument`：参数数量、范围或数值无效；
- `kTargetRejected`：MoveIt 不接受目标；
- `kPlanningFailed`：规划失败；
- `kExecutionFailed`：轨迹执行失败；
- `kBusy`：已有同步或异步运动；
- `kCancelled`、`kTimeout`：取消或等待超时；
- `kNotReady`、`kInternalError`：状态未就绪或 SDK 内部异常。

### 异步调用

```cpp
using namespace std::chrono_literals;

auto motion = robot.moveNamedAsync(
  zhishu_robot_commander_cpp::ArmGroup::kBoth,
  "both_arms_crate_safe");

auto result = motion.waitFor(20s);
if (result.code == zhishu_robot_commander_cpp::ErrorCode::kTimeout) {
  motion.cancel();
  result = motion.wait();
}

const auto snapshot = motion.snapshot();
RCLCPP_INFO(
  node->get_logger(), "task=%lu operation=%s status=%s",
  static_cast<unsigned long>(snapshot.id),
  snapshot.operation.c_str(),
  zhishu_robot_commander_cpp::toString(snapshot.status));
```

一个 `RobotSdk` 实例同一时刻只允许一个运动任务。忙碌时新的同步调用返回
`kBusy`，新的异步调用返回一个已经结束且结果为 `kBusy` 的句柄。`cancel()` 调用
MoveIt 软件停止，它不能代替实体急停、安全 PLC 或驱动器 STO。

### 状态读取

```cpp
auto sdk_status = robot.status();
auto robot_state = robot.readState();
if (robot_state.result) {
  const auto & left_joints = robot_state.left_arm_joints;
  const auto & left_gripper = robot_state.left_gripper_joints;
}
```

`RobotState` 一次返回左右臂关节、左右夹爪关节、左右 Hand 位姿以及
`left_gripper_tcp` / `right_gripper_tcp` 位姿。运动执行期间调用 `readState()` 会等待当前
Commander 操作释放互斥锁，适合任务前后快照，不作为实时遥测接口使用。

TCP C++ API：

```cpp
auto result = robot.moveTcpPose(
  zhishu_robot_commander_cpp::ArmGroup::kLeft, left_tcp_target, false);
auto both = robot.moveBothTcpPoses(left_tcp_target, right_tcp_target);
```

对应异步接口为 `moveTcpPoseAsync()` / `moveBothTcpPosesAsync()`。

旧接口仍可使用：

```cpp
#include <zhishu_robot_commander_cpp/commander.hpp>

zhishu_robot_commander_cpp::Commander commander(node);
commander.moveBothArmsNamed("both_arms_home");
```

依赖本 SDK 的包需要添加：

```cmake
find_package(zhishu_robot_commander_cpp REQUIRED)
ament_target_dependencies(your_target zhishu_robot_commander_cpp)
```

```xml
<depend>zhishu_robot_commander_cpp</depend>
```

安装后的安全示例默认不运动；确认使用仿真或完成真机安全检查后再执行：

```bash
ros2 run zhishu_robot_commander_cpp sdk_example --ros-args \
  -p execute_motion:=true
```

Ubuntu 24.04 / ROS 2 Jazzy 上的一键编译和单元测试（不会发送运动命令）：

```bash
bash scripts/test_commander_sdk_build.sh
```


### 稳定性循环测试

命名姿态循环使用当前 SRDF 已存在的 `both_arms_home -> both_arms_pose1 -> both_arms_home`：

```bash
ros2 launch zhishu_robot_commander_cpp sdk_named_pose_loop.launch.py \
  loop_count:=10 interval_sec:=1.5 use_fake_hardware:=true
```

Joint/EtherCAT 循环直接发送 14 轴 joint target，默认目标位于
`config/joint_ethercat_loop.yaml`：

```bash
ros2 launch zhishu_robot_commander_cpp sdk_joint_ethercat_loop.launch.py \
  loop_count:=1 interval_sec:=1.5 use_fake_hardware:=true
```

两种循环测试都会统计总运行时间、循环数、成功/失败次数、单目标平均/最小/最大耗时；
任一动作失败立即停止。Ctrl+C 会请求取消当前 MoveIt 动作后退出。真机运行前必须先核对
目标关节角、碰撞环境和急停/STO 状态。

### Python SDK

新增 `zhishu_robot_commander_py`。Python 端只作为 ROS 2 client，不重复加载 MoveIt；
运动结果从 `/zhishu_robot_commander/status_structured` 获取。

```bash
colcon build --symlink-install --packages-up-to zhishu_robot_commander_py
```

普通 MoveIt bringup 激活的是覆盖双臂 14 轴的 `dual_arm_controller`，不要同时
激活左右单臂控制器，以免争用相同的 position command interface。
