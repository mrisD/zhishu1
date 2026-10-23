# zhishu_robot_sdk_cpp — 独立 C++ SDK 迁移版

本包从 `zhishu_robot_commander_cpp` **完整复制**原有 C++ SDK、节点、配置、测试和示例。
原包保持不变；新包单独编译 `libzhishu_robot_sdk.so`，不链接原包的库。对外统一只保留
`zhishu_robot_sdk::RobotSdk` 一个 SDK 类，原有运动接口与 Excel 工作流接口集中在同一类中。

## 兼容性与隔离

- **包名**：`zhishu_robot_sdk_cpp`
- **公共头文件**：`#include <zhishu_robot_sdk/sdk.hpp>`
- **命名空间**：`zhishu_robot_sdk`
- **核心类**：唯一对外 SDK `zhishu_robot_sdk::RobotSdk`；内部运动规划类 `zhishu_robot_sdk::Commander`
- **编译目标**：`zhishu_robot_sdk`；库文件 `libzhishu_robot_sdk.so`
- **新桥接节点**：`/zhishu_robot_sdk_commander`，旧节点 `/zhishu_robot_commander` 不受影响
- **依赖**：ROS 2 Jazzy、MoveIt 2、`zhishu_robot_interfaces` 等原包已使用的公共运行服务

对外只保留一个 `zhishu_robot_sdk::RobotSdk` 类。原有运动 API 与新增 Excel 工作流 API 在该类中统一声明；统一双臂 Home 已实现（`setHomePose` / `moveHome`，并附带 Python 版 `Back_Home.py`），其余新增工作流（Jog、轨迹录制/回放、扫码、电吸、IO 等）目前仍是空框架，返回 `kNotReady`，后续逐项接入现有 Commander 和 ROS 2 服务。

自动化构建、测试、动态库依赖与 Fake Hardware 验收流程见 [SDK_ACCEPTANCE.md](SDK_ACCEPTANCE.md)。

## 代码目录

- `include/zhishu_robot_sdk/`：SDK 公共头文件。
- `src/`：SDK 内部实现和 Commander 桥接节点，包括任务状态机 `motion_task.hpp/.cpp`。
- `test/`：单元测试、仿真回归测试、SDK 示例、循环测试，以及测试共用的 `stability_pdo_recording.hpp/.cpp`。

示例和循环测试的可执行程序名称、安装位置及启动命令保持不变；它们仍默认编译，
单元测试和仿真回归测试程序由 `BUILD_TESTING` 控制。

## 构建

```bash
cd ~/guohua_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-select zhishu_robot_sdk_cpp --symlink-install
source install/setup.bash
ros2 pkg executables zhishu_robot_sdk_cpp
```

若已有安装的自定义依赖（如 `zhishu_robot_interfaces`），先 `source` 对应工作空间，或改用
`colcon build --packages-up-to zhishu_robot_sdk_cpp`。

```cpp
#include <zhishu_robot_sdk/sdk.hpp>
// 保留了原有 RobotSdk 的构造、运动、同步/异步状态接口，改用独立命名空间。
// zhishu_robot_sdk::RobotSdk robot(node);
```

CMake 项目示例：

```cmake
find_package(ament_cmake REQUIRED)
find_package(zhishu_robot_sdk_cpp REQUIRED)
# 建议 ROS 2 ament 消费者使用：
ament_target_dependencies(your_target zhishu_robot_sdk_cpp)
```

## 运行安全

新旧两个 ROS 桥接节点具有不同名称，但仍操作**同一 MoveIt / ros2_control 控制链**。
**不要同时向相同的机械臂发送运动指令**。新包不会自动使能或上下电；新示例默认保持原有安全开关。
部分稳定性循环示例会运动真机，请先在 Fake Hardware 测试并验证关节限位、碰撞空间和急停。
以下迁移自原包的说明仅作为现有功能参考，其中关于旧 Python SDK 的章节仍指向原包，
Python SDK 本次未迁移。

---

## 原 C++ SDK 使用说明（已更新包名与新节点名称）


智枢机器人 MoveIt 2 C++ SDK，提供机械臂、头腰关节、夹爪的 C++ 调用接口和 ROS 2
话题桥接节点。Commander 复用已经运行的 `move_group`，不会重复启动控制器，节点
启动时也不会自行发送运动命令。

## 功能

新增工具/工件逐条指定和 `MoveJ`、`MoveL`、`MoveAbsJ`、`Offs`、`RelTool`，详见 [SDK_FRAMES.md](SDK_FRAMES.md)。普通位姿默认 tool0，旧 TCP 接口保留夹爪 TCP 默认，工件默认 wobj0；不导入仿真标定数据。

- 左/右臂：命名目标、7 关节目标、Hand 位姿目标、真实夹爪 TCP 位姿目标、笛卡尔目标
- 双臂：命名目标、14 关节目标、左右末端双目标同步规划
- 回 Home：统一双臂 Home 记录（`setHomePose` / `moveHome`）和 Python 版 `Back_Home.py`
- 头腰：命名目标、3 关节目标和当前关节角度读取
- 夹爪：左/右独立控制、双夹爪同步控制和打开/关闭快捷命令
- C++ SDK 动态库：`libzhishu_robot_sdk.so`
- ROS 2 话题桥接节点：`commander_node`
- 结构化状态：`zhishu_robot_interfaces/msg/CommanderStatus`
- 稳定性测试：命名姿态循环、14 轴 Joint/EtherCAT 循环

## 躯干 / 头部接口

- MoveIt 规划组：`torso_head`
- ros2_control 轨迹控制器：`torso_head_controller`
- 关节目标固定按 `[waist_pitch_joint, head_yaw_joint, head_pitch_joint]` 排列，单位为弧度（rad）。
- 命名目标直接使用 SRDF 中为 `torso_head` 组定义的 named target。

### Commander API

结构化结果接口：

```cpp
Result moveTorsoHeadNamedResult(const std::string & target_name);
Result moveTorsoHeadJointsResult(const std::vector<double> & joints);
std::vector<double> getTorsoHeadJoints() const;
```

同时提供便捷的 `bool` 包装接口 `moveTorsoHeadNamed()` 和
`moveTorsoHeadJoints()`；它们分别调用对应的 `Result` 接口并返回是否成功。
关节目标必须正好包含上述三个关节，顺序不可交换。

### RobotSdk API

同步接口：

```cpp
Result moveTorsoHeadNamed(const std::string & target_name);
Result moveTorsoHeadJoints(const std::vector<double> & joints);
std::vector<double> getTorsoHeadJoints() const;
```

异步接口：

```cpp
MotionHandle moveTorsoHeadNamedAsync(std::string target_name);
MotionHandle moveTorsoHeadJointsAsync(std::vector<double> joints);
```

状态结构 `RobotState` 包含 `torso_head_joints`，按
`[waist_pitch_joint, head_yaw_joint, head_pitch_joint]` 顺序返回当前关节位置，单位为 rad。
若只启用头腰部假硬件，可用 `getTorsoHeadJoints()` 单独读取这三个关节，
不依赖双臂关节状态；状态不可用时抛出异常。

### C++ 可运行示例

先启动含 `torso_head_controller`、MoveIt 和 Commander 的系统。仅验证假硬件时可用：

```bash
ros2 launch zhishu_robot_bringup zhishu_robot.moveit.launch.py \
  torso_head_use_fake_hardware:=true enable_torso_head_hardware:=true \
  enable_left_arm_hardware:=false enable_right_arm_hardware:=false \
  enable_commander:=true use_rviz:=false
```

另开终端并 `source install/setup.bash`。示例默认只读取状态，不会发运动命令：

```bash
ros2 run zhishu_robot_sdk_cpp sdk_torso_head_example
```

只有显式设置 `execute_motion:=true` 才会发送一次运动命令：

```bash
ros2 run zhishu_robot_sdk_cpp sdk_torso_head_example --ros-args \
  -p execute_motion:=true -p motion:=joints -p joints:="[0.0, 0.05, -0.05]"
ros2 run zhishu_robot_sdk_cpp sdk_torso_head_example --ros-args \
  -p execute_motion:=true -p motion:=named -p named_target:=torso_head_origin
```

真机运行前须核对实际位置、关节方向、限位和周边空间；上面的两条
`execute_motion:=true` 命令会实际请求运动。

### ROS 话题桥接

桥接节点私有话题（`~` 表示节点命名空间下的相对名称）：

| 话题 | 类型 | 内容 |
| --- | --- | --- |
| `~/torso_head/joint_command` | `std_msgs/msg/Float64MultiArray` | 三个 rad 关节目标，按 waist pitch、head yaw、head pitch 排列 |
| `~/torso_head/named_target` | `std_msgs/msg/String` | SRDF 中 `torso_head` 组的命名目标 |

可先用以下只读命令确认桥接节点和接口是否已启动：

```bash
ros2 node list
ros2 topic list | grep torso_head
ros2 topic info /zhishu_robot_sdk_commander/torso_head/joint_command
ros2 topic echo /joint_states --once
```

若桥接节点名或命名空间不同，应以 `ros2 node list` 和 `ros2 node info <节点名>`
显示的实际私有话题全名为准。发布 `joint_command` 或 `named_target` 会请求机器人运动；
真机上执行前应核对当前关节状态、规划目标和机械限位。接口定义本身不会触发运动。

## 编译与启动

```bash
cd /home/pi/Documents/zhul/guohua_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-up-to zhishu_robot_sdk_cpp
source install/setup.bash
```

先启动机器人、控制器和 MoveIt：

```bash
ros2 launch zhishu_robot_bringup zhishu_robot.moveit.launch.py \
  use_rviz:=true \
  enable_kernel_ethercat_log:=false
```

再启动 Commander。独立 Commander 启动文件仍有自己的
`use_fake_hardware` 参数；它和 `enable_motors_on_activate` 必须与
bringup 的机器人描述保持一致。混合 mock/真机模式建议直接在主 bringup 中
设置 `enable_commander:=true`，以复用相同的机器人描述：

```bash
ros2 launch zhishu_robot_sdk_cpp commander.launch.py \
  use_fake_hardware:=true enable_motors_on_activate:=true
```

## ROS 2 话题接口

所有接口位于 `/zhishu_robot_sdk_commander` 命名空间。执行结果发布到：

```bash
ros2 topic echo /zhishu_robot_sdk_commander/status
```

结果格式继续保持 `SUCCESS: <command>` 或 `FAILED: <command>`，用于兼容现有
`receiving_sim` 等调用端。新增结构化结果：

```bash
ros2 topic echo /zhishu_robot_sdk_commander/status_structured
```

消息类型为 `zhishu_robot_interfaces/msg/CommanderStatus`，包含 `command`、`success`、
`error_code`、`error_name`、`native_code` 和 `message`。

### 命名目标

```bash
ros2 topic pub --once /zhishu_robot_sdk_commander/left_arm/named_target \
  std_msgs/msg/String "{data: left_arm_home}"

ros2 topic pub --once /zhishu_robot_sdk_commander/right_arm/named_target \
  std_msgs/msg/String "{data: right_arm_home}"

ros2 topic pub --once /zhishu_robot_sdk_commander/both_arms/named_target \
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
ros2 topic pub --once /zhishu_robot_sdk_commander/left_arm/joint_command \
  std_msgs/msg/Float64MultiArray \
  "{data: [0.0, -0.2, 0.0, 0.8, 0.0, 0.0, 0.0]}"

ros2 topic pub --once /zhishu_robot_sdk_commander/both_arms/joint_command \
  std_msgs/msg/Float64MultiArray \
  "{data: [0.0, -0.2, 0.0, 0.8, 0.0, 0.0, 0.0, 0.0, 0.2, 0.0, 0.8, 0.0, 0.0, 0.0]}"
```

### 位姿目标

默认末端为 `L_hand_link` 和 `R_hand_link`：

```bash
ros2 topic pub --once /zhishu_robot_sdk_commander/left_arm/pose_command \
  geometry_msgs/msg/PoseStamped \
  "{header: {frame_id: base_link}, pose: {position: {x: 0.35, y: 0.30, z: 0.75}, orientation: {w: 1.0}}}"
```

右臂使用 `/right_arm/pose_command`。笛卡尔直线目标使用：

- `/zhishu_robot_sdk_commander/left_arm/cartesian_pose_command`
- `/zhishu_robot_sdk_commander/right_arm/cartesian_pose_command`

双臂同步位姿使用 `/both_arms/pose_command`，消息类型为
`geometry_msgs/msg/PoseArray`；`poses[0]` 是左臂目标，`poses[1]` 是右臂目标。

### 真实夹爪 TCP 位姿目标

原 `/pose_command` 仍表示 `L_hand_link` / `R_hand_link` 目标。抓取任务可直接给
`left_gripper_tcp` / `right_gripper_tcp` 目标；Commander 从 MoveIt RobotModel 动态读取
TCP 与 Hand 的固定变换，不硬编码夹爪长度。

```bash
ros2 topic pub --once /zhishu_robot_sdk_commander/left_arm/tcp_pose_command \
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
ros2 topic pub --once /zhishu_robot_sdk_commander/left_gripper/command \
  std_msgs/msg/Float64 "{data: 0.03}"

ros2 topic pub --once /zhishu_robot_sdk_commander/right_gripper/command \
  std_msgs/msg/Float64 "{data: 0.06}"

ros2 topic pub --once /zhishu_robot_sdk_commander/both_grippers/command \
  std_msgs/msg/Float64MultiArray "{data: [0.0, 0.0]}"
```

快捷命令为 `open_left`、`close_left`、`open_right`、`close_right`、
`open_both` 和 `close_both`：

```bash
ros2 topic pub --once /zhishu_robot_sdk_commander/grippers/named_command \
  std_msgs/msg/String "{data: open_both}"
```

### 停止 MoveIt 执行

```bash
ros2 topic pub --once /zhishu_robot_sdk_commander/stop std_msgs/msg/Empty "{}"
```

这只会请求 MoveIt 停止当前软件轨迹，不能代替实体急停或安全回路。

## C++ SDK

推荐新代码使用 `RobotSdk`。它提供结构化错误、同步/异步运动、单任务互斥、状态查询、
等待和取消；原有 `Commander` 的 `bool` 接口继续保留，REST、MCP 和旧程序无需立即
修改。

### 同步调用

```cpp
#include <zhishu_robot_sdk/sdk.hpp>

// node 必须由 MultiThreadedExecutor 持续 spin。
zhishu_robot_sdk::RobotSdk robot(node);

auto result = robot.moveNamed(
  zhishu_robot_sdk::ArmGroup::kBoth,
  "both_arms_home");
if (!result) {
  RCLCPP_ERROR(
    node->get_logger(), "motion failed [%s]: %s",
    zhishu_robot_sdk::toString(result.code),
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
  zhishu_robot_sdk::ArmGroup::kBoth,
  "both_arms_crate_safe");

auto result = motion.waitFor(20s);
if (result.code == zhishu_robot_sdk::ErrorCode::kTimeout) {
  motion.cancel();
  result = motion.wait();
}

const auto snapshot = motion.snapshot();
RCLCPP_INFO(
  node->get_logger(), "task=%lu operation=%s status=%s",
  static_cast<unsigned long>(snapshot.id),
  snapshot.operation.c_str(),
  zhishu_robot_sdk::toString(snapshot.status));
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
  zhishu_robot_sdk::ArmGroup::kLeft, left_tcp_target, false);
auto both = robot.moveBothTcpPoses(left_tcp_target, right_tcp_target);
```

对应异步接口为 `moveTcpPoseAsync()` / `moveBothTcpPosesAsync()`。

### 回 Home：setHomePose / moveHome（Back_Home）

`Back_Home` 用一条统一记录让双臂同时回到 Home 姿态，不再分别读取左右臂各自的历史位置：

```cpp
// 记录当前双臂末端姿态（L_hand_link / R_hand_link），两臂共用一次采样时间与一个文件。
auto saved = robot.setHomePose();
// 双臂同时规划并执行，回到该记录；没有记录时使用 SRDF 内置的 both_arms_home。
auto moved = robot.moveHome();
```

- 记录默认写入 `$HOME/.ros/zhishu_robot_sdk_cpp/home_pose.json`，可用
  `RobotSdkOptions::home_pose_file` 修改。文件为 JSON version 1，包含 `captured_at`、
  `left` 和 `right` 两个末端位姿（`frame_id`、`position`、`orientation`）。
- `moveHome()` 通过 `moveBothPoses()` 重放该记录，因此沿用与普通双臂位姿目标相同的
  校验、同步结果、任务互斥与取消行为。
- 没有记录时回退到 SRDF 的 `both_arms_home`；记录存在但无法解析时直接返回
  `kInvalidArgument`，不会把一侧保存姿态与另一侧的命名姿态静默拼接。写入失败返回
  `kInternalError`。
- `setHomePose()` 在 SDK 侧同步任务中执行：已有运动时返回 `kBusy`，避免采样到运动中
  的姿态；仍应在机器人静止时调用。

安装后可直接运行 Python 版回 Home 程序，默认只检查目标、不发送运动：

```bash
ros2 run zhishu_robot_sdk_cpp Back_Home.py
ros2 run zhishu_robot_sdk_cpp Back_Home.py --ros-args -p execute_motion:=true
```

`Back_Home.py` 与 C++ 的 `moveHome()` 使用同一份记录：存在记录时向
`/zhishu_robot_sdk_commander/both_arms/pose_command` 发布两臂末端位姿（即
`moveBothPoses` 路径），否则向 `both_arms/named_target` 发布 `both_arms_home`。可用参数：
`execute_motion`（默认 `false`）、`target_name`（默认 `both_arms_home`）、
`home_pose_file`、`timeout_sec`、`discovery_timeout_sec` 和 `commander_namespace`
（默认 `/zhishu_robot_sdk_commander`）。程序根据 `status_structured` 上
`both_arms/pose_command` 或 `both_arms/named_target` 的结果决定退出码：0 成功（含未使能
运动时的预检）、1 运动失败、2 配置或环境错误。

旧接口仍可使用：

```cpp
#include <zhishu_robot_sdk/commander.hpp>

zhishu_robot_sdk::Commander commander(node);
commander.moveBothArmsNamed("both_arms_home");
```

依赖本 SDK 的包需要添加：

```cmake
find_package(zhishu_robot_sdk_cpp REQUIRED)
ament_target_dependencies(your_target zhishu_robot_sdk_cpp)
```

```xml
<depend>zhishu_robot_sdk_cpp</depend>
```

安装后的安全示例默认不运动；确认使用仿真或完成真机安全检查后再执行：

```bash
ros2 run zhishu_robot_sdk_cpp sdk_example --ros-args \
  -p execute_motion:=true
```

Ubuntu 24.04 / ROS 2 Jazzy 上的一键编译和单元测试（不会发送运动命令）：

```bash
bash scripts/test_commander_sdk_build.sh
```


### 稳定性循环测试

命名姿态循环使用当前 SRDF 已存在的 `both_arms_home -> both_arms_pose1 -> both_arms_home`：

```bash
ros2 launch zhishu_robot_sdk_cpp sdk_named_pose_loop.launch.py \
  loop_count:=10 interval_sec:=1.5 use_fake_hardware:=true
```

Joint/EtherCAT 循环直接发送 14 轴 joint target，默认目标位于
`config/joint_ethercat_loop.yaml`。其中 `right_return_joints` 是右臂的
预备位和循环返回位，按右臂 7 轴顺序以角度配置，默认
`[30, 15, 0, 0, 0, 0, 0]`；左臂预备位和返回位仍为全零：

```bash
ros2 launch zhishu_robot_sdk_cpp sdk_joint_ethercat_loop.launch.py \
  loop_count:=1 interval_sec:=1.5 use_fake_hardware:=true
```

两种循环测试都会统计总运行时间、循环数、成功/失败次数、单目标平均/最小/最大耗时；
任一动作失败立即停止。Ctrl+C 会请求取消当前 MoveIt 动作后退出。真机运行前必须先核对
目标关节角、碰撞环境和急停/STO 状态。

真机模式先在不记录 PDO 的情况下移动到返回位（左臂全零，右臂第 1 轴 30°、第 2 轴 15°）
并等待 `interval_sec`，随后同步开启左右臂 PDO 记录。正式循环的每一轮包含
“返回位 → 配置目标 → 返回位”，默认在第二轮完整结束后停止，因此 CSV 严格覆盖
两个完整往返，不包含启动时当前位置到返回位的预备运动。
可用 `pdo_record_cycles` 增加记录轮数，或显式关闭记录：

```bash
ros2 launch zhishu_robot_sdk_cpp sdk_joint_ethercat_loop.launch.py \
  loop_count:=10 pdo_record_cycles:=2

ros2 launch zhishu_robot_sdk_cpp sdk_joint_ethercat_loop.launch.py \
  loop_count:=10 record_pdo_diagnostics:=false
```

当任一臂记录服务无法启动时，程序不会进入正式循环，但启动预备运动已经执行；
若只有一侧启动成功，程序会自动回滚，避免左右 CSV 对应不同的运动时间窗。
当前左右臂混合 mock/真机时只存在真机侧的 PDO 服务，不能开启双臂同步 PDO
记录；`record_pdo_diagnostics:=false` 仅关闭录制，不会禁止预备和循环运动。
两臂均为 mock 且设置 `use_fake_hardware:=true` 时会自动跳过 PDO 服务。
记录完成后可运行：

```bash
ros2 run zhishu_robot_hardware analyze_pdo_diagnostics.py \
  /path/to/pdo_master0_*.csv /path/to/pdo_master1_*.csv
```

分析结果中的 `stable_samples` 只统计 WC 完整且驱动器处于 Operation Enabled 的样本，
`moving_samples` 则进一步要求目标位置变化或实际速度超过噪声阈值。新生成的 CSV 同时
保留驱动器原始值，并提供按当前关节配置换算的 `joint_velocity_rad_s` 和
`joint_effort_nm`。位置数据同时包含本周期写入驱动器的
`sent_target_position_joint_rad`、同周期接收的
`feedback_actual_position_joint_rad` 和 `position_error_joint_rad`；环绕关节使用与
ros2_control 相同的连续位置分支，并在 `position_conversion_offset_rad` 中记录每个
样本实际使用的分支偏移。

### Python SDK

新增 `zhishu_robot_commander_py`。Python 端只作为 ROS 2 client，不重复加载 MoveIt；
该 Python SDK 仍对接**原包**的 `/zhishu_robot_commander/status_structured`；本次没有修改该 Python SDK。

```bash
colcon build --symlink-install --packages-up-to zhishu_robot_commander_py
```

普通 MoveIt bringup 激活的是覆盖双臂 14 轴的 `dual_arm_controller`，不要同时
激活左右单臂控制器，以免争用相同的 position command interface。
