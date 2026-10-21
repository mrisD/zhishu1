# SDK 工具/工件坐标系与 ABB 风格接口

## 已确认的规则

- `tool0`：各臂法兰的零偏移坐标系，左臂 `L_hand_link`，右臂 `R_hand_link`。
- `wobj0`：机器人 `base_link`，零偏移。
- 普通位姿、`MoveJ`、`MoveL` 默认 `tool0/wobj0`。
- **兼容例外**：现有 `move_tcp_pose` / `moveTcpPose` 不传工具时，仍使用配置的夹爪 TCP（默认 `left_gripper_tcp` / `right_gripper_tcp`）；显式传 `tool0` 则使用法兰。
- 每条指令独立传入工具、工件名称；没有持续生效的“当前工具/当前工件”。未知名称直接报错，不静默回退。双臂旧位姿接口可以分别指定两套名称。
- 默认坐标配置为空，没有导入任何 Gazebo/TCP/工件标定结果。构造 SDK 或加载坐标文件不会自行执行运动。

## 单位、语义和边界

接口名称参考 [ABB RAPID 官方手册](https://library.e.abb.com/public/b227fcd260204c4dbeb8a58f8002fe64/Rapid_instructions.pdf)，不是 RAPID 解释器，也不是可直接复制 ABB 指令的兼容层。

**统一使用现有 ROS/SI 单位：位置和偏移为米，关节和旋转为弧度，四元数顺序 xyzw。** ABB 手册示例的毫米、度必须显式换算，例如 10 mm 写为 `0.010`，90° 写为 `pi/2`，不能直接照搬 `100`、`90`。

| 接口 | 输入和行为 |
|---|---|
| `MoveJ` | 单臂笛卡尔目标，按所选工具/工件换算到法兰后做关节空间规划；不保证 TCP 路径直线。 |
| `MoveL` | 单臂笛卡尔目标，规划所选 TCP 的直线位置路径和姿态插值；工具有偏移或同时转姿态时也按 TCP 生成法兰路点。 |
| `MoveAbsJ` | 绝对关节角：单臂 7 个，双臂按左 7 + 右 7 共 14 个，单位 rad；工具/工件名称仍校验，但不会对绝对关节角做坐标偏移。 |
| `Offs` | 返回新目标，沿目标所处工件坐标轴平移，姿态不变，不发送运动。 |
| `RelTool` | 返回新目标，先按原目标的工具轴平移，再按工具 X、新 Y、新 Z 顺序旋转，不发送运动。 |

`RelTool` 实现 `p' = p + R*[dx,dy,dz]`、`R' = R*Rx*Ry*Rz`。多轴旋转遵循该手册 RelTool 条目的局部旋转次序，不能误用固定轴 RPY 的 `Rz*Ry*Rx`。`Offs`/`RelTool` 保留输入参考坐标/时间戳，不修改原目标，可以相互嵌套。目标必须已经表达在后续运动采用的工件坐标中；它们不会自动查询当前机器人姿态或选择工件。

运动由现有 MoveIt 规划、碰撞检测、控制器执行。这里的 MoveJ 是关节空间规划，不承诺复现 ABB 控制器的特定同步插补时序；尚不支持 ABB `v100`、`fine/z50` 区域融合、载荷/惯量、外部轴、运动工件跟踪或完整 `robtarget` 配置分支。定义 TCP 偏移不会自动生成工具碰撞几何，实际工具仍须正确建模。

## Python

```python
from zhishu_robot_commander_py import Commander, PoseTarget, Offs, RelTool

# p、joints_rad 必须由应用提供并检查。默认不执行任何运动。
def program(robot, p, joints_rad, execute=False):
    if not execute:
        return
    robot.MoveJ('right', p, tool='tool0', wobj='wobj0').raise_for_error()
    robot.MoveL('right', Offs(p, 0.010, 0, 0), tool='tool1', wobj='wobj1').raise_for_error()
    robot.MoveL('right', RelTool(p, 0, 0, 0.005, rz_rad=0.05),
                tool='tool1', wobj='wobj1').raise_for_error()
    robot.MoveAbsJ('right', joints_rad, tool='tool0', wobj='wobj0').raise_for_error()
```

`Commander` 同时提供 `move_j`、`move_l`、`move_abs_j` 命名。旧接口 `move_pose(..., tool=..., wobj=...)`、`move_tcp_pose(...)` 同样支持逐条指定。双臂接口参数为 `left_tool/right_tool/left_wobj/right_wobj`。新 Python 位姿接口需配套升级 C++ bridge 和 interfaces 包；旧 ROS 话题入口仍保留。

```python
robot.load_coordinate_frames('/absolute/server/path/manual_frames.yaml').raise_for_error()
result, definitions_yaml = robot.get_coordinate_frames()
result.raise_for_error()
robot.save_coordinate_frames('/absolute/server/path/new_frames.yaml').raise_for_error()
```

文件路径位于 **commander 服务器**，不是 Python 客户端电脑。加载采用完整替换、失败保持原定义；运动中加载/保存返回 BUSY。保存拒绝覆盖已有文件。服务调用超时后阻止该客户端继续发送新运动，直到迟到结果返回；排队过期的运动请求不启动。队列截止时间基于客户端/服务器墙上时钟，两机需同步时间。

## C++ RobotSdk

```cpp
sdk.MoveJ(ArmGroup::kRight, p, "tool0", "wobj0");
sdk.MoveL(ArmGroup::kRight, Offs(p, 0.010, 0, 0), "tool1", "wobj1");
sdk.MoveL(ArmGroup::kRight, RelTool(p, 0, 0, 0.005, 0, 0, 0.05), "tool1", "wobj1");
sdk.MoveAbsJ(ArmGroup::kRight, joints_rad, "tool0", "wobj0");
```

也可使用 `RobotSdk::Offs` / `RobotSdk::RelTool`。旧同步/异步位姿接口新增末尾 `MotionFrames{tool,wobj}` 参数，旧调用形式保持源码兼容；新增成员改变二进制 ABI，部署时应重新编译客户 C++ 程序，不能直接替换正在运行的旧库。

## 手工坐标文件

`config/coordinate_frames.yaml` 是空模板，不自动加载。启动可传 `coordinate_frames_file:=/absolute/path/file.yaml`，或运行后通过服务加载。以下大写量是占位符，必须替换成真实人工提供的数据，不能原样加载：

```yaml
version: 1
tools:
  left: {}
  right:
    tool1:
      translation_xyz_m: [X_M, Y_M, Z_M]
      orientation_quaternion_xyzw: [QX, QY, QZ, QW]
workobjects:
  wobj1:
    translation_xyz_m: [X_M, Y_M, Z_M]
    orientation_quaternion_xyzw: [QX, QY, QZ, QW]
```

工具定义相对其所属臂的法兰；工件定义相对 `base_link`。工件也可以用 `user_frame` + `object_frame` 两组相同格式的位姿，运行时取二者乘积；保存输出等价的合成工件位姿。四元数会归一化，零四元数、非有限数、重复名称、覆盖内置名称、未知字段和错误形状均拒绝。

显式指定非默认工件时，`PoseStamped.frame_id` 可为 `base_link`（旧目标类型默认标签）或该工件名称；坐标数值按显式 `wobj` 解释。其他不一致的标签拒绝。使用 `wobj0` 时，已有 `PoseStamped` 的非基坐标标签会经 TF 转换后执行，修复旧直线分支丢弃 `frame_id` 的问题。

## ROS 服务与软件验证

- `~/move_cartesian`：`zhishu_robot_interfaces/srv/MoveCartesian`，含每臂目标、工具/工件名、tcp/cartesian 模式和排队截止时间。
- `~/move_abs_j`：`zhishu_robot_interfaces/srv/MoveAbsJ`。
- `~/coordinate_frames`：`zhishu_robot_interfaces/srv/CoordinateFrames`，操作 `list/load/save`。

`test/commander_fake_validation.launch.py` 固定在域 169，启动前拒绝非 GenericSystem 的硬件描述，不启动 Gazebo，也不连接实际设备。`zhishu_robot_commander_py/test/validate_sdk_frames.py` 默认只检查环境，加 `--execute` 才做假硬件运动联调。测试用人工合成坐标只存临时目录，成功后清空注册表，不进入产品默认配置。该软件联调不等同于真机或动力学仿真验收。
