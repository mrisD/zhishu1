# Gazebo 启动崩溃修复（2026-10-12）

## 根因与修复范围

本机 `ros-jazzy-gz-ros2-control` 为 1.2.20。GDB 在原版插件上复现
`GazeboSimSystem::write()+352` 的 SIGSEGV；故障指令读取空指针。
反汇编与同版本源码对应到 mimic 主指 `JointPosition::Data()[0]`：
Gazebo 已创建位置组件，但第一帧物理更新前，组件的数组仍为空。
原代码没有检查数组长度。普通 `read()` 也存在同类边界风险。

本地补丁仅在组件不存在或位置/速度数组为空时等待后续更新，保持
mimic、实际运动、碰撞几何和接触检查不变。不修改 `/opt/ros`。

源码固定到官方 tag 1.2.20 的提交
`89ca76ed20842377f879c711c756d444c5b7c20b`：
[官方源码](https://github.com/ros-controls/gz_ros2_control/blob/89ca76ed20842377f879c711c756d444c5b7c20b/gz_ros2_control/src/gz_system.cpp)。

## 使用

在工作区根目录执行：

```bash
bash scripts/build_gz_control_safe_overlay.sh
bash scripts/run_with_gz_control_safe_overlay.sh \
  python3 scripts/test_pallet_side_push.py --col 2 --rear-tip \
  --physical --transfer --preload-robot --physics-engine dartsim \
  --suffix _safe_control_new_test
```

构建输出位于 `output/gz_control_safe_overlay`。包装脚本先加载工作区，
再加载修复插件，并设置 Gazebo 插件搜索路径。补丁保存于
`scripts/patches/gz_ros2_control_1.2.20_empty_joint_state.patch`。

预加载测试必须使用 `spawn_robot:=false`，避免再次生成机器人造成两个
同名控制器管理器冲突。三个相关 launch 文件已增加此参数；默认仍为
`true`，普通运行时生成模型方式不变。

## 验证证据

- 原版复现：`output/pallet_middle_rear_push_20261009_lattice_crash_debug_20261012/scene.log`。
- 修复后单实例：`output/pallet_middle_rear_push_20261009_lattice_safe_control_20261012_dart2/`。
- 修复后的“Waiting for initialized mimic joint positions”警告说明启动保护被实际触发。
- 单实例场景的五个控制器已激活；`/proc` 映射确认实际加载的是本地修复库，而不是系统原版库。
- 搬运动作结果需单独检查 `probe.log`，不能由控制器启动成功推断搬运成功。

## 复测结论

- DART 独立启动及开合回归通过：`output/pallet_middle_rear_push_20261009_lattice_safe_control_20261012_smoke1/startup_smoke_result.json`。
- Bullet Featherstone 同组回归通过：`output/pallet_middle_rear_push_20261009_lattice_safe_control_20261012_smoke2_bullet/startup_smoke_result.json`。
- 每轮实际执行 20 mm → 0 → 40 mm → 0 四个指令，检查动作结果和主/从指位置；容差 5 mm。Bullet 最大目标误差约 0.031 mm，主从反向误差小于 0.001 mm。
- 两轮运行中均未复现 Gazebo 的 write 崩溃；主动结束测试时 Gazebo 的 −2 退出码来自 SIGINT，不是自发崩溃。
- 搬运复测的反向接近搜索到第 14 步未找到满足连续性限制的姿态，程序主动报告失败。Gazebo 没有因此崩溃；不能宣称完整搬运成功。
- **仍有独立的 MoveIt 退出故障**：关闭时 `move_group` 的 Executor 析构出现 SIGSEGV（−11）。本次修复解决的是 Gazebo 控制插件运行时空数组访问，并没有修复该退出问题。

状态：Gazebo 已知启动 write 崩溃已修复并通过两种引擎开合回归；完整搬运仍未通过。测试进程均已退出，未提交代码。
