# Gazebo ABB TCP&Z,X 工具坐标系标定（双臂、多工具）

基于 `src(20261014-065607).zip`，ROS 2 Jazzy + Gazebo Harmonic + MoveIt 2。

## 安装与使用

代码已包含在工作区，直接构建和启动：

```bash
cd ~/zhul/guohua_ws
sudo apt install ros-jazzy-gz-ros2-control ros-jazzy-ros-gz-sim \
    ros-jazzy-moveit-ros-planning-interface libeigen3-dev
colcon build --symlink-install --packages-up-to zhishu_robot_tcp_calibration
source install/setup.bash
ros2 launch zhishu_robot_tcp_calibration tcp_auto_gazebo.launch.py \
    arm:=right seed_pose:=current tool:=probe \
    method:=multi_point samples:=12 noise:=ideal
```

左臂：

```bash
ros2 launch zhishu_robot_tcp_calibration tcp_auto_gazebo.launch.py \
  arm:=left seed_pose:=current tool:=probe \
  method:=four_point noise:=gaussian
```

其他选择：`tool:=long_probe`、`noise_seed:=123`、`seed_pose:=current`。

默认使用独立的 `ROS_DOMAIN_ID=67` 与 `GZ_PARTITION=tcp_calibration_sim`，
避免和示教或真机程序混用通信图。可通过 `ros_domain_id`、`gz_partition` 参数修改，
同一场景的后续采样与检查必须使用相同值。

排查启动时可设置 `auto_sample:=false`，先检查 `ros2 control list_controllers`
中的五个控制器均为 `active`，再执行采样。`paused:=true` 可暂停 Gazebo，
但控制器激活需要仿真时间步，解除暂停后应再次确认控制器状态。
采样程序自动等待三个必要控制器激活、关节反馈与工具 TF 就绪，超时则退出；
不再依赖固定等待 40 秒。`sampling_delay` 默认为 0，仍可显式增加延迟。
Gazebo、MoveIt 和自动采样节点的机器人描述均使用 `use_gazebo:=true`，
避免 `/robot_description` 的多个发布者将假硬件配置送入 Gazebo 控制插件。

默认使用 `config/tcp_initial_positions.yaml`：双肘 1 rad，探针伸到机身前方。
夹爪朝法兰负 Z，`probe` 的实际可视锥尖/TCP 为 `[0.015, 0, -0.30]` m，
工具针杆和锥尖带碰撞几何；锥尖网格顶点与 TCP 精确重合。固定参考针采用落地支架，
放置时转换到 Gazebo 世界坐标。
标定专用模型通过固定关节锚定世界，防止轮子滑动破坏固定针尖约束。

仅查看对尖而不驱动机械臂：先以 `auto_sample:=false` 启动，控制器激活后运行
`ROS_DOMAIN_ID=67 GZ_PARTITION=tcp_calibration_sim ros2 run zhishu_robot_tcp_calibration tcp_preview.py`。
复用同一个场景采样可用 `sampler_only:=true sampling_delay:=0.0`，避免重复启动 Gazebo。
同场景可更换 arm、method 或 noise；更换 tool 需要重新启动完整场景以更新机器人模型。
工装确认携带本轮请求时间戳，重复采样会更新工装和碰撞场景，不接受上一轮成功确认。
工装就绪后自动聚焦 Gazebo 相机；可在工装节点设置 `auto_focus:=false` 关闭聚焦。

输出到 `/tmp/guohua_tcp_calibration`，每次运行包含：
- `<arm>_<tool>_<method>_<timestamp>.yaml`：TCP XYZ、RPY、四元数、独立验收与连续路径误差指标
- `<...>_samples.csv`：拟合与独立验证采样
- `<...>_report.md`：误差/条件数/真值对比

使用 RViz：添加 `MarkerArray` 显示，topic 选择 `/tcp_calibration/markers`。
黄色为固定参考点，绿色为已标定 TCP，蓝色为仿真针尖真值。

## ABB TCP&Z,X 自动标定流程

1. 将机械臂运动到 `seed_pose`，用当前法兰 TF 和仿真探针几何计算参考尖点位置。
2. 在 Gazebo 的 `empty` 世界创建固定参考尖点，成功 ACK 前不会做后续标定运动。
3. 生成各轴旋转扰动候选（默认 0.32 rad），从当前到目标姿态插值旋转，逐点使用
   `p_flange = p_pivot - R_flange * t_truth` 构造保持针尖位置的笛卡尔路径。
   MoveIt 检查碰撞和完整路径比例；保守重新定时后再次检查关节跳变与计划针尖偏差。
   不完整或计划偏差超过 0.75 mm 的路径不会执行；执行中按 TF 监测针尖，偏差超过 2 mm 则停止。
4. 控制器执行成功后等待稳定，检测三次 TF 的漂移及针尖偏差。失败的采样被拒绝。
5. 至少 4 点，默认 12 点，用 SVD 求解 TCP 原点相对法兰的 XYZ 与固定尖点位置；检查秩、条件数、RMS。
6. 先抬到参考针上方 `150 mm + elongator_length_m` 的空旷区，记录参考法兰姿态；保持该姿态，分别沿预期工具 `+Z`、`+X` 方向平移 50 mm（可配置），记录两个 elongator 点。用已估计的 TCP 恢复三个点的位置，从参考点的位移求方向，构造右手系：`Y = Z × X`，再正交化 X。参考点不再依赖原固定工装位置。
7. 用未参与方向求解的 `+Y/-X/-Z` 目标在空旷区做独立位置和姿态验证；目标不可达时逐级缩短距离，并在日志中记录实际距离。
8. 通过规划回到原始对尖姿态，再输出相对 `L_hand_link`/`R_hand_link` 的完整 XYZ + 四元数 + RPY。

**不需要手动按回车或移动机械臂。**

**标定与验证隔离**：仿真工具几何由实际工具固定 TF 读取，不再在 C++ 重复硬编码；
这些真值仅用于生成模拟标定动作、轨迹监测和最终误差检查。
`solvePivot()` 与 `solveFrameOrientation()` 只接收法兰采样和已估计的 TCP。
`noise:=gaussian` 对拟合与方向采集输入注入噪声，不污染控制 TF。

工装针杆、底座和支柱具有 Gazebo 碰撞几何，并同步到 MoveIt 碰撞场景。
MoveIt 针尖附近保留 2 mm 接触容差，工装最尖端仍为可视模型，不用于接触力验收。
独立位置 RMS 超过 1 mm、位置真值误差超过 1.5 mm、方向真值误差超过 1°，
或工具坐标检查位置 RMS 超过 1 mm、姿态 RMS 超过 0.5°时，不输出成功结果。

## 注意

- **只用于 Gazebo，不能在真机执行**：程序检查 `use_sim_time=true`，但这并非独立的硬件安全隔离，必须由操作人员确保启动的是 Gazebo 仿真；不要把本功能节点并入真机 bringup。
- 现有机械臂、SRDF、控制器、原始 TCP 保留；新探针只在 `tcp_calibration_enabled:=true` 加载。
- `probe` 与 `long_probe` 是两种示例安装偏移，不是实物尺寸；修改 Xacro 后，采样程序从模型工具 TF 读取实际几何。
- 有效自动姿态受工作空间与 IK 约束；若 12 点采样不够，先尝试 `samples:=8` 或调整起始姿态和 `rotation_step_rad`。
- `probe`/`long_probe` 仿真真值包含非零工具 RPY，专门验证完整方向求解；它们只是仿真尺寸，不代表实物工具。
- 方向解退化（X/Z elongator 太短或近共线）时会拒绝输出成功。
- 自动功能一次只控制一只手臂，若要双臂分别标定，先后执行两条命令。
- 验证几何、避碰规划、控制跟踪和算法流程，不检验真实装配的接触刚性。
- `seed_pose` 默认 `current`，配合上述初始姿态；也可指定 SRDF 命名姿态，规划失败时停止并报告错误。
- Gazebo 仿真验收只反映模拟模型与算法流程，不代表实机精度或安全认证。
