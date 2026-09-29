# 15 kg 小料箱预抽出、双臂抓取与大料箱放置验证方案

**验证日期：** 2026-09-29
**验证环境：** Ubuntu 24.04、ROS 2 Jazzy、Gazebo Sim 8.15、MoveIt 2
**验证模式：** Gazebo headless，`use_rviz:=false`，不启动 Gazebo 图形界面

## 1. 验证目标

验证机器人在以下约束下完成一个外围 15 kg 小料箱的完整转运：

1. 机器人底盘停在托盘前约 0.20 m 处；
2. 小料箱之间的相邻间隙约为 2–4 mm；
3. 先清理外围箱，再处理中心箱；
4. 夹爪不能从密排箱体左右侧水平插入；
5. 通过真实提手孔从上方斜向进入，钩拉外围箱；
6. 预抽出后重新定位，再由双臂抓取 15 kg 小料箱；
7. 将小料箱放入桌面高度为 0.55 m 的大料箱中，并释放。

## 2. 验证方案

### 2.1 外围箱预抽出

验证对象为 `pallet_box_r0_c0`。右臂采用钩拉方式，不要求 TCP 水平夹住外侧 rail：

- 底盘预停靠位置：约 `(0.190, -0.401, 0.000)`；
- 腰部姿态：升降 `0.040 m`，俯仰 `-0.700 rad`；
- 提手孔进入方向：负 Y 侧，从上方斜向进入；
- 名义提手孔 TCP 航点：
  - `(0.524, -0.257, 0.387)`：孔外预姿态；
  - `(0.524, -0.187, 0.377)`：进入孔内；
  - `(0.524, -0.187, 0.377)`：落座/钩拉姿态；
  - `(0.524, -0.307, 0.377)`：向外拉出；
  - `(0.524, -0.257, 0.507)`：抬高脱离。
- 姿态搜索优先使用 `RPY=(-1.571, -0.550, 0.000)`；代码还保留多个俯仰和 ±12 mm TCP 小范围候选。

预抽出轨迹完成后，按 `pre_extract_distance:=0.12` 的仿真位移重新计算箱体中心。

### 2.2 双臂抓取与搬运

预抽出后将箱体中心更新为视觉/仿真重定位结果，再执行双臂对称抓取：

- 源箱重定位中心：约 `(0.704, -0.521, 0.224)`；
- 抓取腰部姿态：升降 `0.040 m`，俯仰 `-0.700 rad`；
- 双臂同时闭合夹爪并完成抬升；
- 搬运时腰部升降 `0.180 m`、俯仰 `-0.100 rad`，避免托盘和周边箱体碰撞。

### 2.3 0.55 m 桌面大料箱放置

基座沿桌下通道移动，最终停靠约为：

```text
(0.700, -1.526, -0.034)
```

放置阶段使用桌面高度 `0.55 m`、大料箱中心 `(1.03, -1.55)`，箱体下降到大料箱内部后打开双臂夹爪，使小料箱依靠接触和重力落入。

## 3. 可复现命令

先编译相关包：

```bash
cd /home/zhul/online/guohua_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install \
  --packages-select zhishu_robot_moveit_config zhishu_robot_mtc_pick_place
source install/setup.bash
```

运行完整 headless 验证：

```bash
ROS_DOMAIN_ID=225 GZ_PARTITION=codex_pallet_final225 \
timeout 900s ros2 launch zhishu_robot_mtc_pick_place \
  pallet_bin_15kg_transfer.launch.py \
  headless:=true \
  use_rviz:=false \
  execute:=true \
  validate_all_nine:=false \
  full_transfer_box_index:=0 \
  pre_extract_enabled:=true \
  skip_pre_extract_stage:=false \
  pre_extract_box_index:=0 \
  pre_extract_distance:=0.12 \
  handle_z_offset:=0.22 \
  pregrasp_clearance:=0.04 \
  source_grasp_x:=0.45 \
  source_grasp_y:=-0.68 \
  grasp_waist_pitch:=-0.70 \
  2>&1 | tee /tmp/pallet-final-real225.out
```

## 4. 通过标准与实际结果

| 检查项 | 结果 | 日志证据 |
|---|---|---|
| 右臂提手孔预抽出规划与执行 | 通过 | `right-arm pre-extraction ... execution completed` |
| 预抽出后箱体重新定位 | 通过 | `Relocalized ... world center=(0.704, -0.521, 0.224)` |
| 双臂抓取 15 kg 小料箱 | 通过 | `full-transfer pick ... execution completed` |
| 基座进入桌下放置位 | 通过 | `Base docked at (0.700, -1.526, -0.034)` |
| 放入 0.55 m 桌面大料箱并释放 | 通过 | `large-bin insertion ... execution completed` |
| 双臂回零与任务收尾 | 通过 | `Complete transfer finished` |

完整日志：[`/tmp/pallet-final-real225.out`](/tmp/pallet-final-real225.out)

## 5. 代码与模型调整

- [`pallet_bin_15kg_transfer.cpp`](../src/zhishu_robot_mtc_pick_place/src/pallet_bin_15kg_transfer.cpp)：
  - 提手孔斜向航点和姿态/位置候选搜索；
  - 每次 IK 查询复用 `PlanningScene`，避免重复构造碰撞场景；
  - 当前关节状态边界归一化，避免控制器量化误差触发 `Start state is out of bounds`；
  - 预抽出后重定位、双臂抓取、桌下停靠和大料箱放置。
- [`pallet_bin_15kg.sdf`](../src/zhishu_robot_gazebo/worlds/pallet_bin_15kg.sdf)：增加外围箱预抽出支撑/导向几何。
- [`gripper.xacro`](../src/zhishu_robot_description/urdf/gripper.xacro)：放宽夹爪手指的仿真边界，避免闭合时的边界误差。
- [`zhishu_robot.srdf`](../src/zhishu_robot_moveit_config/config/zhishu_robot.srdf)：补充弯腰姿态、夹爪和底盘相关碰撞处理。
- [`moveit_controllers.yaml`](../src/zhishu_robot_moveit_config/config/moveit_controllers.yaml)：将双臂执行统一到 `dual_arm_controller`，避免选中未激活的单臂控制器。

## 6. 当前限制与下一步

本次运行没有收到 `/pallet_bin/target_pose` 的新视觉消息，因此日志提示使用了 Gazebo 中按 `pre_extract_distance` 推算的位移作为仿真回退值。也就是说，钩拉、双臂抓取、搬运和放入已经通过 Gazebo 物理接触验证，但真实 RGB-D 提手孔检测和预抽出后的视觉重定位仍需要启动对应 perception 节点后单独验收。

真实部署前应至少补做：

1. 发布真实提手孔/箱体中心到 `/pallet_bin/target_pose`；
2. 验证视觉坐标与 `base_footprint` 的外参和时间戳；
3. 低速执行一个外围箱，确认夹爪钩拉力、箱体滑移和 15 kg 负载下的夹持安全裕量。
