# 双臂箱体导纳控制

本包用于左右机械臂共同搬运箱体。左右腕部各使用一个六维力/力矩传感器，
控制器将两个传感器的数据转换到 `base_link` 后分解为：

- 共同外力：驱动箱体整体柔顺平移和旋转；
- 左右内力差：生成方向相反的微调速度，使两只手负载趋于一致；
- 参考速度：用于主动抬升、平移和放下箱体，导纳修正叠加在参考速度上。

控制方程为：

```text
M * x_ddot + D * x_dot + K * x = F_external - F_desired
```

## 仿真组成

- Gazebo Harmonic 世界：`dual_arm_box_admittance.sdf`；
- 中空箱外形尺寸：`0.30 × 0.40 × 0.15 m`（X × Y × Z），壁厚 `0.01 m`；
- 腕部六维力传感器：`/left_wrist_ft`、`/right_wrist_ft`；
- 双 MoveIt Servo：分别控制左右夹爪 TCP；
- 导纳控制节点：`dual_arm_admittance`；
- 调试输出：共同力 `/dual_arm_admittance/common_wrench`，内力差
  `/dual_arm_admittance/internal_wrench`。

## 编译和启动

在工作区根目录执行：

```bash
cd ~/guohua_ws
rosdep install --from-paths src --ignore-src -r -y --rosdistro jazzy
colcon build --symlink-install --packages-up-to zhishu_robot_admittance
source install/setup.bash

ros2 launch zhishu_robot_admittance dual_arm_box_admittance_sim.launch.py
```

无显示器的远程 Ubuntu 使用：

```bash
ros2 launch zhishu_robot_admittance dual_arm_box_admittance_sim.launch.py \
  headless:=true use_rviz:=false
```

本演示默认启用 VMware/Wayland 兼容渲染（X11、Ogre、Mesa 软件 OpenGL），用于
避免 Gazebo 窗口闪烁或黑屏。若在具有正常显卡驱动的实体 Ubuntu 上运行，可恢复
硬件加速：

```bash
ros2 launch zhishu_robot_admittance dual_arm_box_admittance_sim.launch.py \
  force_x11:=false software_rendering:=false render_engine_gui:=ogre2
```

启动后先确认两个传感器都有数据：

```bash
ros2 topic hz /left_wrist_ft
ros2 topic hz /right_wrist_ft
ros2 topic echo --once /dual_arm_admittance/common_wrench
```

控制器启动时自动采集 100 对互相同步、且左右两侧都已更新的样本去皮，在这期间
始终输出零速度。默认要求左右样本时间戳相差不超过 `0.02 s`；超时、不同步、
TF 丢失或出现 `NaN/Inf` 时会输出零速度并清空导纳动态状态。

默认启动会复用 MTC 求得的上沿夹持 IK，将两只夹爪放在左右侧壁中心，并以同一个
轨迹起始时间在 4 秒内同步闭合到 `0.054 m`。低冲击闭合成功后，程序会按 MTC 的
处理方式在 `L_hand_link` 与料箱之间动态建立可拆卸关节，避免仅靠接触摩擦搬运时
出现对拉、抖动或箱体飞出。导纳节点会等待夹持和附着完成、接触稳定后再启动去皮。

调试时可以关闭自动夹持，或调整闭合位置和持续时间：

```bash
ros2 launch zhishu_robot_admittance dual_arm_box_admittance_sim.launch.py \\
  auto_grasp:=false

ros2 launch zhishu_robot_admittance dual_arm_box_admittance_sim.launch.py \\
  gripper_position:=0.050 grasp_duration:=6.0
```

如需只检查 Gazebo 接触、不建立搬运连接，可设置 `attach_box:=false`。

自动夹持完成后控制器会自动去皮；确认箱体稳定后再启用导纳：

```bash
ros2 service call /dual_arm_admittance/tare std_srvs/srv/Trigger "{}"
ros2 service call /dual_arm_admittance/enable std_srvs/srv/SetBool "{data: true}"
```

自动执行沿 `base_link` 的 +Z 方向、梯形速度的 200 mm 单段直线搬运，
以及停止保持验收。双臂导纳默认贡献 40 mm，腰部升降轴按同一梯形进度
同步贡献 160 mm；协调运动可避开腕关节限位和水平奇异区，并且不会与
箱体支撑台产生水平摩擦：

```bash
ros2 run zhishu_robot_admittance run_admittance_validation
```

直线距离、峰值速度和方向可通过参数覆盖；默认验收范围为 200±10 mm，
最大横向偏离不超过 5 mm：

```bash
ros2 run zhishu_robot_admittance run_admittance_validation --ros-args \
  -p linear_distance_m:=0.200 -p linear_speed_mps:=0.020 \
  -p arm_linear_distance_m:=0.040 -p linear_axis:=z \
  -p linear_direction:=1.0
```

### 离地 20 cm 箱体测试

低位场景与上述常规场景分开，常规启动不变。低位箱体外形为
`0.30 × 0.40 × 0.20 m`，箱底在离地 `0.20 m` 的支撑台上，箱体中心在
`x=0.42 m, y=0`。腰部俯仰预置为 URDF 下限 `-0.7854 rad`（约 −45°）；
两臂初始关节位由这个腰部姿态和箱体上沿抓取点求逆运动学得到。该场景将夹爪
闭合目标设为 `0.055 m`，而常规场景仍为 `0.050 m`。

```bash
ros2 launch zhishu_robot_admittance dual_arm_box_admittance_sim.launch.py \
  scenario:=low_box headless:=true use_rviz:=false
```

等自动夹持、附着和力传感器去皮完成后，在另一终端运行：

```bash
ros2 run zhishu_robot_admittance run_admittance_validation --ros-args \
  -p arm_linear_distance_m:=0.0 \
  -p cross_track_feedback_gain:=3.0 \
  -p output_path:=/tmp/zhishu_low_box_validation.json
```

该测试让腰部升降轴贡献全部 `0.20 m`，两臂通过箱体实测位置反馈补偿 X/Y
偏移；横向反馈速度每轴限制在 `0.02 m/s`。不启用反馈时，该低位姿态的
横向偏移约 `12 mm`，超过 `5 mm` 的验收阈值。2026-09-26 的本机
Gazebo 测试中，箱体实际抬升 `0.1985 m`，最大横向偏移 `0.00057 m`，
其余间距、姿态、停稳与传感器检查均通过。这里验证的是仿真中的夹持抬升，
不是水平转运、放置或真机验证。Gazebo 仍使用左手到箱体的可拆卸固定关节；
通过该测试不代表单靠双侧摩擦夹持就能承载箱体，真机使用前还需验证实际
夹爪接触、负载和碰撞余量。

## 箱体共同运动命令

### 复用九箱场景，仅测试首箱

九箱场景的目标为 `pallet_box_r0_c0`，不是单箱世界的 `carry_box`。
先用 `pallet_bin_15kg_transfer.launch.py` 的 `admittance_handoff:=true`、
`full_transfer_box_index:=0`、`validate_all_nine:=false`、
`keep_all_source_rows:=true` 完成首箱抓持。看到 `ADMITTANCE_HANDOFF_READY`
后，MTC 不再执行底盘搬运和放置。整个过程保留其余八箱。

所有命令必须使用场景相同的 `ROS_DOMAIN_ID` 和 `GZ_PARTITION`。
九箱世界需加载 `gz-sim-forcetorque-system`，并确认两个腕部力话题有数据。
可在已经运行的世界加载：

```bash
gz service -s /world/pallet_bin_15kg/entity/system/add \
  --reqtype gz.msgs.EntityPlugin_V --reptype gz.msgs.Boolean --timeout 5000 \
  --req 'plugins { name: "gz::sim::systems::ForceTorque" filename: "gz-sim-forcetorque-system" }'
ros2 launch zhishu_robot_admittance pallet_first_box_admittance.launch.py
```

导纳节点默认禁用。待 MTC 已停止、首箱抓稳后，加载并配置左右臂控制器，
严格切换 `dual_arm_controller` 到 `left_arm_controller` 和
`right_arm_controller`，再将两侧 Servo 的 `switch_command_type` 设为 1。
不要同时激活双臂轨迹控制器和两个单臂控制器。

仅双臂、50 mm、5 mm/s 的竖直导纳测试：

```bash
ros2 run zhishu_robot_admittance run_admittance_validation --ros-args \
  -p world_name:=pallet_bin_15kg -p model_name:=pallet_box_r0_c0 \
  -p linear_distance_m:=0.050 -p arm_linear_distance_m:=0.050 \
  -p linear_speed_mps:=0.005 -p linear_axis:=z \
  -p stop_on_measured_distance:=true -p linear_distance_tolerance_m:=0.002 \
  -p output_path:=/tmp/pallet_first_box_admittance.json
```

验收测量真实 Gazebo 箱体位移、横向偏差、姿态、TCP 间距与停止漂移。
这段测试不执行第二个箱子，也不代表完成到收料箱的全程搬运。

自动完成交接和导纳验收可运行工作区脚本
`scripts/test_pallet_first_box_admittance.py --scene-log <场景日志> --output-dir <结果目录>`。
它等待首箱的 `ADMITTANCE_HANDOFF_READY`，加载双 Servo、严格切换控制器，
执行仅双臂导纳抬升，请求 50 mm、参考速度 5 mm/s，根据箱体实测位移停止，
按 ±2 mm 验收，保留 Servo 的奇异性、限位和碰撞保护。
验收结束后禁用导纳，保留 Gazebo 场景和导纳节点。仿真服务与 `gz sim -g`
显示窗口应分开启动，避免显示窗口退出影响仿真。仿真计时使用 `/clock`，
允许低于实时速度运行；连续 5 秒没有时钟进展才按仿真停止退出。

参考速度必须使用 `base_link` 坐标系。下面命令让箱体以 `0.015 m/s` 向上运动；
停止发布 0.25 秒后，参考速度会自动归零：

```bash
timeout 5s ros2 topic pub -r 50 \
  /dual_arm_admittance/reference_twist geometry_msgs/msg/TwistStamped \
  "{header: {frame_id: base_link}, twist: {linear: {z: 0.015}}}"
```

向前搬运：

```bash
timeout 5s ros2 topic pub -r 50 \
  /dual_arm_admittance/reference_twist geometry_msgs/msg/TwistStamped \
  "{header: {frame_id: base_link}, twist: {linear: {x: 0.02}}}"
```

停止并清除导纳状态：

```bash
ros2 service call /dual_arm_admittance/enable std_srvs/srv/SetBool "{data: false}"
```

导纳启用后，如果任一力传感器超时、左右时间戳不同步、TF 不可用或输入包含
`NaN/Inf`，节点会立即输出零速度并锁存为禁用。数据恢复后不会自动继续运动，
需排除原因并再次调用 `enable` 服务。

## 参数调整

### 只用腕部六维力估测箱底反作用力

首箱放置默认使用大箱中心 `bin_target_x:=1.03`、Y=`-1.55 m`。
底盘停靠 X 同步调整为 `bin_target_x - 0.48`，保持经过验证的相对抓取距离，
避免仅移动放置目标导致腕关节达到限位。居中目标对应停靠 X=`0.55 m`。

可用 `scripts/record_pallet_wrist_forces.py --scene-log <场景日志> --output-dir <记录目录>`
连续保存两腕原始六维数据。控制器同时在持箱、插入完成、承重转移、支撑确认、
开始松爪（夹爪位置首次小于 0.04 m）及接近全开（两侧均小于 0.001 m）输出
`WRIST_FT_STAGE`，包含两腕采样时间、六维读数、夹爪位置和估测支撑力。
导出的 `key_stage_forces.csv` 力单位为 N、力矩为 N·m，数值位于各自传感器
局部坐标系，不能直接把左右原始 Fz 或力模长相加来当作箱重。

九箱世界不再包含 `tote_bottom_contact` 箱体接触测力传感器。
`estimate_bottom_reaction` 将两腕测得的三轴力旋转到世界坐标系，补偿
两侧夹爪总计 0.56 kg 的自重，按低速准静态力平衡计算：

```text
N_est = (m_box + m_tools) * g + F_left_world_z + F_right_world_z
```

Gazebo 腕部传感器报告负载对手腕的力，不能把两个力的模长相加。
默认箱重 15 kg、每侧工具 0.28 kg；悬空静止时 `N_est ≈ 0 N`，
箱体完全受支撑且夹爪卸载时约为 `147.15 N`。
该估计器独立于导纳去皮，**不能在持箱时去皮后再估计绝对支撑力**。
它检查六轴数据有限性、两腕同步、TF 和底盘姿态新鲜度，并以 2 Hz 低通滤波。
输出 `/pallet_bin/estimated_bottom_reaction` 的 `force.z`，坐标系为 `world`；
其他分量不表示已估计出接触力矩。

`pallet_bin_15kg_transfer.launch.py` 的新场景自动启动估计器。复用已有场景时可单独启动：

```bash
ros2 run zhishu_robot_admittance estimate_bottom_reaction --ros-args -p use_sim_time:=true
```

`seatToteOnBinFloor` 使用这个估计，近底板阶段按虚拟阻尼
`D*v = 147.15 - N_est` 生成下压微步，单步不超过 0.25 mm。
放爪前要求支撑至少 90% 箱重、估计新鲜，并同时满足底部高度、平放姿态和
闭爪检查。原始接触力话题不参与控制。

这是准静态估计：加速阶段的惯性力可能产生短暂负值；不做非负截断来掩盖这些误差。
腕力本身只能给出环境总竖直支撑，结合底板几何与碰撞余量推断箱底支撑，
不能独立分辨侧壁摩擦与底板法向力。模型质量和传感器符号必须与实际系统一致。
验收时加 `-p require_reaction_estimate:=true` 可要求估计流持续有效，并记录估计范围。

参数文件为 `config/admittance.yaml`：

- `mass`：虚拟质量，越大响应越慢；
- `damping`：虚拟阻尼，抑制振荡；
- `stiffness`：虚拟刚度，决定稳态位移；
- `wrench_deadband`：力和力矩死区；
- `max_sensor_skew_sec`：允许的左右传感器最大采样时间差；
- `internal_balance_gain`：左右负载均衡速度增益；
- `max_internal_balance_velocity`：负载均衡修正速度上限，防止两臂对拉；
- `max_common_wrench`：进入虚拟动力学前的共同力/力矩限幅；
- `max_admittance_displacement`：导纳偏移上限；
- `max_linear_velocity`、`max_angular_velocity`：最终命令限速；
- `left_wrench_sign`、`right_wrench_sign`：真实传感器方向与仿真相反时逐轴改为
  `-1.0`。

真实机械臂第一次使用时应从较大的虚拟质量、阻尼和较低速度上限开始。必须保留
急停、关节限位、碰撞检测和传感器超时保护；不要在未夹稳箱体时启用导纳运动。

### 居中放箱的桌面接触修正（2026-10-16）

目的地桌子规划模型与 Gazebo 一致：桌面加四条桌腿，不再使用桌面下方的实心体。
`destination_table` 对全机器人恢复碰撞检查，包含腰部。居中停靠前协调抬腰到至少
0.28 m，放置阶段保持该高度，避免升降连杆下沿压在 0.55 m 桌面上。
`recover_table_clearance:=true` 仅用于已经检查过的 0.24–0.28 m 腰部接触状态：
保持实际左右 TCP 位姿、闭爪和底盘位置，单调向上脱离到 0.28 m，随后恢复腰部与桌面检查。
底部导纳微调使用当前 FK 姿态仅作竖直平移，不重建名义抓取姿态。

复用现场时必须使用与现场一致的夹指长度；本次九箱场景为
`tote_finger_length:=0.36`，不能沿用通用 launch 的 0.20 m 默认值。
现场恢复完成后箱子中心误差 0.735 mm，完整状态碰撞检查通过，腰部桌面间隙约
30.56 mm。松爪前估测反作用 243.26 N，仍有约 96.11 N 向下预载；松爪后为
147.15 N。尚需优化预载；不要通过提高 300 N 上限掩盖冲击。
完整恢复读数与限制见 `output/pallet_center_wrist_20261016/table_clearance/report.md`。
进桌前预防逻辑编译通过，但尚未从托盘起点完整重跑。

### 首箱完整流程：必须先侧拉

一键测试保留九个物理箱子，只操作第一个：**右手侧拉约 120 mm → 重新定位 →
双手抓取抬升 → 搬运至大箱正中间 → 腕力导纳落座 → 卸夹紧预载 → 验证承重 → 松爪回撤**。

```bash
cd /home/dky666/zhul/codex_guohua_ws
python3 scripts/run_pallet_first_box_wrist_placement.py \
  --output-dir output/first_box_wrist_placement_new --domain-id 182
```

该脚本生成新场景，统一 360 mm 夹指和 300 mm TCP 偏置，使用工作区已有的
Gazebo 控制兼容 overlay，并打开 Gazebo 窗口。结束后场景保持打开，Ctrl+C 关闭。
`--output-dir` 必须为尚不存在的目录；并行场景应选择不同 `--domain-id`。

落座现在使用**双向**竖直导纳：支撑不足时向下微调，压力过大时向上卸力；
接触附近单步最多 50 µm。检测底板接触后先将夹爪从受载夹紧位置小幅退至
0.053 m，完成卸预载。只有支撑估计位于箱重的 90%–110%（132.435–161.865 N）、
箱心距底板目标不超过 1.5 mm、倾斜不超过 0.5°、反馈新鲜且稳定后才允许完全松爪。
保留原有 [-30, 300] N 力保护，不提高上限。

运动前核对规划 TCP 与现场 TF，尺寸不匹配时停止。独立桌面监测不依赖规划的
碰撞放行矩阵，使用实测底盘姿态和全部机器人碰撞几何核对桌面、四条桌腿。
数据包括 `wrist_force_trace.jsonl`、`key_stage_forces.csv` 和 `table_clearance_summary.json`。
完整场景可用 `scripts/finalize_pallet_first_box_wrist_placement.py --output-dir <目录>`
在相同 `GZ_PARTITION` 环境下核对实际九箱位姿和释放顺序。

微步执行还需要避免时间参数化过滤：落箱专用 TOTG 的 `min_angle_change` 使用
`1e-9 rad`，保留几十微米对应的关节变化；默认 `0.001 rad` 会把这类轨迹缩为单点。
局部数值逆解核对 TCP 残差不超过 0.1 µm，专项测试覆盖 25 µm 下移、向上卸力、
2 mm 自由下降及微小关节轨迹保留。程序拒绝没有实际运动段的落箱轨迹。
导纳位置参考在世界坐标中累积，避免受载跟踪误差抵消每次小步；累计指令压入量
限制在预计底板位置以下最多 0.5 mm，腕力估计过期则保持闭爪停止。
