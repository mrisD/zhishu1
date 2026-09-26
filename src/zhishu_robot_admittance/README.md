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
