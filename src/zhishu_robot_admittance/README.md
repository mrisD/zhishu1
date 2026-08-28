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

初始姿态只用于把两只手放到箱体两侧，不等于已经可靠抓住箱体。当前工程的夹爪
由关节轨迹控制器驱动，确认两侧夹爪与蓝色抓取区域对准后，将两个主动指关节
从 `0.0 m` 闭合到 `0.04 m`；该位置与仿真箱体直径 `0.06 m` 的把手匹配，
可避免指爪穿透把手产生过大的接触力：

```bash
ros2 action send_goal /left_gripper_controller/follow_joint_trajectory \
  control_msgs/action/FollowJointTrajectory \
  "{trajectory: {joint_names: [left_gripper_left_finger_joint], points: [{positions: [0.04], time_from_start: {sec: 2}}]}}"

ros2 action send_goal /right_gripper_controller/follow_joint_trajectory \
  control_msgs/action/FollowJointTrajectory \
  "{trajectory: {joint_names: [gripper_left_finger_joint], points: [{positions: [0.04], time_from_start: {sec: 2}}]}}"
```

确认箱体已夹稳后必须重新去皮，再启用导纳：

```bash
ros2 service call /dual_arm_admittance/tare std_srvs/srv/Trigger "{}"
ros2 service call /dual_arm_admittance/enable std_srvs/srv/SetBool "{data: true}"
```

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
