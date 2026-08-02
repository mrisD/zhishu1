# Quest 3S 双手柄控制智枢机器人双臂

## 数据链路

```text
Quest 左右手柄 OpenXR 位姿/侧握键
        -> Unity TCP（二进制，72 Hz）
        -> quest_dual_arm_teleop_node
        -> /left_servo/delta_twist_cmds
        -> /right_servo/delta_twist_cmds
        -> 两个 MoveIt Servo 节点
        -> left/right_arm_controller
        -> ros2_control + EtherCAT CSP
```

本实现采用“速度映射”，不是将手柄绝对坐标直接设为机械臂目标坐标，因此按下使能键时不会跳到 Quest 世界原点。每个手柄的侧握键是对应机械臂的 deadman：按住才运动，松开立即发送零速度。

## 1. 安装依赖

```bash
sudo apt update
sudo apt install -y \
  ros-jazzy-moveit-servo \
  ros-jazzy-controller-manager \
  ros-jazzy-joint-trajectory-controller \
  ros-jazzy-eigen3-cmake-module
```

## 2. 放入工作空间并编译

把本包复制到原工作空间 `src/`：

```bash
cp -r quest_dual_arm_teleop ~/your_ws/src/
cd ~/your_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install --packages-select quest_dual_arm_teleop
source install/setup.bash
```

## 3. 真机启动

不要同时启动原来的 `zhishu_robot.moveit.launch.py`，因为原启动文件默认激活 `dual_arm_controller`，会和左右独立控制器争用相同关节。

```bash
source /opt/ros/jazzy/setup.bash
source ~/your_ws/install/setup.bash
ros2 launch quest_dual_arm_teleop quest_dual_arm_teleop.launch.py
```

确认：

```bash
ros2 control list_controllers
```

必须看到：

```text
joint_state_broadcaster  active
left_arm_controller      active
right_arm_controller     active
```

且不应激活 `dual_arm_controller`。

检查 Servo：

```bash
ros2 topic echo /left_servo/status
ros2 topic echo /right_servo/status
```

状态 `0` 表示正常。

## 4. 不连接 Quest，先做低速 Servo 冒烟测试

确保机器人周围无人、实体急停可用，先只测试一个方向：

```bash
ros2 run quest_dual_arm_teleop servo_smoke_test.py \
  --arm left --axis linear_z --speed 0.01 --duration 1.0
```

再测试右臂：

```bash
ros2 run quest_dual_arm_teleop servo_smoke_test.py \
  --arm right --axis linear_z --speed 0.01 --duration 1.0
```

如果方向与机器人坐标不一致，先检查 `base_link` 在 RViz 中的 X/Y/Z 方向，不要直接提高速度。

## 5. Quest USB 通信

```bash
adb devices
adb reverse tcp:5005 tcp:5005
adb reverse --list
```

Quest Unity 脚本的 Host 使用 `127.0.0.1`，Port 使用 `5005`。

启动 Quest 应用后检查：

```bash
ros2 topic echo /quest/connected
ros2 topic hz /quest/left_controller/pose
ros2 topic hz /quest/right_controller/pose
```

`/quest/connected` 应为 `true`，手柄位姿通常约 72 Hz。

## 6. 操作方式

- 按住左手柄侧握键：左手柄移动速度映射为左机械臂末端速度。
- 按住右手柄侧握键：右手柄移动速度映射为右机械臂末端速度。
- 松开某侧侧握键：该侧机械臂立即停止。
- 两侧可同时按住，实现双臂同步遥操作。
- Quest 断连、手柄跟踪失效或 0.15 秒未收到数据：两臂发送零速度并复位。

坐标映射：

```text
Quest +Z 前  -> ROS +X 前
Quest +X 右  -> ROS -Y 右
Quest +Y 上  -> ROS +Z（手柄姿态话题）
```

当前真机安装下，MoveIt Servo 的竖直指令方向与上述 Quest 显示坐标相反，
因此 `invert_vertical_command: true` 仅反转机器人命令的 Z 线速度。若机器人安装方向改变，
将该参数设为 `false` 即可，无需改动 URDF。

## 7. 首次真机参数

`config/quest_teleop.yaml` 已使用保守值：

```yaml
invert_vertical_command: true
translation_scale: 0.70
rotation_scale: 0.75
max_linear_speed: 0.15
max_angular_speed: 0.60
grip_threshold: 0.65
input_timeout: 0.15
filter_alpha: 0.35
```

调参顺序：

1. 先保持最大线速度 `0.03~0.06 m/s`。
2. 确认 XYZ 方向后再调整 `translation_scale`。
3. 最后启用并调整旋转；若旋转太灵敏，先将 `rotation_scale` 改为 `0.2`。
4. 不要通过增大 `filter_alpha` 解决延迟；先检查 TCP 频率和 EtherCAT 周期。

## 8. 关键诊断命令

```bash
ros2 topic hz /joint_states
ros2 topic hz /left_servo/delta_twist_cmds
ros2 topic hz /right_servo/delta_twist_cmds
ros2 topic echo /quest/left_controller/deadman
ros2 topic echo /quest/right_controller/deadman
ros2 control list_hardware_interfaces
```

若手柄有数据但机械臂不动：

```bash
ros2 service call /left_servo/start_servo std_srvs/srv/Trigger {}
ros2 service call /right_servo/start_servo std_srvs/srv/Trigger {}
```

若 Servo 状态提示碰撞或奇异点，先将机械臂用低速规划移动到远离奇异点和自碰撞的安全初始姿态，再启动遥操作。

## 9. 安全要求

- Quest 按键不是实体急停，操作员必须能直接触及机器人急停。
- 首次运行应悬空、低速、单臂测试，并限制机器人工作区内人员进入。
- 不要同时运行 MTC 执行节点、MoveGroup 轨迹执行和 Quest Servo 遥操作。
- 两个 Servo 节点会进行碰撞、奇异点和关节限位检查，但这些软件保护不能替代硬件限位与急停。

## Fix for controller parameters on ROS 2 Humble

If `left_arm_controller` or `right_arm_controller` reports that `joints` or
`command_interfaces` is empty, the launch file must pass the controller YAML
explicitly to each `controller_manager/spawner` with `--param-file`.
This fixed package already does that. It also loads `kinematics.yaml` into RViz.

## v3 controller-loading fix

This version deliberately does not pass the full MoveIt `ros2_controllers.yaml` to the arm spawners.
Each arm has a dedicated parameter file using the ROS 2 wildcard node selector `/**`, and each spawner explicitly passes the controller plugin type with `-t`.
This avoids a Humble failure mode where the spawner records the `params_file` path but the embedded controller instance still sees empty read-only `joints` and `command_interfaces` parameters.

After launch, verify:

```bash
ros2 control list_controllers
ros2 param get /left_arm_controller joints
ros2 param get /left_arm_controller command_interfaces
ros2 param get /right_arm_controller joints
ros2 param get /right_arm_controller command_interfaces
```

Expected: both controllers are `active`, each `joints` list contains seven joints, and `command_interfaces` is `[position]`.

## v4 Servo Cartesian fix

This version intentionally keeps `kinematics.yaml` on `move_group` and RViz, but does **not** pass
`robot_description_kinematics` to `left_servo` or `right_servo`. The goal is:

```text
move_group / MTC / RViz -> KDL IK
MoveIt Servo Twist      -> Jacobian pseudo-inverse
```

It also changes the Quest deadman stop behavior: an inactive/disconnected Quest publishes a zero
Twist only when transitioning from active motion to stopped. It no longer floods the Servo input
topic with zero commands while idle, so the standalone smoke tests can run while the teleop node
exists.

Direct JointJog diagnostic:

```bash
ros2 run quest_dual_arm_teleop servo_joint_smoke_test.py \
  --arm left --joint elbow_pitch --speed 0.05 --duration 1.0
```

Cartesian diagnostic:

```bash
ros2 run quest_dual_arm_teleop servo_smoke_test.py \
  --arm left --axis linear_x --speed 0.01 --duration 2.0
```


## v4 recommended real-robot test sequence

The all-zero arm posture is a poor Cartesian Servo starting posture. This package adds
`left_arm_servo_ready`, `right_arm_servo_ready`, and `both_arms_servo_ready` to the SRDF.
With the current DH model, the left/right Jacobian condition numbers at the selected Servo-ready
pose are about 12 (the all-zero posture is about 113).

After launching the stack, move to the named Servo-ready pose using MoveIt planning:

```bash
ros2 launch quest_dual_arm_teleop move_to_servo_ready.launch.py
```

Then verify Servo is started:

```bash
ros2 service call /left_servo/start_servo std_srvs/srv/Trigger "{}"
ros2 service call /right_servo/start_servo std_srvs/srv/Trigger "{}"
```

Finally run the Cartesian smoke test:

```bash
ros2 run quest_dual_arm_teleop servo_smoke_test.py \
  --arm left --axis linear_x --speed 0.01 --duration 2.0
```

For isolating Servo/controller/hardware from Cartesian kinematics, use JointJog:

```bash
ros2 run quest_dual_arm_teleop servo_joint_smoke_test.py \
  --arm left --joint elbow_pitch --speed 0.05 --duration 1.0
```
