# 智枢双臂机器人 ROS 2 Jazzy 使用说明

本工作空间用于智枢双臂机器人的 Gazebo 仿真、MoveIt 规划、RViz 可视化、
MoveIt Task Constructor、EtherCAT 真机控制，以及 Meta Quest 3S 双手柄遥操作。

适用环境：

- Ubuntu 24.04
- ROS 2 Jazzy
- Gazebo Harmonic（`gz sim`）
- MoveIt 2
- 默认 `colcon build` 目录：`build/`、`install/`、`log/`

## 1. 安全说明

真机启动前必须确认：

- 操作员可以立即触及实体急停。
- 机器人工作范围内无人、无障碍物。
- 首次运动使用低速并只测试一个方向。
- EtherCAT 从站数量、顺序及网卡配置正确。
- 不要同时启动会争用相同关节的控制器。
- Quest 手柄按键不是实体急停，不能替代硬件安全回路。

以下命令中，`use_fake_hardware:=true` 只用于安全测试；真机使用
`use_fake_hardware:=false`。

## 2. 安装依赖

```bash
cd /home/zhul/online/guohua_ws
source /opt/ros/jazzy/setup.bash

sudo rosdep init 2>/dev/null || true
rosdep update
rosdep install --from-paths src --ignore-src -r -y --rosdistro jazzy
```

确认所有依赖满足：

```bash
rosdep check --from-paths src --ignore-src --rosdistro jazzy
```

## 3. 编译工作空间

项目使用默认 `install/`，不要指定其他安装目录：

```bash
cd /home/zhul/online/guohua_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
```

每次新开终端都需要执行：

```bash
cd /home/zhul/online/guohua_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
```

如果修改了 URDF、SRDF、YAML 或 launch 文件，重新运行：

```bash
colcon build --symlink-install
source install/setup.bash
```

## 4. Gazebo + MoveIt + RViz

这是日常仿真推荐的启动方式，一条命令同时启动 Gazebo、控制器、MoveIt 和 RViz：

```bash
ros2 launch zhishu_robot_gazebo zhishu_robot.gazebo_moveit.launch.py
```

默认参数：

- Gazebo 显示界面：开启。
- RViz：开启。
- 世界：`pick_and_place_demo.world`。
- 控制器：`joint_state_broadcaster`、`body_posture_controller`、
  `dual_arm_controller`、左右夹爪控制器。

只在 Ubuntu 桌面显示 Gazebo，不启动 MoveIt：

```bash
ros2 launch zhishu_robot_gazebo zhishu_robot.gazebo.launch.py \
  headless:=false use_camera:=false
```

无界面运行 Gazebo + MoveIt，适合 SSH、VSCode Remote 和自动测试：

```bash
ros2 launch zhishu_robot_gazebo zhishu_robot.gazebo_moveit.launch.py \
  headless:=true use_rviz:=false
```

选择其他世界：

```bash
ros2 launch zhishu_robot_gazebo zhishu_robot.gazebo_moveit.launch.py \
  world_file:=empty.sdf
```

查看可用参数：

```bash
ros2 launch zhishu_robot_gazebo zhishu_robot.gazebo_moveit.launch.py --show-args
```

## 5. 在 RViz 中控制 Gazebo 机器人

启动组合 launch 后：

1. 在 RViz 的 MotionPlanning 面板选择规划组：
   `left_arm`、`right_arm` 或 `both_arms`。
2. 设置交互目标或选择随机有效目标。
3. 点击 **Plan** 检查轨迹。
4. 确认轨迹安全后点击 **Execute** 或 **Plan & Execute**。

检查 Gazebo 控制器：

```bash
ros2 control list_controllers
```

应看到以下控制器为 `active`：

```text
joint_state_broadcaster
body_posture_controller
dual_arm_controller
left_gripper_controller
right_gripper_controller
```

检查关节状态：

```bash
ros2 topic hz /joint_states
ros2 topic echo /joint_states --once
```

`both_arms` 已经过随机规划和 Gazebo 执行回归。两侧肘关节初始位置设置为
`0.001 rad`，用于避开硬下限 `0 rad` 附近的浮点误差。

## 6. 只启动 MoveIt

不启动 Gazebo，只启动 MoveGroup 和 RViz：

```bash
ros2 launch zhishu_robot_moveit_config standalone_move_group.launch.py
```

无 RViz：

```bash
ros2 launch zhishu_robot_moveit_config standalone_move_group.launch.py \
  use_rviz:=false
```

MoveIt 官方生成的 demo（使用 mock hardware）：

```bash
ros2 launch zhishu_robot_moveit_config demo.launch.py
```

## 7. 真机 MoveIt 控制

### 7.1 EtherCAT 检查

先查看网卡：

```bash
ip -br link
cat /sys/class/net/实际网卡名/address
```

IgH EtherCAT 配置文件通常为：

```text
/usr/local/etherlab/etc/sysconfig/ethercat
```

根据实际网卡填写两个主站，例如：

```bash
MASTER0_DEVICE="左臂网卡名或MAC"
MASTER1_DEVICE="右臂网卡名或MAC"
DEVICE_MODULES="generic"
```

启动并检查 EtherCAT：

```bash
sudo /etc/init.d/ethercat start
sudo /etc/init.d/ethercat status
ls -l /dev/EtherCAT*
ethercat master
ethercat slaves
```

持续观察从站：

```bash
watch -n 0.2 ethercat slaves
```

查看内核日志：

```bash
sudo dmesg -wT | grep -Ei \
  "EtherCAT|Link state|TIMED OUT|UNMATCHED|SKIPPED|Synchronization"
```

### 7.2 实时权限

```bash
sudo groupadd -f realtime
sudo usermod -aG realtime "$USER"
```

创建 `/etc/security/limits.d/99-realtime.conf`：

```text
@realtime soft rtprio 99
@realtime hard rtprio 99
@realtime soft memlock unlimited
@realtime hard memlock unlimited
```

修改用户组后需要注销并重新登录。

### 7.3 启动真机

真机 MoveIt 使用 `dual_arm_controller`：

```bash
ros2 launch zhishu_robot_bringup zhishu_robot.moveit.launch.py \
  use_fake_hardware:=false use_rviz:=true
```

SSH/headless 环境：

```bash
ros2 launch zhishu_robot_bringup zhishu_robot.moveit.launch.py \
  use_fake_hardware:=false use_rviz:=false
```

在连接电机前，可先用 mock hardware 验证 launch：

```bash
ros2 launch zhishu_robot_bringup zhishu_robot.moveit.launch.py \
  use_fake_hardware:=true use_rviz:=false
```

真机启动后检查：

```bash
ros2 control list_controllers
ros2 control list_hardware_components
ros2 control list_hardware_interfaces
ros2 topic hz /joint_states
```

## 8. Meta Quest 3S 双臂遥操作

Quest 遥操作使用左右独立控制器：

```text
left_arm_controller
right_arm_controller
```

不要同时启动使用 `dual_arm_controller` 的真机 MoveIt bringup，否则会争用相同的
关节命令接口。

启动 Quest 真机遥操作：

```bash
ros2 launch quest_dual_arm_teleop quest_dual_arm_teleop.launch.py \
  use_fake_hardware:=false launch_rviz:=true tcp_port:=5005
```

SSH/headless：

```bash
ros2 launch quest_dual_arm_teleop quest_dual_arm_teleop.launch.py \
  use_fake_hardware:=false launch_rviz:=false tcp_port:=5005
```

先用 mock hardware 测试：

```bash
ros2 launch quest_dual_arm_teleop quest_dual_arm_teleop.launch.py \
  use_fake_hardware:=true launch_rviz:=false tcp_port:=5005
```

USB 连接 Quest：

```bash
adb devices
adb reverse tcp:5005 tcp:5005
adb reverse --list
```

Unity 端使用：

```text
Host: 127.0.0.1
Port: 5005
```

检查连接和数据：

```bash
ros2 topic echo /quest/connected
ros2 topic hz /quest/left_controller/pose
ros2 topic hz /quest/right_controller/pose
ros2 topic echo /left_servo/status
ros2 topic echo /right_servo/status
```

移动到适合 Servo 的初始姿态：

```bash
ros2 launch quest_dual_arm_teleop move_to_servo_ready.launch.py
```

如果 Servo 未启动：

```bash
ros2 service call /left_servo/start_servo std_srvs/srv/Trigger "{}"
ros2 service call /right_servo/start_servo std_srvs/srv/Trigger "{}"
```

低速单臂测试：

```bash
ros2 run quest_dual_arm_teleop servo_smoke_test.py \
  --arm left --axis linear_z --speed 0.01 --duration 1.0
```

当前 Quest 配置已提高位移响应，并修正竖直方向：

```yaml
translation_scale: 0.70
rotation_scale: 0.75
max_linear_speed: 0.15
max_angular_speed: 0.60
invert_vertical_command: true
```

参数文件：

```text
src/quest_dual_arm_teleop/config/quest_teleop.yaml
```

## 9. MoveIt Task Constructor

MTC 默认只规划、不执行，首次运行建议使用 mock hardware：

```bash
ros2 launch zhishu_robot_mtc_pick_place mtc_demos.launch.py \
  use_fake_hardware:=true use_rviz:=true
```

双臂搬运箱体任务：

```bash
ros2 launch zhishu_robot_mtc_pick_place dual_arm_crate_carry.launch.py \
  use_fake_hardware:=true use_rviz:=true execute:=false
```

确认规划结果安全后，才允许在正确环境中使用：

```bash
execute:=true
```

查看箱体位置、尺寸及规划参数：

```bash
ros2 launch zhishu_robot_mtc_pick_place \
  dual_arm_crate_carry.launch.py --show-args
```

## 10. RViz 和 Gazebo 显示问题

出现以下错误表示当前终端没有图形显示：

```text
qt.qpa.xcb: could not connect to display
```

解决方法任选其一：

1. 在 Ubuntu 本机桌面终端启动带 UI 的 launch。
2. 在 SSH/VSCode Remote 中使用无界面参数：

   ```bash
   headless:=true use_rviz:=false
   ```

3. Mac 安装并启动 XQuartz，然后使用可信 X11 转发：

   ```bash
   ssh -Y t490_ubuntu24.04
   echo "$DISPLAY"
   rviz2
   ```

X11 转发 Gazebo 3D 界面可能较慢。远程调试时更推荐 Gazebo headless，在 Ubuntu
本机显示 RViz/Gazebo，或配置专用远程桌面。

不要通过强制设置 `QT_QPA_PLATFORM=offscreen` 来交互使用 RViz；offscreen 只适用于
自动化启动测试，不会产生可操作窗口。

## 11. 自动测试

### 11.1 静态 Jazzy 检查

```bash
python3 scripts/validate_jazzy.py
```

### 11.2 全部 launch 回归

该脚本覆盖工作空间中的全部 18 个 launch：

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
bash scripts/test_all_launch_files.sh /home/zhul/online/guohua_ws
```

日志位置：

```text
log/launch_smoke/
```

### 11.3 MoveIt 随机规划

先启动 Gazebo + MoveIt headless，再执行：

```bash
python3 scripts/test_moveit_planning_services.py \
  --samples 10 --planning-time 8
```

### 11.4 Gazebo 双臂规划与执行

脚本会自动启动 Gazebo + MoveIt，规划并执行一个碰撞检测通过的邻近双臂目标，
随后对左右臂和双臂进行随机规划压力测试：

```bash
bash scripts/test_gazebo_moveit_execution.sh \
  /home/zhul/online/guohua_ws
```

结果与日志：

```text
log/gazebo_moveit_execution.result
log/gazebo_moveit_planning_regression.log
log/gazebo_moveit_execute_regression.log
```

### 11.5 包级测试

```bash
colcon test --event-handlers console_cohesion+
colcon test-result --verbose
```

## 12. Launch 文件索引

| 功能 | 命令 |
|---|---|
| Quest 双臂遥操作 | `ros2 launch quest_dual_arm_teleop quest_dual_arm_teleop.launch.py` |
| 移动到 Servo 初始姿态 | `ros2 launch quest_dual_arm_teleop move_to_servo_ready.launch.py` |
| 真机 MoveIt bringup | `ros2 launch zhishu_robot_bringup zhishu_robot.moveit.launch.py` |
| URDF/RViz 显示 | `ros2 launch zhishu_robot_description display_dh.launch.py` |
| Gazebo | `ros2 launch zhishu_robot_gazebo zhishu_robot.gazebo.launch.py` |
| Gazebo + MoveIt + RViz | `ros2 launch zhishu_robot_gazebo zhishu_robot.gazebo_moveit.launch.py` |
| MoveIt demo | `ros2 launch zhishu_robot_moveit_config demo.launch.py` |
| MoveGroup | `ros2 launch zhishu_robot_moveit_config move_group.launch.py` |
| 独立 MoveGroup | `ros2 launch zhishu_robot_moveit_config standalone_move_group.launch.py` |
| MoveIt RViz | `ros2 launch zhishu_robot_moveit_config moveit_rviz.launch.py` |
| Robot State Publisher | `ros2 launch zhishu_robot_moveit_config rsp.launch.py` |
| Setup Assistant | `ros2 launch zhishu_robot_moveit_config setup_assistant.launch.py` |
| 生成型控制器加载 | `ros2 launch zhishu_robot_moveit_config spawn_controllers.launch.py` |
| 当前控制器加载组件 | `ros2 launch zhishu_robot_moveit_config load_ros2_controllers.launch.py` |
| 静态虚拟关节 TF | `ros2 launch zhishu_robot_moveit_config static_virtual_joint_tfs.launch.py` |
| SQLite Warehouse | `ros2 launch zhishu_robot_moveit_config warehouse_db.launch.py` |
| MTC 抓取演示 | `ros2 launch zhishu_robot_mtc_pick_place mtc_demos.launch.py` |
| MTC 双臂箱体搬运 | `ros2 launch zhishu_robot_mtc_pick_place dual_arm_crate_carry.launch.py` |

`spawn_controllers.launch.py` 和 `load_ros2_controllers.launch.py` 是组件型 launch，
需要已有 `/controller_manager`；日常使用优先启动完整 bringup。

## 13. 常见问题

### `both_arms` 规划失败

```bash
ros2 topic echo /joint_states --once
```

检查左右肘关节是否因为数值误差小于 `0 rad`。当前配置初始值为 `0.001 rad`。
如果修改后仍读取旧值，说明终端可能使用了旧 overlay：

```bash
colcon build --symlink-install
source install/setup.bash
ros2 pkg prefix zhishu_robot_moveit_config
```

### Plan 失败但 Plan & Execute 有动作

先检查 RViz 当前规划组和起始状态是否已经更新，再查看 MoveGroup 日志中的：

```text
CheckStartStateBounds
CheckStartStateCollision
```

不要在 Gazebo 尚未发布稳定 `/joint_states` 时立即规划。

### 控制器不完整

```bash
ros2 control list_controllers
ros2 control list_hardware_interfaces
```

Gazebo/MoveIt 应使用 `dual_arm_controller`；Quest Servo 应使用
`left_arm_controller` 和 `right_arm_controller`。两种模式不能同时占用相同关节。

### Gazebo 有机器人但姿态异常

确认使用的是当前默认 `install/`：

```bash
ros2 pkg prefix zhishu_robot_description
ros2 pkg prefix zhishu_robot_moveit_config
ros2 pkg prefix zhishu_robot_gazebo
```

三个路径都应指向：

```text
/home/zhul/online/guohua_ws/install/
```

### 查看某个 launch 的参数

```bash
ros2 launch 包名 launch文件名 --show-args
```

## 14. 停止系统

正常停止使用 `Ctrl+C`。重新启动前确认没有残留进程：

```bash
pgrep -af "ros2 launch|gz sim|move_group|ros2_control_node|rviz2"
```

真机停止后再次确认 EtherCAT 驱动器和机器人处于安全状态。
