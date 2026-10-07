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

主 MoveIt 启动用 `left_arm_use_fake_hardware`、`right_arm_use_fake_hardware`
和 `torso_head_use_fake_hardware` 分别选择 mock 或真机；其他独立启动文件的
`use_fake_hardware` 参数仍按各自文档使用。

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
  left_arm_use_fake_hardware:=false right_arm_use_fake_hardware:=false \
  torso_head_use_fake_hardware:=false use_rviz:=true
```

SSH/headless 环境：

```bash
ros2 launch zhishu_robot_bringup zhishu_robot.moveit.launch.py \
  left_arm_use_fake_hardware:=false right_arm_use_fake_hardware:=false \
  torso_head_use_fake_hardware:=false use_rviz:=false
```

在连接电机前，可先用 mock hardware 验证 launch：

```bash
ros2 launch zhishu_robot_bringup zhishu_robot.moveit.launch.py \
  use_rviz:=false
```

左右臂与头腰部可以分别选择 ros2_control mock 或 EtherCAT 真机。
三个 `*_use_fake_hardware` 参数在主 MoveIt 启动中默认均为 `true`（mock）。
`enable_left_arm_hardware` / `enable_right_arm_hardware` 只决定是否加载该组件。
例如左臂 mock、右臂真机（头腰部不加载）：

```bash
ros2 launch zhishu_robot_bringup zhishu_robot.moveit.launch.py \
  left_arm_use_fake_hardware:=true right_arm_use_fake_hardware:=false \
  enable_torso_head_hardware:=false enable_motors_on_activate:=false
```

反过来可设 `left_arm_use_fake_hardware:=false right_arm_use_fake_hardware:=true`。
这里的 mock 只模拟关节状态和命令，不是 Gazebo 物理仿真。手眼标定启动文件
`zhishu_robot.handeye.launch.py` 也接受这三个独立参数，默认均为 `false`
（真机）；其右臂默认不加载，如需控制右臂需另设
`enable_right_arm_hardware:=true`。真机首次验证建议保持
`enable_motors_on_activate:=false`，确认硬件状态后再启用电机。

真机启动后检查：

```bash
ros2 control list_controllers
ros2 control list_hardware_components
ros2 control list_hardware_interfaces
ros2 topic hz /joint_states
```

### 7.4 日志记录

真机 MoveIt bringup 默认启动中央日志记录器。每次启动会建立独立会话目录：

```text
<guohua_ws>/zhishu_robot_logs/YYYY-MM-DD/HH-MM-SS/
  zhishu_robot_YYYYMMDD_HHMMSS_PID.log
  ethercat_kernel.log
  native/
```

中央日志汇总 `/rosout`，达到 10 MiB 后自动轮转；`native/` 保存 ROS 2 原生的
逐进程日志。日志写盘位于独立进程，不进入 EtherCAT 控制循环。

首次启用 EtherCAT 内核日志时，在机器人上执行一次权限配置：

```bash
sudo $(ros2 pkg prefix zhishu_robot_logging)/lib/zhishu_robot_logging/\
setup_kernel_ethercat_sudo.sh
```

常用启动参数：

```bash
ros2 launch zhishu_robot_bringup zhishu_robot.moveit.launch.py \
  log_root:=/home/zhul/robot_logs \
  log_max_file_size_mb:=10 \
  log_min_severity:=20
```

分别关闭中央日志或内核日志：

```bash
enable_log_recorder:=false
enable_kernel_ethercat_log:=false
```

也可设置环境变量 `ZHISHU_ROBOT_LOG_DIR` 修改默认日志根目录。详细说明见
`src/zhishu_robot_logging/README.md`。

### 7.5 Commander C++ SDK

`zhishu_robot_commander_cpp` 提供 MoveIt 2 C++ SDK 和 ROS 2 话题桥接节点，支持
左右臂、双臂以及两个夹爪。SDK 0.2 增加结构化错误、同步/异步调用、任务等待、
状态查询和软件取消，并保持原 `Commander` 布尔接口兼容。先启动真机或假硬件
bringup，再启动 Commander：

```bash
ros2 launch zhishu_robot_commander_cpp commander.launch.py \
  use_fake_hardware:=true enable_motors_on_activate:=true
```

监听执行结果：

```bash
ros2 topic echo /zhishu_robot_commander/status
```

示例：双臂回到 SRDF home 状态：

```bash
ros2 topic pub --once /zhishu_robot_commander/both_arms/named_target \
  std_msgs/msg/String "{data: both_arms_home}"
```

Commander 不会自行启动控制器，启动参数必须和已经运行的 bringup 保持一致。
完整接口和 C++ 示例见 `src/zhishu_robot_commander_cpp/README.md`。
仅编译和单元测试可运行 `bash scripts/test_commander_sdk_build.sh`，该脚本不会发送运动命令。

### 7.6 MCP C++ 读写接口

`zhishu_robot_mcp` 使用 `ament_cmake + rclcpp + cxxmcp` 提供本地 stdio MCP
Server。它可以读取 ROS 节点、话题、控制器、关节、EtherCAT 和日志状态，也可以在
显式解锁后通过 MoveIt 控制机械臂和夹爪，或调用固定的故障复位与电机使能服务。

编译：

```bash
colcon build --symlink-install --packages-up-to zhishu_robot_mcp
source install/setup.bash
```

默认只读启动：

```bash
export ZHISHU_MCP_WORKSPACE=/home/pi/Documents/zhul/guohua_ws
ros2 run zhishu_robot_mcp zhishu_robot_mcp_server
```

仿真写接口需要设置：

```bash
export ZHISHU_MCP_ENABLE_WRITES=1
export ZHISHU_MCP_ROBOT_MODE=simulation
```

真机写接口还需要额外设置 `ZHISHU_MCP_ALLOW_PHYSICAL=1`。写接口默认关闭、固定
白名单并有限流；机械臂运动异步执行，`stop_motion` 不能代替实体急停。完整工具、
确认短语和 MCP 客户端进程配置见 `src/zhishu_robot_mcp/README.md`。

### 7.7 独立 REST API

`zhishu_robot_rest_api` 是与 MCP 完全独立的 `ament_cmake + rclcpp` 包，使用
`cpp-httplib` 提供版本化 HTTP JSON 接口，并通过现有 Commander SDK/MoveIt 控制
机械臂。默认只监听 `127.0.0.1:8088` 且只读。

```bash
colcon build --symlink-install --packages-up-to zhishu_robot_rest_api
source install/setup.bash
export ZHISHU_REST_WORKSPACE=/home/pi/Documents/zhul/guohua_ws
ros2 run zhishu_robot_rest_api zhishu_robot_rest_api_server
```

运行后可以访问：

```text
GET http://127.0.0.1:8088/api/v1/health
GET http://127.0.0.1:8088/api/v1/openapi.json
GET http://127.0.0.1:8088/api/v1/robot/status
```

写接口必须配置 `ZHISHU_REST_ENABLE_WRITES=1` 和 Bearer Token；真机还需要
`ZHISHU_REST_ROBOT_MODE=physical` 与 `ZHISHU_REST_ALLOW_PHYSICAL=1`。非回环监听
需要单独设置 `ZHISHU_REST_ALLOW_REMOTE=1`，并应通过 HTTPS 反向代理或可信 VPN
访问。完整路由、环境变量和 `curl` 示例见 `src/zhishu_robot_rest_api/README.md`。

### 7.8 LES 收货流程 Gazebo 仿真

`zhishu_robot_receiving_sim` 按照 LES 收货流程图串联 LES Mock、扫码 Mock、Gazebo
差速 AGV 和现有 Commander/MoveIt。默认会故意产生一次零件箱错码，反馈 LES 后
重新匹配，再完成放箱和 AGV 返回；真实 LES、真实扫码和夹持接触物理均保留为明确
可替换边界。

```bash
colcon build --symlink-install --packages-up-to zhishu_robot_receiving_sim
source install/setup.bash
ros2 launch zhishu_robot_receiving_sim receiving_demo.launch.py
```

流程状态可通过 `/receiving_sim/state` 和 `/receiving_sim/events` 查看。完整仿真范围、
手动启动、故障注入参数及限制见 `src/zhishu_robot_receiving_sim/README.md`。完成编译后
也可以运行 `bash scripts/test_receiving_workflow_sim.sh` 做一键黑盒验收。

### 7.9 15 kg 小料箱预抽出与双臂转运验证

2026-10-07 当前方案已改为 **200 mm 夹爪、140 mm TCP、约 50 mm 浅夹取**，
第二步保留预抽出底盘站位，取消原有 12 cm 横向调位导致的转向再回正。
独立初始场景首箱完整搬运已通过；旧录像箱壁实际为 142 mm，
闭爪插入修正为约 48 mm（此前按名义箱口误报 54 mm）。现已补齐到 148 mm，详见
[`200 mm 夹爪验证及新视频`](docs/pallet_bin_200mm_validation_20261007.md)。
以下 320 mm 数据为历史方案，不是当前夹爪尺寸。

148 mm 模型经进一步优化已从独立初始场景连续完成整轮：桌边腰部采用
0.14 m / -0.20 rad，大箱内落点 x=0.98 m，箱内释放间隙 50 mm。
实际浅夹取约 48.5 mm，最大采样倾角约 0.27°，详见
[`148 mm 最终完整录像验证`](docs/pallet_bin_final148_video_20261007.md)。
原失稳失败保留在 [`148 mm 修正与失败回归记录`](docs/pallet_bin_148mm_retest_20261007.md)。

**−45° 最大弯腰候选复测**：原 200 mm 夹爪未通过；360 mm 夹爪配合
300 mm TCP、250/210 mm 携带/桌边升降台、低速缓启底盘，已从独立初始
场景连续完成首箱预抽出、双臂搬运、入箱、撤爪及回位。实际最大弯腰约
−44.98°，初始夹取深度约 50 mm；该候选由启动参数选择，不覆盖原默认
200 mm 配置。复现与失败边界见
[`−45° 长夹爪完整流程验证`](docs/pallet_bin_45deg_validation_20261007.md)。

**最新验证方案：落底后松爪**。360 mm 手指 / 300 mm TCP / −45°
配置从独立初始场景连续完成首箱搬运及回位，桌边箱心目标改为 0.94 m。
初始实测浅夹取约 46 mm，实际最大弯腰约 −44.98°。
保持闭爪以 2 mm 小步下降，真实箱底接触、高度和倾角稳定确认后才完全开爪；
开爪后沉降约 0.16 mm，不沿用上述历史方案约 50 mm 悬空释放。
完整 8 倍速 MP4 为 4 分 1.4 秒，记录实际 Gazebo 图像并自动切换视角，
验证记录见 [`落底松爪与 8 倍速完整录像`](docs/pallet_bin_floor_release_video_20261007.md)。
这是位置控制加接触确认，不是导纳力控，也不构成实机安全认证。

当前模型已补平小料箱缺口，并移除右夹爪附加横块；底盘与托盘边沿名义间距
为 0.20 m，首箱放在托盘近侧边沿。流程为夹住外围箱壁预抽出、读取物理位姿
重新定位、双臂抓取，再转运至 0.55 m 桌面上的大料箱。

2026-10-03/04 升级版候选验证中，首个外围箱 `r0_c0` 已从初始场景连续完成
约 120 mm 物理预抽出、双臂抓取、约 0.962 m 协调携带、绕行、入箱释放及回位。
释放后中心约 `(1.0310, -1.5500, 0.6360) m`；1 Hz 记录的最大箱体倾角约 0.10°，
入箱阶段约 0.04°。长指启动/停车改为升降 0.10 m、腰部 -0.30 rad，取消回位后
独立归零造成的底盘干涉。2026-10-04 三视角录像复测已包含上述端点修正，
从初始场景连续完成整轮首箱搬运，最终中心约 `(1.0325, -1.5500, 0.6360) m`；
本轮 1 Hz 采样最大倾角约 0.42°。录像和复现方式见
[`录像验证记录`](docs/pallet_bin_video_validation_20261004.md)。
当前仿真采用候选质量分布、支撑轮代理、250 N·m 腰部力矩、320 mm 长指和
摩擦参数，均需实机核对，不能作为硬件能力认证。详见
[`升级版候选验证`](docs/pallet_bin_upgraded_validation_20261003.md)。
原轻质量模型约 37° 前倾的失败保留在
[`动态复核历史`](docs/pallet_bin_dynamic_retest_20261003.md)。

无界面首箱全流程复测命令（候选参数，不是实机认证）：

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ROS_DOMAIN_ID=225 GZ_PARTITION=codex_pallet_final225 \
ros2 launch zhishu_robot_mtc_pick_place pallet_bin_15kg_transfer.launch.py \
  headless:=true use_rviz:=false execute:=true \
  validate_all_nine:=false full_transfer_box_index:=0 \
  pre_extract_enabled:=true
```

执行模式要求新鲜的 Gazebo 物理位姿，不能用指令位移代替实际预抽出结果；
抬升、底盘倾角和最终入箱位置均有物理结果检查。真实部署仍需接入视觉重定位。
本轮未验证九箱完整清理及中心箱搬运。
[`2026-09-29 历史记录`](docs/pallet_bin_15kg_transfer_validation_20260929.md)
保留供参考，其轨迹完成记录不代表当前模型通过了 15 kg 动态搬运验收。

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
| 独立日志记录器 | `ros2 launch zhishu_robot_logging log_recorder.launch.py` |
| Commander 话题桥接 | `ros2 launch zhishu_robot_commander_cpp commander.launch.py` |
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
