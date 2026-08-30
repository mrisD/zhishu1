# zhishu_robot_receiving_sim

根据收货流程图实现的 ROS 2 Jazzy + Gazebo Harmonic 仿真。目标是验证流程编排、
异常回路、AGV 到位和机械臂动作，不把尚未接入的真实 LES、视觉扫码或夹持接触
伪装成真实功能。

## 仿真范围

| 流程步骤 | 仿真方式 |
|---|---|
| LES 收货及质检 | `/les_mock/receive_and_inspect` Mock 服务，可注入质检失败 |
| LES 推送单据并叫箱 | `/les_mock/dispatch_task` Mock 服务 |
| AGV 搬运料框到工位 | Gazebo 差速 AGV、`cmd_vel` 和里程计闭环 |
| 机器人从码垛搬运零件箱 | Commander + MoveIt 执行双臂/夹爪命名姿态 |
| 零件箱条码匹配 | Scanner Mock；默认第一次错码，反馈 LES 后第二次成功 |
| 料框 SKU 扫描 | Scanner Mock |
| 机器人放箱 | MoveIt 放置姿态；箱体通过 Gazebo `set_pose` 放入 AGV 料框 |
| 机器人反馈 LES 完成 | `/les_mock/complete_task` Mock 服务 |
| 通知 AGV 入库 | Mock 通知后 Gazebo AGV 返回起点 |

当前箱体没有通过接触动力学 attach 到夹爪。机械臂在 Gazebo 中真实执行规划轨迹，
箱体在完成放置动作后使用 Gazebo 官方 `SetEntityPose` 接口移动到 AGV 料框，随后由
AGV 载回入库区。这一边界避免
把“动作流程仿真”误认为已经完成真实抓取力学验证。

## 编译与启动

```bash
cd /home/pi/Documents/zhul/guohua_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-up-to zhishu_robot_receiving_sim
source install/setup.bash

ros2 launch zhishu_robot_receiving_sim receiving_demo.launch.py
```

默认自动开始，第一次零件箱扫码故意不匹配，用于验证流程图中的重新匹配分支。

只验证流程和 Gazebo AGV，不执行 MoveIt 动作：

```bash
ros2 launch zhishu_robot_receiving_sim receiving_demo.launch.py \
  simulate_robot_motion:=false
```

手动启动：

```bash
ros2 launch zhishu_robot_receiving_sim receiving_demo.launch.py auto_start:=false
ros2 service call /receiving_sim/start std_srvs/srv/Trigger '{}'
```

## 查看流程

最新状态采用 transient-local QoS，事件流保留每一步的 JSON：

```bash
ros2 topic echo /receiving_sim/state
ros2 topic echo /receiving_sim/events
```

控制服务：

```bash
ros2 service call /receiving_sim/start std_srvs/srv/Trigger '{}'
ros2 service call /receiving_sim/stop std_srvs/srv/Trigger '{}'
ros2 service call /receiving_sim/reset std_srvs/srv/Trigger '{}'
```

## 故障注入

质检失败：

```bash
ros2 launch zhishu_robot_receiving_sim receiving_demo.launch.py quality_pass:=false
```

连续两次错码、第三次成功：

```bash
ros2 launch zhishu_robot_receiving_sim receiving_demo.launch.py \
  part_mismatch_attempts:=2
```

连续三次错码将达到默认重试上限并进入 `failed`：

```bash
ros2 launch zhishu_robot_receiving_sim receiving_demo.launch.py \
  part_mismatch_attempts:=3
```

默认 `require_robot_motion_success:=false`：如果 MoveIt 某个演示姿态规划失败，流程会
记录失败并继续虚拟业务链路。要把机械臂动作失败作为整单失败：

```bash
ros2 launch zhishu_robot_receiving_sim receiving_demo.launch.py \
  require_robot_motion_success:=true
```

## 可替换接口

后续接入真实系统时，可以保持 `receiving_orchestrator` 的流程状态机不变，逐项替换：

- 将 `les_mock` 服务替换为真实 REST/MQTT/数据库适配器；
- 将 `scanner_mock` 替换为相机或扫码枪驱动；
- 将 Gazebo AGV `cmd_vel` 替换为 WCS/AGV 调度接口；
- 将命名姿态演示替换为基于箱体位姿的 MTC 抓取与 Gazebo detachable joint。

## 测试

一键黑盒验收需要先关闭自动启动：

```bash
# 终端 1
ros2 launch zhishu_robot_receiving_sim receiving_demo.launch.py auto_start:=false

# 终端 2
source /home/pi/Documents/zhul/guohua_ws/install/setup.bash
ros2 run zhishu_robot_receiving_sim receiving_acceptance_test
```

也可以直接从工程根目录运行一键脚本。脚本会启动无界面 Gazebo、执行验收并只关闭
自己启动的 launch 进程：

```bash
cd /home/pi/Documents/zhul/guohua_ws
bash scripts/test_receiving_workflow_sim.sh
```

可选参数：`--gui` 显示 Gazebo，`--skip-robot` 跳过 MoveIt，`--no-retry` 关闭
第一次错码注入。

验收程序会自行 reset/start，并检查：

- 全部流程状态按流程图顺序到达 `receipt_complete`；
- 第一次条码不匹配、LES 反馈和重新扫码分支确实发生；
- AGV 里程计最大位移至少为 0.8 m，且最终返回起点附近；
- 七条 Commander 搬箱命令全部成功，且没有失败结果；
- Gazebo `set_pose` 已确认把零件箱放入 AGV 料框。

如果只启动业务流程和 AGV、关闭机械臂动作，可相应关闭验收项：

```bash
ros2 run zhishu_robot_receiving_sim receiving_acceptance_test --ros-args \
  -p require_commander_result:=false
```

纯状态机单元测试：

```bash
colcon test --packages-select zhishu_robot_receiving_sim
colcon test-result --verbose
```
