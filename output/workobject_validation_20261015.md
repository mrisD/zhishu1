# 工件坐标系标定 Gazebo 验证记录

日期：2026-10-15。执行位置：远程电脑 192.168.110.59，工作空间 `/home/dky666/zhul/codex_guohua_ws`。ROS 2 Jazzy + Gazebo Harmonic，专用 ROS_DOMAIN_ID=68、GZ_PARTITION=workobject_calibration_sim。原有 domain168 程序未停止、未下发运动。

## 实现与边界

采用 X1/X2/Y1 三点法。默认原点为 Y1 在 X1—X2 直线上的投影，可切换以 X1 为原点。用户提供的 B 站 p=28 页面在开发环境未能播放；方法参照 [ABB 官方 RAPID 手册 DefFrame](https://library.e.abb.com/public/688894b98123f87bc1257cc50044e809/Technical%20reference%20manual_RAPID_3HAC16581-1_revJ_en.pdf)，不声称逐帧复现视频界面。

已标定的 TCP 来自 `right_probe_multi_point_1792042435.yaml`。自动采样用 Gazebo 几何真值模拟操作员对准动作，求解器只使用实测法兰位姿与已标定 TCP 换算出的三个测量点。真值另用于独立验算。对象坐标系相对用户坐标系为单位变换。

工件台有三根金色尖针和 X1/X2/Y1 几何字样；右臂有蓝色探针杆与金色尖端。红/绿/蓝轴为解算的工件坐标系；青色线框为工件面上方 55 mm 的 60×50 mm 验证矩形。桌板、立柱、底座、针杆加入 Gazebo 和 MoveIt 碰撞场景。尖针接触目标为几何对准，不进行力接触仿真。

## 两轮验收

工件台在基坐标中的位置均为 `[0.55, -0.225, 0.53]` m，roll/pitch 均为 `[-0.04, 0.06]` rad。标定基线约 120 mm，Y 点离 X 轴约 60 mm。

| 验证项 | 默认投影原点、无噪声 | 工件旋转、X1 原点、0.1 mm 噪声 |
|---|---:|---:|
| 工件 yaw | 0.28 rad | 0.50 rad |
| 原点误差 | 0.009161 mm | 0.131360 mm |
| 坐标系转角误差 | 0.005768° | 0.118916° |
| 三个独立目标最大位置误差 | 0.260178 mm | 0.493059 mm |
| 矩形四边观测最大偏差 | 0.321493 mm | 0.312738 mm |
| 返回初始姿态 | 是 | 是 |
| 验收 | 通过 | 通过 |

阈值：原点、独立目标和矩形观测偏差均 <1 mm，坐标系转角误差 <1°。矩形执行过程每次回调观测真实 TCP 到独立真值线段的距离，超过 2 mm 取消轨迹；以上为离散观测最大值，并非连续时间误差上界。

第二轮非采样左臂的最大关节漂移为 `0.000031712 rad`，低于 `0.02 rad` 停止阈值。实际读取 `base_link → workobject_calibrated` TF 与报告一致，位置约 `[0.497470, -0.253820, 0.533669]` m，四元数 xyzw 约 `[-0.026739, 0.025049, 0.247391, 0.968223]`。

## 结果文件

- 默认原点：`workobject_verified_20261015/20261015_141539/`，含 `calibration.yaml`、`samples.csv`、`SUCCESS`、`gazebo_final.png`。
- 旋转带噪声：`workobject_rotated_noise_20261015/20261015_142003/`，含相同结果文件。
- 第一轮结束后停止旧 TF 发布节点时，旧版退出处理产生了 ROS context 诊断；原内容保留在 `post_validation_shutdown.txt`，远程另保留 `original_shutdown_diagnostic.txt`。该诊断发生于验收通过之后，退出处理已修复，并在不连接仿真的独立域进行 SIGINT 检查，未再生成 FAILURE 标记。

## 运行和测试

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch zhishu_robot_workobject_calibration workobject_gazebo.launch.py
```

第二轮在第一轮完成、停止旧采样节点但保留 Gazebo 后运行：

```bash
ros2 launch zhishu_robot_workobject_calibration workobject_gazebo.launch.py \
  sampler_only:=true board_yaw:=0.50 noise_sigma_mm:=0.10 origin_mode:=first_point
```

远程 `colcon build` 通过；`colcon test --packages-select zhishu_robot_workobject_calibration` 的 7 项数学测试和 3 项探针选择兼容性测试全部通过；相关文件 `git diff --check` 通过。

开发排查中，双探针工件场景曾触发非采样左腕跟踪保护。保留完整双臂保持轨迹和漂移监控，并将本工件仿真设为只挂载右臂探针后，两轮完整验证均通过，未放宽关节、跟踪或碰撞保护。原 TCP 场景的默认双探针配置保持不变。

Gazebo 及最终工件 TF 保留运行，机械臂已返回初始姿态。此次未执行 Git commit 或 push，未修改实机控制器的工具/工件数据。
