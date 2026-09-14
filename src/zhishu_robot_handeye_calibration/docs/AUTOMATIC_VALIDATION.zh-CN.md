# 手眼标定仿真自动验证说明

本文说明智枢机器人头部相机 Eye-to-Hand 标定的无人值守仿真、结果判定和
2026-09-14 实测结果。标定关系为：

```text
base_link -> head_camera_optical_frame
```

## 1. 自动流程

`handeye_auto.launch.py` 会自动完成以下工作：

1. 启动 Gazebo Harmonic、ROS 2 控制器和 MoveIt。
2. 将头部关节保持为 `yaw=0.5 rad`、`pitch=0.15 rad`；自动节点在开始及每次
   采样时检查 `0.002 rad` 容差，越界立即失败。
3. 依次规划并执行 12 组左臂关节姿态。
4. 每次运动稳定后，从 `/camera_head/color/image_raw` 检测
   `DICT_5X5_250`、3×4 ArUco GridBoard，并使用相机内参估计
   `camera -> target` 位姿。
5. 按图像时间戳读取 TF 中的 `base -> left_gripper_tcp` 位姿。
6. 分别运行 Tsai、Park、Horaud、Andreff、Daniilidis 五种 OpenCV
   Eye-to-Hand 求解器，并按 AX=XB 重投影误差选择结果。
7. 将结果与独立 TF 真值 `base_link -> head_camera_optical_truth` 比较。该帧与
   `head_camera_optical_frame` 重合，但只由 `robot_state_publisher` 发布，不会被
   RViz HandEye 面板的相机位姿猜测覆盖。
8. 写入完整结果 YAML、MoveIt 原生 samples YAML 和带检测坐标轴的逐帧图像。
9. RViz 已运行时，通过 `/handeye_calibration/auto_load_samples` 通知插件；插件在
   Qt 主线程中填充自己的 `Pose samples` 树，选中本轮最佳 solver，自动 Solve，
   并显示平移、四元数与重投影误差。

这里明确区分两类误差：

- `reprojection_*`：样本之间的 AX=XB 内部一致性误差；
- `truth_*`：最终标定外参与 Gazebo/robot_state_publisher 真值的绝对误差。

只有内部一致性和真值误差都通过门槛，最终状态才会是 `PASS`。

## 2. 编译与运行

```bash
cd /home/zhul/online/guohua_ws
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install --packages-up-to zhishu_robot_handeye_calibration
source install/setup.bash

ros2 launch zhishu_robot_handeye_calibration handeye_auto.launch.py \
  output_file:=/home/zhul/online/guohua_ws/handeye_calibration_result.yaml \
  image_directory:=/home/zhul/online/guohua_ws/handeye_calibration_samples
```

若运行环境限制 ROS 日志目录，可先指定一个可写目录：

```bash
export ROS_LOG_DIR=/tmp/zhishu_ros_log
```

自动流程不启动 Gazebo GUI 和 RViz，也不依赖 Xvfb。ROS 2 DDS 仍需要允许
本机网络套接字；受限容器中应为进程提供对应权限。

需要在桌面上观察采样时，先启动可视化栈：

```bash
ros2 launch zhishu_robot_handeye_calibration handeye_gazebo.launch.py \
  headless:=false use_rviz:=true
```

再在第二个终端运行自动节点：

```bash
source /opt/ros/jazzy/setup.bash
source /home/zhul/online/guohua_ws/install/setup.bash
ros2 run zhishu_robot_handeye_calibration handeye_auto_calibration --ros-args \
  -p use_sim_time:=true \
  -p output_file:=/home/zhul/online/guohua_ws/handeye_calibration_result.yaml \
  -p image_directory:=/home/zhul/online/guohua_ws/handeye_calibration_samples \
  -p moveit_samples_file:=/home/zhul/online/guohua_ws/handeye_moveit_samples.yaml
```

RViz 中的 `HandEye Sample Poses` 会累计显示每个有效样本的 RGB 坐标轴、
`S1` 至 `S12` 标签、可见 marker 数和总进度；`HandEye Detection Image`
显示当前接受帧的检测结果。每接受一个姿态，HandEyeCalibration 都会自动切到
`Calibrate` 页并刷新原生 `Pose samples` 树；完成时显示 `Sample 1` 至
`Sample 12`，每项可
展开查看 `TF base-to-eef` 与 `TF camera-to-target`；下方显示
`base_link -> head_camera_optical_frame` 的平移、四元数和 Reprojection error。

可视化 launch 中的常驻 relay 会把 Gazebo 的 `/camera_head/camera_info` 转发为
MoveIt 插件按 image_transport 约定查找的 `/camera_head/color/camera_info`，因此
HandEyeCalibration 的 Target 页能持续收到相机内参和图像。

`handeye_moveit_samples.yaml` 是插件原生格式，顶层为样本序列，每个样本含
`effector_wrt_world` 和 `object_wrt_sensor` 两个按行展开的 4×4 矩阵。它与包含
PASS/FAIL、阈值、仿真真值和 solver 比较的完整结果文件用途不同。

桌面模式下 Gazebo server 与 GUI 分进程运行，GUI 使用兼容性的 OGRE renderer，
避免 Mesa/EGL 环境中 OGRE2 强制软件渲染导致的启动后闪退。

## 3. 成功判据

默认判据如下：

| 指标 | 默认要求 |
|---|---:|
| 有效样本数 | ≥ 5 |
| 每个有效样本可见 marker | ≥ 3 |
| 平移真值误差 | ≤ 0.03 m |
| 旋转真值误差 | ≤ 0.05 rad |
| AX=XB 平移 RMSE | ≤ 0.02 m |
| AX=XB 旋转 RMSE | ≤ 0.03 rad |
| 求解器输出 | 所有矩阵元素为有限值 |

标定板在部分大姿态下会被手臂遮挡。默认 `minimum_markers:=3` 保留完整的
12 个姿态；若设置为 6，本场景实测会保留 8 个姿态。两种设置都通过了
真值验证。工程使用时可按可见性和姿态多样性的权衡调整该参数。

## 4. 本次自动验证结果

运行日期：2026-09-14；环境为 Ubuntu 24.04、ROS 2 Jazzy、Gazebo
Harmonic、OpenCV 4.6.0。

结果：`PASS`

| 项目 | 数值 |
|---|---:|
| 有效样本 | 12 / 12 |
| marker 数量 | 6, 6, 9, 9, 6, 6, 3, 6, 3, 5, 3, 7 |
| 选中求解器 | Horaud1995 |
| AX=XB 平移 RMSE | 0.00750606 m |
| AX=XB 旋转 RMSE | 0.00993169 rad |
| 真值平移误差 | 0.00799582 m |
| 真值旋转误差 | 0.00594544 rad（约 0.3407°） |

标定结果：

```yaml
parent_frame: base_link
child_frame: head_camera_optical_frame
translation_xyz_m: [0.0850798518, 0.0251614026, 1.3738529878]
quaternion_xyzw: [-0.5998740692, 0.4026319156, -0.3221149919, 0.6117847445]
transform_matrix:
  - [ 0.4682589451, -0.0889268151,  0.8791049891, 0.0850798518]
  - [-0.8771869673,  0.0727860662,  0.4746000559, 0.0251614026]
  - [-0.1061912654, -0.9933751608, -0.0439227167, 1.3738529878]
  - [ 0.0,           0.0,           0.0,          1.0]
```

本次按上面的示例命令覆盖了默认输出路径，因此完整结果位于工作空间根目录的
`handeye_calibration_result.yaml`，其中还包含五种求解器的比较结果、仿真真值
和每个样本的两组输入 TF。插件原生样本位于 `handeye_moveit_samples.yaml`，
检测图像位于 `handeye_calibration_samples/`。

## 5. 参数与结果文件

主要 launch 参数：

| 参数 | 默认值 | 说明 |
|---|---:|---|
| `output_file` | `/tmp/handeye_calibration_result.yaml` | YAML 输出路径 |
| `image_directory` | `/tmp/handeye_calibration_samples` | 检测图像目录 |
| `moveit_samples_file` | `/tmp/handeye_moveit_samples.yaml` | MoveIt 原生样本文件；用于 RViz 自动导入 |
| `truth_sensor_frame` | `head_camera_optical_truth` | 仿真专用真值帧；不要改为 RViz 会发布猜测的 sensor frame |
| `minimum_markers` | `3` | 接受样本所需最少 marker |
| `sample_attempts_per_pose` | `3` | 每个姿态的图像/检测重试次数 |
| `translation_tolerance` | `0.03` | 平移真值误差门槛，单位 m |
| `rotation_tolerance` | `0.05` | 旋转真值误差门槛，单位 rad |
| `reprojection_translation_tolerance` | `0.02` | AX=XB 平移 RMSE 门槛，单位 m |
| `reprojection_rotation_tolerance` | `0.03` | AX=XB 旋转 RMSE 门槛，单位 rad |

自动标定节点退出码：

- `0`：得到结果并通过内部一致性与真值门槛；
- `2`：得到的样本或结果未通过验证；
- `1`：启动、相机、MoveIt 或 TF 等运行时错误。

异常路径会尽量写入 `status: FAIL` 和 `reason`。由于 ROS 2 launch 不保证把
子进程退出码原样传给外层命令，CI 或脚本应以 YAML 的 `status` 为最终判据。

## 6. 常见问题

- 相机超时：确认 `/camera_head/color/image_raw` 和
  `/camera_head/camera_info` 存在，Gazebo Sensors 系统已正常加载。
- 有效样本不足：适当降低 `minimum_markers`，或修改采样姿态以减少遮挡。
- MoveIt 执行失败：检查 `dual_arm_controller` 是否为 `active`，以及
  `/joint_states` 是否持续发布。
- 头部锁定告警：不要继续使用该轮样本，确认初始位置文件和 guard 参数均为
  `0.5/0.15 rad`。
- 真值矩阵变成单位阵：不要用 `head_camera_optical_frame` 作为仿真真值。MoveIt
  HandEye Context 会在求解前向该帧发布单位初始猜测；使用模型中的
  `head_camera_optical_truth`。
- launch 结束时 MoveGroup 在当前 ROS 2 Jazzy 版本可能打印 SIGINT 退出栈；
  标定进程已先写完结果且以 0 退出，结果有效性应以 YAML 的 `status` 和误差
  字段为准。
