# 顶置 RGB-D 相机动态饮料瓶抓取

## 1. 当前能力

本包在 Gazebo 中完成以下闭环：顶置 RGB-D 相机采集彩色图、深度图和点云；YOLO 分割饮料瓶；由分割掩膜与深度估计瓶体三维中心、尺寸和抓取高度；跟踪瓶体速度并预测位置；MoveIt 规划机械臂到拦截位姿；瓶体进入夹爪接触窗口后停止输送、闭合夹爪、附着瓶体并抬升。

仿真使用的相机话题与 Orbbec Gemini Pro 实机接口保持一致：

- `/camera_head/color/image_raw`
- `/camera_head/color/camera_info`
- `/camera_head/depth/image_raw`
- `/camera_head/depth_registered/points`

相机内参和 `camera_head_*_optical_frame` 到 `base_link` 的 TF 共同完成像素、深度到机器人坐标系的转换。瓶子尺寸不写死，而是通过掩膜内点云的稳健分位数在线估计。

## 2. 方案与节点

| 节点 | 输入 | 输出/作用 |
| --- | --- | --- |
| `bottle_perception` | RGB、深度、CameraInfo、TF | 3D 检测、抓取位姿、预测位姿、速度、调试图 |
| `obstacle_cloud` | 配准点云、瓶体位姿、TF | 去除桌面和目标瓶后的障碍点云 |
| `planning_scene_obstacles` | 融合后的体素模型 | 将体素盒写入 MoveIt PlanningScene |
| `moving_bottle` | 参数/服务 | Gazebo 瓶体运动、停止和复位 |
| `moving_bottle_grasp` | 抓取位姿、跟踪速度 | MoveIt 拦截、闭合、附着、抬升 |

关键输出话题：

- `/bottle_grasp/detections_3d`
- `/bottle_grasp/grasp_pose`
- `/bottle_grasp/predicted_grasp_pose`
- `/bottle_grasp/tracked_twist`
- `/bottle_grasp/obstacle_points`
- `/bottle_grasp/reconstructed_obstacles`
- `/bottle_grasp/status`
- `/bottle_perception/debug_image`

## 3. 为什么选择 YOLO26n-seg

选择轻量的 YOLO26n 分割模型，而不是仅输出检测框的模型。分割掩膜能排除框内桌面背景，使深度中值、瓶体范围和抓取位置更稳定；`n` 型号也适合当前 CPU 仿真以及后续边缘设备部署。

COCO 预训练模型对 Gazebo 顶视角中的瓶盖/圆形瓶体没有可靠响应，因此项目内提供了针对当前仿真视角微调的权重：

`models/bottle_yolo26n_seg.pt`

训练集由仿真自动采集 80 帧（64 训练、16 验证）。本次验证结果为：框 mAP50 0.995、框 mAP50-95 0.940、掩膜 mAP50 0.995、掩膜 mAP50-95 0.905。实机部署仍应采集 Gemini Pro 图像继续微调，尤其覆盖透明瓶、反光、不同标签、液位和光照。

## 4. 构建与运行

可选 Python 依赖：

```bash
python3 -m pip install ultralytics opencv-python pyyaml
```

如需启用 MoveIt 原生 OctoMap 更新器，还需系统包（当前启动文件同时提供无需该插件的 PlanningScene 回退）：

```bash
sudo apt-get install ros-jazzy-moveit-ros-perception
```

构建：

```bash
cd /home/zhul/online/guohua_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-select \
  zhishu_robot_description \
  zhishu_robot_gazebo \
  zhishu_robot_moveit_config \
  zhishu_robot_mtc_pick_place \
  zhishu_robot_bottle_grasp
source install/setup.bash
```

无界面全自动动态抓取：

```bash
ros2 launch zhishu_robot_bottle_grasp bottle_grasp_sim.launch.py \
  headless:=true use_rviz:=false detector_backend:=yolo execute_grasp:=true
```

需要观察规划与点云时：

```bash
ros2 launch zhishu_robot_bottle_grasp bottle_grasp_sim.launch.py \
  headless:=false use_rviz:=true detector_backend:=yolo execute_grasp:=true
```

`detector_backend:=color` 仅用于验证相机、TF 与深度几何，不是最终识别方案。可用 `execute_grasp:=false` 只运行感知与跟踪。

正常完成标志：

```text
Moving bottle grasp and lift completed
```

停止整套程序时，在启动终端按 `Ctrl-C`。如需确认没有残留：

```bash
ps -eo pid,ppid,args | rg 'ros2 launch|move_group|gz sim|bottle_perception|moving_bottle|obstacle_cloud'
```

## 5. 关键参数

感知参数位于 `config/bottle_grasp.yaml`：

- `confidence`：YOLO 置信度阈值。
- `mask_erode_pixels`：向内收缩掩膜，减少边缘混合深度。
- `minimum_points`：三维估计所需最少有效深度点。
- `prediction_horizon`：跟踪位置前向预测时间。
- `tracker_alpha`、`tracker_beta`：位置/速度滤波系数。

抓取执行参数在 `launch/bottle_grasp_sim.launch.py` 中传入：

- `intercept_x/y`：夹爪 TCP 的拦截位置。
- `grasp_z_offset=0.025`：由深度抓取带到夹爪 TCP 的竖直标定偏移。
- `trigger_y=-0.19`：瓶体首次接触张开夹指时的触发位置；它与 TCP 的 `intercept_y` 分开标定。
- `trigger_lead/tolerance`：接触触发窗口。
- `intercept_timeout`：等待移动瓶进入窗口的超时。

换夹爪、瓶型或输送速度后，应先重新标定 `grasp_z_offset` 和 `trigger_y`。

## 6. Gemini Pro 实机接入与手眼标定

仓库已包含 Orbbec ROS 2 SDK：`src/third_party/orbbec_sdk_ros2`。实机应启用彩色、深度、点云和 D2C（深度对齐到彩色），并将 SDK 话题重映射为本包约定的话题。详细的 Gemini Pro 启动、USB 权限和参数说明见：

`src/zhishu_robot_handeye_calibration/docs/GEMINI_PRO_CAMERA.zh-CN.md`

必须复用手眼标定得到的真实外参，将相机光学坐标系正确连接到 `base_link`。本项目在 `head_camera_handeye.xacro` 中增加的顶置安装仅供仿真使用，不应覆盖实机标定结果。上线前至少检查：

```bash
ros2 topic hz /camera_head/color/image_raw
ros2 topic hz /camera_head/depth/image_raw
ros2 topic echo --once /camera_head/color/camera_info
ros2 run tf2_ros tf2_echo base_link camera_head_color_optical_frame
```

然后先设置 `execute_grasp:=false` 验证调试图和三维抓取位姿，再在低速、限位和急停可用的条件下启用运动。

## 7. 重新生成仿真训练集

先启动仿真但关闭执行器，再运行采集节点：

```bash
ros2 launch zhishu_robot_bottle_grasp bottle_grasp_sim.launch.py \
  execute_grasp:=false run_perception:=false

ros2 run zhishu_robot_bottle_grasp capture_yolo_dataset --ros-args \
  -p output_dir:=/tmp/bottle_yolo_dataset -p max_images:=80
```

训练并导出权重：

```bash
python3 src/zhishu_robot_bottle_grasp/scripts/train_sim_yolo.py \
  --dataset /tmp/bottle_yolo_dataset \
  --weights yolo26n-seg.pt \
  --output src/zhishu_robot_bottle_grasp/models/bottle_yolo26n_seg.pt
```

采集节点利用当前仿真的目标颜色自动生成分割标签，只适用于合成数据；实机数据应人工标注或经过人工复核。

## 8. 点云避障与三维重建路线

当前阶段已打通实时路径：Gemini Pro/Gazebo 注册点云经过工作空间裁剪、下采样、桌面滤除和目标瓶排除，再以 3 cm 体素进行多帧融合。体素至少连续命中两次才成为障碍，2 秒未再次观测则衰减；仿真时钟回退会自动清空旧地图。光学帧点云发布到 `/bottle_grasp/obstacle_points` 供 MoveIt OctoMap 使用，`base_link` 下的模型发布到 `/bottle_grasp/reconstructed_obstacles`。

为了兼容尚未安装 `moveit_ros_perception` 的机器，`planning_scene_obstacles` 会把确认体素直接作为一个多盒碰撞对象 `rgbd_reconstructed_obstacles` 写入 MoveIt PlanningScene。因此基础避障不依赖额外系统插件；安装插件后还可并行使用 `sensors_3d.yaml` 的射线清障 OctoMap。固定输送台与障碍箱也作为独立碰撞物加入 PlanningScene。

这是在线避障所需的第一阶段，但还不是完整的持久三维重建。后续建议按以下顺序推进：

1. 用静态场景和移动机械臂回放验证 OctoMap 清除、TF 时间同步及自遮挡过滤。
2. 增加机器人自滤波与传感器噪声模型，避免机械臂点云成为自身障碍。
3. 对静态障碍采用 TSDF/ESDF 或持久 OctoMap 融合；动态障碍保留短时占据层。
4. 将抓取规划改为持续重规划/伺服拦截，并把相机延迟、YOLO 推理延迟和执行器制动距离纳入预测。
5. 在实机中逐级提高瓶速，记录抓取成功率、定位误差、最小障碍间距和急停测试结果。

## 9. 本次验收

- 彩色诊断后端已完成一次完整动态抓取。
- 自定义 YOLO26n-seg 后端在启用体素重建碰撞物后完成完整动态抓取。
- 仿真点云实测约 1.07 Hz，每帧约 109 个确认体素；MoveIt 世界已验证包含 `rgbd_reconstructed_obstacles`。
- 单元测试覆盖 RGB-D 几何、alpha-beta 跟踪、体素命中确认、衰减和时钟回退。
- 程序退出后已确认无本次启动的 ROS、MoveIt 或 Gazebo 残留进程。
