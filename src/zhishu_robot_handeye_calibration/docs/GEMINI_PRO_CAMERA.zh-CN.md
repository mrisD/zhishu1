# Gemini Pro 官方驱动与 RGB 内参标定

本文配置头部 Gemini Pro 相机，不启动机器人、MoveIt、ros2_control 或 EtherCAT。
RGB 用于当前手眼标定；深度、硬件 D2C、深度点云和彩色点云同时接通，为后续功能预留。

## 已验证配置

- 相机：ORBBEC WHITETEC Gemini Pro，SDK 识别名称 `SV1301S_U3`
- 深度接口：`2bc5:0614`
- RGB 接口：`2bc5:0511`
- 序列号：`AY2755200Y8`
- 连接：USB 2.0
- 固件：`RD3013`
- ROS 2：Jazzy，Ubuntu 24.04，x86_64
- Orbbec Wrapper：`1.5.15`
- Orbbec SDK：`1.10.35`
- RGB：`640 x 480 @ 30 Hz`，设备格式 MJPG，ROS 编码 `rgb8`
- 深度：原始配置 `640 x 400 @ 30 Hz`、Y11；硬件 D2C 后发布为
  `640 x 480`、`16UC1`

旧款 Gemini Pro 使用 OrbbecSDK ROS2 的 `main`（v1.x）分支。不要换成
`v2-main`，后者不支持传统 OpenNI 协议设备。

## 主工程内置官方驱动

官方驱动已经放入主工作空间：

```text
src/third_party/orbbec_sdk_ros2/orbbec_camera_msgs
src/third_party/orbbec_sdk_ros2/orbbec_camera
```

因此不再需要单独创建或 source `orbbec_jazzy_ws`。首次构建前安装系统依赖：

```bash
sudo apt install -y \
  ros-jazzy-image-publisher \
  nlohmann-json3-dev \
  libgoogle-glog-dev

cd /home/zhul/online/guohua_ws
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install --packages-up-to zhishu_robot_handeye_calibration \
  --cmake-args -DCMAKE_BUILD_TYPE=Release
```

安装经过核对的 udev 规则：

```bash
sudo install -m 0644 \
  /home/zhul/online/guohua_ws/src/third_party/orbbec_sdk_ros2/orbbec_camera/scripts/99-obsensor-libusb.rules \
  /etc/udev/rules.d/99-obsensor-libusb.rules
sudo udevadm control --reload-rules
sudo udevadm trigger --subsystem-match=usb
```

重新插拔相机后，设备节点应为 `root:video`、权限 `0666`。可验证：

```bash
source /opt/ros/jazzy/setup.bash
source /home/zhul/online/guohua_ws/install/setup.bash
ros2 run orbbec_camera list_devices_node
```

输出应包含 `SV1301S_U3`、PID `0x0614`、序列号 `AY2755200Y8`。

## 构建并启动主工程

驱动与主工程在同一个工作空间中构建，每个新终端只需 source 主工程：

```bash
cd /home/zhul/online/guohua_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-up-to zhishu_robot_handeye_calibration
source install/setup.bash
ros2 launch zhishu_robot_handeye_calibration gemini_pro_camera.launch.py
```

不要同时启动旧的 `v4l2_camera`、OBS、浏览器相机页面或其他占用 Gemini Pro 的程序。

默认开启：

```text
/camera_head/color/image_raw             RGB 图像
/camera_head/color/camera_info            RGB 内参
/camera_head/depth/image_raw              对齐到 RGB 的深度图
/camera_head/depth/camera_info            对齐后的深度内参
/camera_head/depth_to_color/image_raw     D2C 可视叠加图
/camera_head/depth/points                 深度点云（xyz）
/camera_head/depth_registered/points      彩色点云（xyz + rgb）
/camera_head/depth_to_color               深度到 RGB 的设备外参
```

使用 `zhishu_robot.handeye.launch.py` 时会自动加载手眼标定专用 RViz 配置：
Target 页预填 7×5 ChArUco、`DICT_4X4_50`、30 mm 方格和 22 mm marker，Context
页预填 Eye-to-Hand、`head_camera_optical_frame`、`handeye_target`、`L_hand_link`
和 `base_link`。RViz 同时显示原始 RGB 话题与插件检测输出
`/handeye_calibration/target_detection`；无需再从 By topic 手工添加。

RGB、注册深度和两类点云统一使用：

```text
frame_id: head_camera_optical_frame
```

这个 TF 由机器人模型发布，官方相机节点默认 `publish_tf:=false`，避免出现同名 TF
的多个发布者。点云默认 `ordered_pc:=true`，尺寸为 `640 x 480`；厂商驱动的无序点云
模式在过滤无效点后没有同步更新 `row_step`，不建议使用。

只需临时验证 RGB 时，可降低负载：

```bash
ros2 launch zhishu_robot_handeye_calibration gemini_pro_camera.launch.py \
  enable_depth:=false \
  enable_d2c:=false \
  enable_point_cloud:=false \
  enable_colored_point_cloud:=false
```

## ChArUco RGB 内参标定

驱动在没有用户 YAML 时会发布设备工厂内参。手眼标定仍以最终分辨率下重新标定的
RGB 内参为准。保持相机 launch 运行，在第二个终端执行：

```bash
cd /home/zhul/online/guohua_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 run zhishu_robot_handeye_calibration calibrate_charuco_intrinsics.py
```

标定板参数固定为：

```text
DICT_4X4_50
7 x 5 个大方格
square_length = 0.030 m
marker_length = 0.022 m
实物外框 = 240 x 220 mm（外框不参与角点几何计算）
```

窗口快捷键：

- `Space`：保存当前有效画面；
- `R`：删除上一张；
- `C`：计算但不保存；
- `S`：计算并保存；
- `Q` 或 `Esc`：退出。

建议采集 30～50 张清晰画面，覆盖中心、四角、边缘、不同距离和板面倾角。避免逆光、
反光和运动模糊。只有 RMS 重投影误差不超过 `0.8 px` 才会保存，建议达到 `0.5 px`
以下。

输出文件：

```text
~/.ros/camera_info/gemini_pro_rgb_640x480.yaml
~/.ros/camera_info/gemini_pro_rgb_640x480.report.yaml
```

停止并重新启动相机后，launch 会自动加载第一个文件，并替代工厂 RGB CameraInfo。
标定程序写入的 `camera_name` 默认为 `camera_head_color`，与官方驱动的 RGB
CameraInfo 管理器一致。
也可以显式指定：

```bash
ros2 launch zhishu_robot_handeye_calibration gemini_pro_camera.launch.py \
  camera_info_url:=file:///home/zhul/.ros/camera_info/gemini_pro_rgb_640x480.yaml
```

## 验证

```bash
ros2 topic hz /camera_head/color/image_raw
ros2 topic hz /camera_head/depth/image_raw
ros2 topic hz /camera_head/depth_to_color/image_raw
ros2 topic hz /camera_head/depth/points
ros2 topic hz /camera_head/depth_registered/points

ros2 run zhishu_robot_handeye_calibration check_camera_info.py
ros2 run zhishu_robot_handeye_calibration check_camera_target.py
```

五路数据在真机验收中均达到约 `30 Hz`。同时订阅 RGB、深度、D2C 和两类完整有序点云
时，测试 PC 上相机容器约占用 `63%` CPU、`135 MB` RSS；点云没有订阅者时驱动不会
持续构造点云，实际负载会降低。

`check_camera_target.py` 会把结果保存到：

```text
/tmp/handeye_camera_check.png
```

标定板没有放入画面时检测到 0 个 marker 是正常的；采样前应稳定看到至少 6 个 marker。

## 常见问题

### `usbEnumerator createUsbDevice failed`

检查 `/dev/bus/usb` 下 `2bc5:0614` 和 `2bc5:0511` 对应节点是否具有写权限。重新安装
udev 规则并重新插拔相机。

### 相机被占用

完全退出旧 V4L2 节点、OBS 和浏览器。官方 SDK 必须同时控制深度接口与 RGB UVC 接口。

### CameraInfo 提示 `rational_polynomial`

这是设备工厂内参的畸变模型，不代表驱动失败。完成 ChArUco 内参标定并重启后，项目
生成的 YAML 会提供 `plumb_bob` 模型。

### 点云尺寸异常

保持默认 `ordered_pc:=true`。不要启用厂商驱动的无序点云模式；该版本在无序模式下
存在 `width/data` 已缩减但 `row_step` 保留原值的问题。
