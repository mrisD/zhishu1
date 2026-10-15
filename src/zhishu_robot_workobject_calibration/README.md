# 工件坐标系三点标定 Gazebo 仿真

参照 ABB X1/X2/Y1 工件用户坐标系三点示教流程：已标定 TCP 分别对准三个基准尖针，记录法兰 TF，使用 `base_T_flange * flange_T_tcp` 得到测量点；X1→X2 为 +X，Y1 确定 +Y 半平面，右手规则确定 +Z。

默认 `origin_mode:=axis_intersection`：原点是 Y1 在 X1—X2 直线上的垂足。可选 `first_point`：以 X1 为原点。这两个约定不能混用。方法定义参考 ABB RAPID 技术手册 `DefFrame` 的 Origin=3 / Origin=1：
https://library.e.abb.com/public/688894b98123f87bc1257cc50044e809/Technical%20reference%20manual_RAPID_3HAC16581-1_revJ_en.pdf

用户提供的 B 站 p=28 页面在开发环境无法播放，未声称逐帧复现；这里实现可核验的官方三点法。

## 运行

```bash
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select zhishu_robot_workobject_calibration
source install/setup.bash
ros2 launch zhishu_robot_workobject_calibration workobject_gazebo.launch.py output_dir:=/tmp/workobject_validation
```

专用 ROS_DOMAIN_ID=68、GZ_PARTITION=workobject_calibration_sim，不连接实机或正在运行的 domain168 程序。启动固定底座机械臂、仅挂载右臂的已标定探针、带 X1/X2/Y1 三尖针的倾斜工件台。非工作臂不挂载多余长探针，避免对工件场景造成干扰；原 TCP 仿真仍默认双探针。桌板、立柱和针杆同时进入 Gazebo 和 MoveIt 碰撞场景；尖端为几何对准目标，不模拟力接触。规划失败、接近路径不完整或运动执行失败立即停止，禁止绕过碰撞检查。全部下发为完整双臂轨迹，左臂显式保持初始实测关节位置，速度和加速度为零，漂移超过 0.02 rad 取消轨迹。

流程：等待控制器 → 三次安全转移/直线接近/稳定测量/直线退回 → 三点解算 → 显示 RGB 工件坐标轴并发布静态 TF → 按新工件坐标移动至三个未用于标定的点 → 在工件面上方 55 mm 沿工件 X/Y 轴走 60×50 mm 矩形 → 返回初始姿态。青色线框表示该矩形，没有碰撞实体。

输出 `calibration.yaml`、含法兰实测位姿的 `samples.csv`，成功时写入 `SUCCESS`。验收要求原点误差 <1 mm、坐标系转角误差 <1°、三个独立目标位置误差各 <1 mm、矩形四边的实测 TCP 到独立真值线段最大距离各 <1 mm。矩形执行期间观测偏差超过 2 mm 将取消轨迹。仿真真值仅用于模拟操作员生成接近点及独立验算，不进入求解器；测量使用前次 TCP 标定文件，不直接使用探针真值。

本例 object frame 相对 user frame 为单位变换，输出显式记录 `base_T_user`、`user_T_object`、`base_T_object`，TF 链为 `base_link → workobject_user → workobject_calibrated`。姿态为四元数 xyzw，位置单位米。真实应用必须额外管理对象偏移、示教安全和控制器坐标约定，不能直接拿本自动采样节点驱动实机。

改变工件位姿可使用 `board_x/y/z`、`board_roll/pitch/yaw`；已运行同一仿真时 `sampler_only:=true` 只启动新的标定节点（须先停止旧采样节点以避免重复发布同名 TF）。`noise_sigma_mm` 仅向采样点注入可复现高斯噪声，不改动真值。保存报告和 GUI 截图后再停止节点。

纯数学验证：`python3 -m unittest discover -s src/zhishu_robot_workobject_calibration/test -v`。

ROS 工作空间内使用 `colcon test --packages-select zhishu_robot_workobject_calibration`，包含 7 项数学测试和 3 项探针选择/默认兼容性测试。
