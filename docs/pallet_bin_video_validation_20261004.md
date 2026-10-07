# 15 kg 首箱完整搬运录像验证（2026-10-04）

## 结果

从未移动的初始料箱阵列开始，外围箱 `r0_c0` 连续完成右臂夹箱壁预抽出约
120 mm、物理位姿重新定位、双臂抓取和后撤、腰臂协调抬升、底盘绕行、
同步入箱、松爪竖直撤离和双臂回位。程序正常退出。
本次已包含长指启动与最终停车腰部修正：升降 0.10 m、腰部 -0.30 rad，
回位后不再盲目归零。最终箱体中心 `(1.032455, -1.550016, 0.636000) m`，
最终倾角约 0.000088°；1 Hz 遥测采样最大倾角 0.415°，入箱阶段 0.208°。
协调抬升实测中心高度 0.9641 m。

## 视频与证据

输出目录：`artifacts/pallet_bin_video_20261004/`。

- `full_process.webm`：完整原速录像，24 分 02.8 秒，960×620、10 fps。
- `overview_4x.webm`：明确标注的 4 倍速版本，6 分 00.7 秒。
- `telemetry.jsonl`：1 Hz 实际 Gazebo 箱体、底盘与腰部状态。
- `recording_summary.json`：录像结果与相机位置。
- `stage_*.png`：阶段截图。
- `transfer.log`、`scene.log`、`recorder.log`：本轮日志副本。

抓取近景、场景全景、入箱近景按执行阶段自动切换。画面来源是实际 Gazebo
观察相机，不是动画回放或桌面录屏。相机模型没有碰撞体，观察世界只增加相机
及调整显示对比度，未更改原世界的物理参数。原速采用墙钟时间，仿真非恒定
实时运行，故不能据此认定真实机器人作业节拍。已抽查两份视频首、中、末帧可解码。

## 复现

终端先加载 `/opt/ros/jazzy/setup.bash` 和工作区 `install/setup.bash`。
各终端使用相同的空闲 `ROS_DOMAIN_ID` 和 `GZ_PARTITION`。

```bash
python3 scripts/record_pallet_bin_transfer.py world \
  --source src/zhishu_robot_gazebo/worlds/pallet_bin_15kg.sdf \
  --output /tmp/pallet_video.sdf
ros2 launch zhishu_robot_mtc_pick_place pallet_bin_15kg_transfer.launch.py \
  world_file:=/tmp/pallet_video.sdf start_transfer:=false \
  headless:=true use_rviz:=false
```

场景和控制器就绪后，在另一个终端先录制：

```bash
python3 scripts/record_pallet_bin_transfer.py record \
  --output-dir artifacts/pallet_bin_video_new \
  --log /tmp/pallet-video-transfer-new.out --max-seconds 2400
```

再在第三个终端执行：

```bash
ros2 launch zhishu_robot_mtc_pick_place pallet_bin_15kg_transfer.launch.py \
  reuse_gazebo:=true transfer_start_delay:=0 headless:=true use_rviz:=false \
  execute:=true validate_all_nine:=false full_transfer_box_index:=0 \
  pre_extract_enabled:=true > /tmp/pallet-video-transfer-new.out 2>&1
```

使用已有 Gazebo Python transport、Pillow、NumPy、ROS Python 与 GStreamer
VP8/WebM 编码，无需启动 Gazebo 界面。复测应使用新的日志文件与输出目录。

## 边界

本次只验证首个外围箱，不代表已清理全部九箱或完成中心箱搬运。
重新定位使用 Gazebo 物理位姿，不是真实视觉识别。腕部力传感器数据未就绪。
320 mm 长指、摩擦、质量分布、支撑代理与腰部力矩仍是候选仿真参数，
不能代替实机负载、稳定性或全部自碰撞安全认证；详见升级版候选验证记录。
