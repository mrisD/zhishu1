# 重启后第三排导纳验证

当前测试：`output/pallet_full_process_admittance_20261018/third_row_diagnostic5`，仅测试第三排 `[6,7,8]`，不与旧前三箱合并成九箱通过，本轮不录制。

旧轮次 third_row_diagnostic4 的 Gazebo Transport 固定在已失效的 Wi-Fi 地址 192.168.110.147，而接口地址后来变为 192.168.110.195。ROS DDS 回环配置未覆盖 Gazebo Transport，导致时钟、腕力和动作不再推进。

已将 Gazebo 的 `GZ_IP` 和 `GZ_RELAY` 固定为 `127.0.0.1`；ROS DDS 固定本机 lo。每轮生成 `network_config.json`，服务器和 GUI 共用；新 GUI 入口为 `scripts/open_pallet_sim_gui.py --run-dir <本轮目录>`。不再手工读取无线网卡地址打开界面。

重启后检查：无旧 Gazebo 或搬运主进程残留，现有三个 CTest 目标通过。新轮次连续 15 秒通信检查通过：仿真时间从 39.098 s 推进至 52.428 s，接收 2665 条关节状态、左右各 3331 条腕力；数据有限且末次接收新鲜，Gazebo 场景服务可达并含机器人。完整报告为本轮 `communication_health.json`。此为启动期通信检查，搬运通过仍须依据本轮独立验收结果。

外层新增无数据监测：每箱启动宽限 90 秒后，腕力 CSV 超过 30 秒不再增长则拒绝本轮并清理进程，避免只剩进程和窗口却持续等待。时钟/通信检查命令：

```bash
bash scripts/run_with_gz_control_safe_overlay.sh python3 scripts/check_pallet_sim_feedback.py \
  --run-dir output/pallet_full_process_admittance_20261018/third_row_diagnostic5 --seconds 15
```

运动沿用提速版：载物底盘线/角速度 0.12 m/s、0.18 rad/s；空载接近/回零反馈段 0.5 秒，接触及载物关键段 0.25 秒；第三排接触闭爪 0.25 mm/s，保留实测增量限幅、65 N·m软退让、100 N·m/450 N硬保护和物理抬升检查。
