# Zhishu robot logging

本包将 ROS 2 `/rosout` 汇总为便于搜索的单一文本日志，并保留终端输出和
ROS 原生进程日志。日志写盘在独立进程中完成，不进入 1 kHz EtherCAT 控制线程。

默认目录结构：

```text
<colcon-workspace>/
  build/
  install/
  log/
  zhishu_robot_logs/YYYY-MM-DD/HH-MM-SS/
    zhishu_robot_YYYYMMDD_HHMMSS_PID.log
    zhishu_robot_YYYYMMDD_HHMMSS_PID_part001.log
    ethercat_kernel.log
    native/
```

每行包含本地时间、严重级别、ROS logger、源文件/行号、函数和消息。消息中的
换行符会被转义，以保证一个事件只占一个物理行。文件达到配置大小后自动轮转。

主 bringup 默认启动日志记录：

```bash
ros2 launch zhishu_robot_bringup zhishu_robot.moveit.launch.py \
  enable_log_recorder:=true \
  log_root:=/home/zhul/robot_logs \
  log_session:=09-35-14 \
  log_max_file_size_mb:=10 \
  log_min_severity:=10
```

严重级别为 `DEBUG=10`、`INFO=20`、`WARN=30`、`ERROR=40`、`FATAL=50`。
也可以通过 `ZHISHU_ROBOT_LOG_DIR` 修改默认根目录。使用
`enable_log_recorder:=false` 关闭 `/rosout` 汇总记录。

`ethercat_kernel.log` 仅记录启动后新增且匹配 `EtherCAT`、`Link state`、
`TIMED OUT`、`UNMATCHED`、`SKIPPED` 或 `Synchronization` 的内核消息。使用
`enable_kernel_ethercat_log:=false` 可关闭此功能。

Ubuntu 通常限制普通用户读取内核消息。在机器人上编译并 source 工作空间后，
为当前普通用户执行一次：

```bash
sudo $(ros2 pkg prefix zhishu_robot_logging)/lib/zhishu_robot_logging/setup_kernel_ethercat_sudo.sh
```

脚本只允许免密码执行精确的只读命令
`/usr/bin/dmesg --follow-new --ctime`。采集器始终使用 `sudo -n`；如果权限未配置，
它会立即退出并把原因写入 `ethercat_kernel.log`，不会等待密码或阻塞 bringup。

独立启动日志程序：

```bash
ros2 launch zhishu_robot_logging log_recorder.launch.py
```
