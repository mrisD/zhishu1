# zhishu_robot_mcp

`zhishu_robot_mcp` 是智枢双臂机器人的 C++17 MCP Server。它通过 `rclcpp`
读取 ROS 2 状态，并复用 `zhishu_robot_commander_cpp` 通过 MoveIt 规划运动。MCP
协议层使用社区维护的 [`cxxmcp`](https://github.com/caomengxuan666/cxxmcp)，固定为
随包提供的 `v1.2.8` SDK 源码。

Server 使用本地 `stdio` 传输，不监听网络端口。标准输出只传输 MCP JSON-RPC，
ROS 日志被强制写到标准错误。

## 安全边界

- 写功能默认关闭，必须在启动进程时设置 `ZHISHU_MCP_ENABLE_WRITES=1`。
- 真机还需要第二个独立开关 `ZHISHU_MCP_ALLOW_PHYSICAL=1`。
- MCP 工具不能修改这些启动策略。
- 不提供任意 shell、任意 topic publish 或任意 service call，只暴露固定白名单接口。
- 所有运动通过 MoveIt 规划，真机默认速度和加速度缩放为 `0.10`。
- 真机关节数组目标要求 `/joint_states` 不超过 1 秒，并限制单次关节变化量。
- 一次只接受一个 MCP 运动任务；运动异步执行，可以查询状态或请求停止。
- `stop_motion` 只是 MoveIt 软件停止请求，不能代替实体急停、安全 PLC 或断电回路。
- 故障复位不会自动使能电机；两步操作保持分离。
- MCP `confirmation` 字段用于防止误调用，不是身份认证。真正的权限边界是启动时写开关、
  真机双重开关和固定接口白名单。

## MCP 工具

只读工具：

- `get_server_info`
- `get_robot_status`
- `get_ethercat_status`
- `list_ros_nodes`
- `list_ros_topics`
- `list_recent_logs`
- `read_log`
- `get_motion_status`

写工具：

- `move_named_target`
- `move_joint_target`
- `set_gripper`
- `stop_motion`
- `reset_arm_fault`
- `set_motors_enabled`

每个工具都发布 MCP `readOnlyHint`、`destructiveHint`、`idempotentHint` 和
`openWorldHint`。这些注解只帮助客户端展示风险，服务端仍会独立执行权限检查。

## 编译

`cxxmcp v1.2.8` 的完整 SDK 源码和许可证已放在 `third_party/cxxmcp/` 中，
首次构建和清理后重建均不需要访问 GitHub：

```bash
cd /home/pi/Documents/zhul/guohua_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-up-to zhishu_robot_mcp
source install/setup.bash
```

依赖来源和发布归档校验值记录在 `third_party/README.md`。

## 启动模式

只读模式是默认值：

```bash
export ZHISHU_MCP_WORKSPACE=/home/pi/Documents/zhul/guohua_ws
ros2 run zhishu_robot_mcp zhishu_robot_mcp_server
```

仿真可写模式：

```bash
export ZHISHU_MCP_ENABLE_WRITES=1
export ZHISHU_MCP_ROBOT_MODE=simulation
export ZHISHU_MCP_WORKSPACE=/home/pi/Documents/zhul/guohua_ws
ros2 run zhishu_robot_mcp zhishu_robot_mcp_server
```

真机可写模式需要双重启动开关：

```bash
export ZHISHU_MCP_ENABLE_WRITES=1
export ZHISHU_MCP_ROBOT_MODE=physical
export ZHISHU_MCP_ALLOW_PHYSICAL=1
export ZHISHU_MCP_MAX_WRITES_PER_MINUTE=10
export ZHISHU_MCP_MAX_JOINT_STEP_RAD=0.35
export ZHISHU_MCP_WORKSPACE=/home/pi/Documents/zhul/guohua_ws
ros2 run zhishu_robot_mcp zhishu_robot_mcp_server
```

真机模式只应在操作员能够触及实体急停、工作区已经清空且机器人已完成现场检查时启用。

## MCP 客户端进程配置

客户端必须直接启动 Server 进程并保留 stdio。通用配置等价于：

```json
{
  "command": "bash",
  "args": [
    "-lc",
    "source /opt/ros/jazzy/setup.bash && source /home/pi/Documents/zhul/guohua_ws/install/setup.bash && exec ros2 run zhishu_robot_mcp zhishu_robot_mcp_server"
  ],
  "env": {
    "ZHISHU_MCP_WORKSPACE": "/home/pi/Documents/zhul/guohua_ws",
    "ZHISHU_MCP_ROBOT_MODE": "simulation",
    "ZHISHU_MCP_ENABLE_WRITES": "0"
  }
}
```

不同 MCP Host 的外层配置字段可能不同，但 `command`、`args`、`env` 的进程语义相同。

## 写操作确认值

- 机械臂或夹爪运动：`MOVE ROBOT`
- 故障复位：`RESET FAULT`
- 电机使能：`ENABLE MOTORS`
- 电机禁用：`DISABLE MOTORS`

运动接口接受请求后立即返回 `motion-*` ID。使用 `get_motion_status` 查询
`queued`、`running`、`succeeded`、`failed`、`stop_requested` 或 `stopped` 状态。

## 日志范围

MCP 只允许读取以下两个根目录中的 `.log` 和 `.txt`：

```text
robot:     $ZHISHU_ROBOT_LOG_DIR 或 <workspace>/zhishu_robot_logs
stability: <workspace>/log/ethercat_stability
```

绝对路径、`..` 越界、符号链接越界和其他扩展名都会被拒绝；当前单次日志读取窗口为
256 KiB（实现上限 1 MiB），并默认只返回最后 200 行。
