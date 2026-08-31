# zhishu_robot_rest_api

`zhishu_robot_rest_api` 是独立的 C++17 ROS 2 REST API 包。它不依赖
`zhishu_robot_mcp` 或 `cxxmcp`，HTTP 层使用
[`cpp-httplib`](https://github.com/yhirose/cpp-httplib)，JSON 使用
[`nlohmann/json`](https://github.com/nlohmann/json)。机器人运动仍通过工程现有的
`zhishu_robot_commander_cpp` 和 MoveIt 执行。

## 默认安全边界

- 默认只监听 `127.0.0.1:8088`，默认只读。
- 开启写接口时必须配置至少 24 个字符的 Bearer Token。
- 非回环地址必须同时设置 `ZHISHU_REST_ALLOW_REMOTE=1` 和 Bearer Token。
- 真机写入还需要 `ZHISHU_REST_ALLOW_PHYSICAL=1`。
- 不提供任意 shell、任意 ROS topic publish 或任意 service call。
- 固定 EtherCAT 查询使用 3 秒超时；请求体默认限制 64 KiB。
- 真机 MoveIt 速度和加速度缩放为 `0.10`，关节目标要求新鲜状态和单步限幅。
- 故障复位与电机使能保持为两个独立接口；复位不会自动使能电机。
- `stop` 是 MoveIt 软件停止，不能代替实体急停或安全回路。
- HTTP 本身不提供 TLS。跨机器使用时应放在 HTTPS 反向代理或可信 VPN 后面。

## 编译

```bash
cd /home/pi/Documents/zhul/guohua_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-up-to zhishu_robot_rest_api
source install/setup.bash
```

`nlohmann/json v3.12.0` 和 `cpp-httplib v0.46.0` 的单头文件及许可证已随包放在
`third_party/` 中。首次构建和清理后重建均不需要访问 GitHub。

## 启动

默认本机只读：

```bash
export ZHISHU_REST_WORKSPACE=/home/pi/Documents/zhul/guohua_ws
ros2 run zhishu_robot_rest_api zhishu_robot_rest_api_server
```

仿真可写：

```bash
export ZHISHU_REST_API_TOKEN="$(openssl rand -hex 32)"
export ZHISHU_REST_HOST=127.0.0.1
export ZHISHU_REST_PORT=8088
export ZHISHU_REST_ENABLE_WRITES=1
export ZHISHU_REST_ROBOT_MODE=simulation
ros2 run zhishu_robot_rest_api zhishu_robot_rest_api_server
```

真机可写：

```bash
export ZHISHU_REST_API_TOKEN="$(openssl rand -hex 32)"
export ZHISHU_REST_ENABLE_WRITES=1
export ZHISHU_REST_ROBOT_MODE=physical
export ZHISHU_REST_ALLOW_PHYSICAL=1
export ZHISHU_REST_MAX_WRITES_PER_MINUTE=10
export ZHISHU_REST_MAX_JOINT_STEP_RAD=0.35
ros2 run zhishu_robot_rest_api zhishu_robot_rest_api_server
```

远程监听还必须显式设置：

```bash
export ZHISHU_REST_HOST=0.0.0.0
export ZHISHU_REST_ALLOW_REMOTE=1
```

## API

所有正常 JSON 响应都使用统一结构：

```json
{
  "ok": true,
  "code": "ok",
  "message": "",
  "data": {},
  "request_id": "rest-1",
  "timestamp": "2026-08-30T12:00:00Z"
}
```

只读接口：

- `GET /api/v1/health`
- `GET /api/v1/openapi.json`
- `GET /api/v1/server`
- `GET /api/v1/robot/status`
- `GET /api/v1/ethercat/status`
- `GET /api/v1/ros/nodes`
- `GET /api/v1/ros/topics?filter=&limit=200`
- `GET /api/v1/logs?limit=50`
- `POST /api/v1/logs/read`
- `GET /api/v1/motions/current`

写接口：

- `POST /api/v1/motions/named`
- `POST /api/v1/motions/joints`
- `POST /api/v1/grippers`
- `POST /api/v1/motions/stop`
- `POST /api/v1/faults/reset`
- `PUT /api/v1/motors/enabled`

完整请求模式见安装后的 OpenAPI 文档，运行时也可访问：

```text
http://127.0.0.1:8088/api/v1/openapi.json
```

## 调用示例

查询状态：

```bash
curl --fail-with-body http://127.0.0.1:8088/api/v1/robot/status
```

仿真运动：

```bash
curl --fail-with-body \
  -H "Authorization: Bearer ${ZHISHU_REST_API_TOKEN}" \
  -H 'Content-Type: application/json' \
  -d '{"group":"both_arms","target":"both_arms_home","confirmation":"MOVE ROBOT"}' \
  http://127.0.0.1:8088/api/v1/motions/named
```

查询异步运动状态：

```bash
curl --fail-with-body http://127.0.0.1:8088/api/v1/motions/current
```

确认短语必须完全匹配：

- 运动和夹爪：`MOVE ROBOT`
- 故障复位：`RESET FAULT`
- 电机使能：`ENABLE MOTORS`
- 电机禁用：`DISABLE MOTORS`

## 环境变量

| 变量 | 默认值 | 作用 |
|---|---:|---|
| `ZHISHU_REST_HOST` | `127.0.0.1` | HTTP 监听地址 |
| `ZHISHU_REST_PORT` | `8088` | HTTP 端口 |
| `ZHISHU_REST_ALLOW_REMOTE` | `0` | 允许非回环监听 |
| `ZHISHU_REST_API_TOKEN` | 空 | 至少 24 字符；写或远程模式必需 |
| `ZHISHU_REST_ENABLE_WRITES` | `0` | 开启固定白名单写接口 |
| `ZHISHU_REST_ROBOT_MODE` | `simulation` | `simulation` 或 `physical` |
| `ZHISHU_REST_ALLOW_PHYSICAL` | `0` | 真机写入二次开关 |
| `ZHISHU_REST_MAX_WRITES_PER_MINUTE` | `10` | 写入限流，`0` 表示关闭限流 |
| `ZHISHU_REST_MAX_JOINT_STEP_RAD` | `0.35` | 真机关节单次最大变化量 |
| `ZHISHU_REST_MAX_REQUEST_BYTES` | `65536` | 最大 HTTP 请求体，范围 1 KiB–1 MiB |
| `ZHISHU_REST_CORS_ORIGIN` | 空 | 允许的精确 Origin；拒绝 `*` |
| `ZHISHU_REST_WORKSPACE` | 自动推断 | 工程根目录 |
| `ZHISHU_ROBOT_LOG_DIR` | `<workspace>/zhishu_robot_logs` | 机器人日志目录 |
