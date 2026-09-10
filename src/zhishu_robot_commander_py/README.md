# zhishu_robot_commander_py

面向客户应用的 Python Commander SDK。Python 包通过 ROS 2 调用已经运行的
`zhishu_robot_commander_cpp` bridge，不重复加载 MoveIt，也不会在构造对象时发送运动命令。

## 编译

```bash
cd ~/online/guohua_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-up-to zhishu_robot_commander_py
source install/setup.bash
```

## 启动

可以让主 bringup 可选启动 Commander：

```bash
ros2 launch zhishu_robot_bringup zhishu_robot.moveit.launch.py \
  use_fake_hardware:=true enable_commander:=true
```

也可以在 MoveIt 已运行后单独启动：

```bash
ros2 launch zhishu_robot_commander_cpp commander.launch.py use_fake_hardware:=true
```

## 示例

```python
from zhishu_robot_commander_py import ArmGroup, Commander, GripperGroup, PoseTarget

with Commander(default_timeout=60.0) as robot:
    if not robot.wait_until_ready(10.0):
        raise RuntimeError("Commander 未启动")

    robot.move_named(ArmGroup.BOTH, "both_arms_home").raise_for_error()

    left_tcp = PoseTarget(
        x=0.35, y=0.30, z=0.75,
        qx=0.0, qy=0.0, qz=0.0, qw=1.0,
        frame_id="base_link",
    )
    robot.move_tcp_pose(ArmGroup.LEFT, left_tcp).raise_for_error()
    robot.open_gripper(GripperGroup.BOTH).raise_for_error()
```

## API

- `move_named()`：命名姿态。
- `move_joints()`：单臂 7 轴 / 双臂 14 轴，单位 rad。
- `move_pose()` / `move_both_poses()`：Hand link 位姿。
- `move_tcp_pose()` / `move_both_tcp_poses()`：真实夹爪 TCP 位姿。
- `move_gripper()` / `move_both_grippers()`：当前 guohua_ws 夹爪位置范围 `[0.0, 0.06]` m。
- `open_gripper()` / `close_gripper()`。
- `stop()`：请求 MoveIt 软件停止。

结果使用 `zhishu_robot_interfaces/msg/CommanderStatus`，监听
`/zhishu_robot_commander/status_structured`。旧的 `/zhishu_robot_commander/status`
仍是 String，保持现有程序兼容。

当前 Topic 协议没有 request ID，因此不要让多个 Python Commander client 并发向同一个
bridge 发送运动命令。客户端本地等待超时不代表机器人已经物理停止；必要时调用 `stop()`，
实体急停/STO 仍是最终安全措施。
