# 九箱场景首箱导纳测试（2026-10-16）

仅测试 `pallet_box_r0_c0`，其余八箱保留。Gazebo 服务与 GUI 独立启动。
MTC 负责抽出、抓持及进入水平持箱姿态；`admittance_handoff:=true` 后不执行底盘搬运或放置。
严格停用 `dual_arm_controller` 并启用左右臂控制器后，由双 MoveIt Servo 执行导纳速度。

优化后的导纳段由双臂独立执行，腰部贡献为零。根据 Gazebo 实测箱体位移停止，最后 10 mm 减速；
保留奇异性、限位、碰撞、传感器超时保护。请求 50 mm，距离容差 ±2 mm。

| 指标 | 向上 | 回程 |
|---|---:|---:|
| 实际位移（mm） | 49.723 | 49.760 |
| 最大横向偏差（mm） | 0.090 | 0.086 |
| 停止速度（mm/s） | 1.379 | 1.500 |
| 保持漂移（mm） | 0.001282 | 0.000000 |
| 验收 | 通过 | 通过 |

其余八箱最大位移 0.000575 mm。单元/回归测试 17 项通过。
相关 ROS 包构建通过，世界文件 SDF 校验通过。运动结束后导纳禁用，GUI 保留。

## 复现

使用相同 ROS_DOMAIN_ID、GZ_PARTITION 启动九箱场景和独立 GUI。场景启动参数包含
`admittance_handoff:=true full_transfer_box_index:=0 validate_all_nine:=false keep_all_source_rows:=true`。
然后执行：

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
python3 scripts/test_pallet_first_box_admittance.py --scene-log <场景日志> --output-dir <结果目录>
```

脚本等待首箱物理抬升交接标志，然后加载 Servo、切换控制器并执行向上 50 mm 导纳验收。
具体参数见 `src/zhishu_robot_admittance/README.md`。

## 原始证据与限制

结果位于 `output/admittance_20261016/retry/`：`final_lift_50mm.json`、
`closed_loop_return_50mm.json`、`source_boxes_final.json`、`summary.json`。
初次 GUI/仿真组合实例退出后重启过一次，不声称全过程没有重启。
调参过程中的宽容差腰部测试及开环双臂过冲结果保留，未作为最终导纳通过证据。
本次仅验证首箱竖直导纳搬运，不包含转运到收料箱、放置、外力扰动辨识或真机测试。
