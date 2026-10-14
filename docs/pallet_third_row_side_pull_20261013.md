# 第三排侧拉与重新录制（2026-10-13）

## 任务约束

第一排已搬完；第二排、第三排共六箱保留在同一场景。第三排必须先侧拉再抓取，逐箱搬运，沿用中间排连续腰臂轨迹及速度。完整验证通过前不交付为成功视频。

## 本次实现

- 支撑板 `perimeter_extraction_deck` 从长 0.40 m 扩为 1.25 m，中心 `(1.009355,-0.805,0.075)`，宽/厚仍为 0.41/0.15 m；Gazebo 物理碰撞、可视模型和 MoveIt 世界几何一致，覆盖三排。
- 新增 `--rear-side-pull`：保留 r1/r2 六箱，第三排按 c0 → c1 → c2。从后侧站位朝 -X，采用第一排右臂侧拉的左臂镜像；在机器人局部 +Y 拉 0.12 m，对应世界 -Y。从实测箱位继续双臂夹取，未使用箱体瞬移或 Gazebo attachment。
- 底盘始终在托盘外侧；箱子拉出后，后侧抓取站对准实测 y。空载经北侧返回下一站。
- 底盘速度/加速度 0.12 m/s、0.18 rad/s、0.08 m/s²、0.10 rad/s²；机械臂比例 0.12/0.08；连续轨迹合并、携带高度 0.94 m、放置落底确认流程沿用已有优化。
- 修复倒序搬运误删规划障碍物的问题。第三排未搬的箱子和第二排始终参与规划。
- 恢复夹爪与底盘、腰部、相机和头部的碰撞检查，并恢复本模式抬升阶段的邻箱检查。原有其他模式仍存在历史碰撞豁免，不能据此宣称全机器人所有碰撞均已严格验证。
- 靠站去掉微小位置误差下的目标朝向混合控制；近站先定向、再沿目标方向微调，累计旋转超过一圈则停止。预置站位免去原有多余绕行。

## 执行

```bash
bash scripts/run_with_gz_control_safe_overlay.sh python3 scripts/run_pallet_first_row_sequence.py \
  --output-dir output/pallet_third_row_side_pull_extended_20261013_01 \
  --source-row 2 --rear-side-pull --stop-on-failure
```

已通过本包编译、Python 语法检查和 Git diff 空白检查。当前输出为试录证据，最终结果以 `sequence_result.json`、`recording_summary.json` 和完整视频解码为准。

## 直接抓取试录的发现

此前保留六箱的直接抓取试录，第一箱在抬升时倾角达到约 10°，底盘被带动倾斜，已中止，未作为成功视频。箱间仅约 4 mm，原抓取位置有夹指进入邻箱壁区域的风险。依据用户要求改为先侧拉拉开空间，取消直接抓取方案。

## 驻停坐标修复与侧拉实测

首轮镜像侧拉暴露旧 `runRightArmExtraction` 驻停控制直接使用世界 X/Y 误差作机体速度，后侧朝向 π 时形成正反馈。已与中间排驻停代码一致，使用 `dx*cos(yaw)+dy*sin(yaw)` 控制前后速度；差速底盘不发送横向速度。

第二轮 `output/pallet_third_row_side_pull_extended_20261013_02/` 第一箱侧拉、撤爪已执行完成：世界 y 从 -0.400617 到 -0.520541 m，位移约 119.924 mm，箱高仍约 0.224 m；底盘保持约 `(2.067994,-0.520617)` m。此处仅证明第一箱侧拉通过，完整三箱搬运仍需看最终结果。

## 第一箱完整通过，第二箱规划修复后续录

第二轮第一箱已完成侧拉 → 抓取抬升 → 携带绕行 → 入箱落底 → 松爪撤爪 → 回位 → 空载返回。松爪前实测箱高 0.636801 m、倾角 0.306°、底部反力 260.2 N，松爪后稳定箱高约 0.636000 m。第一箱成功片段已导出并逐帧解码：`output/pallet_third_row_side_pull_delivery_20261013/box_0_verified_8x.mp4`。

第二箱未执行侧拉即被规划拦截（竖直下探 96.875%）。后侧站位横向误差约 17 mm，名义插入点不可达，旧 IK 格点求解选了偏移点，但 Cartesian 阶段仍使用名义直线。修复包括：后侧侧拉站位横向容差 2 mm；该模式仅接受与实际直线路径一致的名义 IK 点和固定朝向，提高单臂重试预算，避免不一致的偏移解。

使用第一箱完成且目标箱已清空的日志检查点继续第二、三箱：

```bash
bash scripts/run_with_gz_control_safe_overlay.sh python3 scripts/run_pallet_first_row_sequence.py \
  --output-dir output/pallet_third_row_side_pull_extended_20261013_03 \
  --source-row 2 --rear-side-pull --stop-on-failure \
  --continue-after-output output/pallet_third_row_side_pull_extended_20261013_02
```

续录场景保留第二排三箱与第三排 c1/c2；c0 仅在已有搬运、空载返回和目标箱清空证据成立时移除。最终录像将标明检查点续录，不能描述为三箱无重启连续仿真。

续录中的第二箱已实际侧拉约 119.94 mm、双臂夹取并协调起身，携带箱高约 0.9423 m。空载下探候选轨迹中曾检测到 `gripper_left_finger_link` 与 `left_gripper_right_finger_link` 相碰，ValidateSolution 拦截后重规划成功；该失败候选未执行。第二箱随后放置、收臂通过，返程末段中断，详见下节。

视频组装脚本：`output/pallet_third_row_side_pull_delivery_20261013/assemble_delivery.py`。检查第一箱完成/空载返回/清箱证据、第二箱物理放置/收臂证据、第三箱完整成功摘要后，组装第三排 8 倍速、原速和九箱合集；续录切换处插入明确说明卡片，生成逐帧解码摘要。阶段 0 实际源字幕已经显示 360 mm，无需覆盖原画面字幕。

## 第二箱放置完成，返程末段中断后继续第三箱

`..._03/box_1.log` 中第二箱松爪前落底数据：箱心 0.636116 m、倾角 0.044°、底部反力 144.6 N。松爪后物理放置验证 `(0.9790,-1.5500,0.6360)`，随后 `return both arms home: execution completed`。返回北侧后，最后靠站处实测位置约 `(2.068275,0.289888)`，目标 `(2.068,0.286220)`；横向残差约 3.67 mm。流程在此中断，无完整成功摘要，不能宣称第二箱空载返程通过。

发现 launch 未声明/转发 `base_position_tolerance` 和 `base_yaw_tolerance`，命令行 2 mm/2 mrad 设置实际被忽略。原节点位置容差 4 mm、该模式横向容差 2 mm，位置锁定可在横向尚不合格时每周期重新锁住。已补 launch 参数，并让锁定同时满足横向容差。已编译通过；此处返程修复尚未完成重新执行验证。

第二箱有效搬运片段截至收臂完成、空载返程开始前（原速约 1573.8 s），导出 `box_1_placement_verified_8x.mp4`，1968 帧逐帧解码通过。返程中断段留在原始 WebM/日志中，不混入成功片段。

第三箱从明确重建的四箱场景续录：中间排三箱 + 第三排 c2。`--continue-after-placement-output` 要求同时提供已验证的第一箱检查点、第二箱侧拉/落底/放置/收臂日志，且第二箱来源引用同一个第一箱检查点。该模式仅重建剩余场景，不冒充已完成第二箱返程或直接恢复物理快照。

```bash
bash scripts/run_with_gz_control_safe_overlay.sh python3 scripts/run_pallet_first_row_sequence.py \
  --output-dir output/pallet_third_row_side_pull_extended_20261013_04 \
  --source-row 2 --rear-side-pull --stop-on-failure \
  --continue-after-output output/pallet_third_row_side_pull_extended_20261013_02 \
  --continue-after-placement-output output/pallet_third_row_side_pull_extended_20261013_03
```

最终组装改为第一箱完整通过片段 → 说明卡 → 第二箱放置/收臂通过片段 → 说明卡（返程中断）→ 第三箱续录。

## 第三箱完成与交付核对

`..._04/box_2.log` 第三箱侧拉实测约 119.936 mm，携带箱心约 0.9425 m；入箱连续轨迹 58.50 → 39.58 s。松爪前落底箱心 0.636533 m、倾角 0.204°、底部反力 249.2 N。松爪后物理放置验证 `(0.9781,-1.5500,0.6360)`；收臂执行完成及 `Complete transfer finished` 均存在。

第三箱录制摘要 `passed=true`，本轮实际录制完成 1 箱，前置已验证放置 2 箱，原速 14648 帧、8 倍速 1831 帧。录制器在视频/摘要写完后的清理阶段 exit=-11，因此必须以交付视频的逐帧解码核对帧数，不能仅看退出码。第三排 8 倍速合成已核对 6394 帧全解码。

本轮只读参数快照 `effective_motion_parameters.json` 已确认 2 mm/2 mrad 靠站设置和优化速度实际生效。运行开始后修正过 Python 汇总中的 `removed_after_return` 字段，因此本轮原始 `sequence_result.json` 的旧字段仍包含 c1；该箱未通过空载返回。原文件保留，交付 `validation_result.json` 按逐箱日志明确记录这一限制。

最终交付目录 `output/pallet_third_row_side_pull_delivery_20261013/`：第三排 8 倍速 6394 帧/639.4 s，原速 50586 帧/5058.6 s；九箱第一排→第三排→中间排合集 22514 帧/2251.4 s。三个文件全部逐帧解码通过，第三箱源视频原速 14648 帧与 8 倍速 1831 帧全部保留。两次第三排续录和中间排原有续录均在画面中标明，交付 README、视频章节 JSON 及逐箱验证报告一并保存。
