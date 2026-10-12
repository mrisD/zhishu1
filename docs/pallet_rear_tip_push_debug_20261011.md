# 中排后方指尖推送：2026-10-11 调试记录

后续更新：2026-10-12 已修复控制插件 mimic 空数组读取，并完成完整连续搬运验证及成功录像重跑；见 `gazebo_startup_crash_fix_20261012.md` 和 `pallet_rear_push_optimization_20261012.md`。以下保留 2026-10-11 的失败诊断，不代表最新状态。

当时状态：**未通过完整物理搬运验证，未生成新方案成功视频，不可当作已验证参数。**

## 目标与保持项

- r0/r2 清空，仅保留中排；先验证 r1_c2。
- 从后方推送 324 mm，再绕行到正面双臂搬入 0.55 m 桌面大箱。
- 手指实体长度 360 mm；双臂抓取 TCP 为夹爪基座起 300 mm。
- 新增独立指尖参考系：夹爪基座起 380 mm（含 20 mm 安装偏移）。
- 腰部下限 −90°；后方名义站位 x=2.068 m、yaw=π，对应名义底盘/托盘边沿间距 200 mm。尚未验证全流程实际间距。
- 无界面测试，无推箱附件绑定或箱体瞬移；保留碰撞检查。

## 本轮结果

1. bullet4 起始状态被相机与左肘碰撞阻断。
2. 左肩 roll 改为既有外展停放角 −0.261799 rad 后，bullet8 起始检查不再被该碰撞阻断。
3. bullet11 的插值采样确认：第 45 步切换关节分支时，右夹爪扫过头部、腰部。目标点检查不能替代完整路径检查。
4. 增加相邻路点关节变化限制后，bullet14/15 求解到第 46 步；第 47 步无法找到满足约束的连续姿态，未执行推箱。
5. 最新未完成验证的候选：先闭合夹爪、取消最后悬停下降、从接触目标反向求解接近路点。已编译通过，但后续测试受启动故障阻断。
6. 多轮运行时生成模型测试出现控制插件未就绪；暂停单步初始化未稳定解决。
7. 预加载机器人诊断中，Bullet 与 DART 均在 `gz_ros2_control::GazeboSimSystem::write()` 崩溃。预加载方式不能作为成功替代。

## 证据目录

均相对工作区根目录 `output/`：

- `pallet_middle_rear_push_20261009_lattice_rear_tip_20261011_bullet11/probe.log`：插值碰撞部件和深度。
- `pallet_middle_rear_push_20261009_lattice_rear_tip_20261011_bullet15/probe.log`：连续性约束后的第 47 步失败。
- `pallet_middle_rear_push_20261009_lattice_rear_tip_20261011_bullet_trace2/scene.log`：运行时生成模型未就绪时的调用栈。
- `pallet_middle_rear_push_20261009_lattice_rear_tip_20261011_preload1/scene.log`：Bullet 控制插件 write 崩溃。
- `pallet_middle_rear_push_20261009_lattice_rear_tip_20261011_preload2_dart/scene.log`：DART 控制插件 write 崩溃。

下一步先修复/定位控制插件初始化与 write 崩溃，再验证反向求解候选、真实推送位移、绕行、双臂搬运和落底后松爪。现有旧方案成功视频不能替代本方案验证。
