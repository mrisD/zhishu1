# 360 mm 长指落底后松爪与 8 倍速录像（2026-10-07）

## 条件与参数

最新仿真验证方案；首个外围箱 `r0_c0`，300×400×148 mm、15 kg。
初始底盘边沿与托盘边沿名义间距 200 mm，首箱位于托盘近侧边沿。
小箱缺口补平，右夹爪无附加横块。大箱位于 0.55 m 桌面，
内底高度 0.562 m，小箱平放后的箱心高度 0.636 m。

- 手指实体长度 360 mm，TCP 固定偏移 300 mm；指尖超过 TCP 80 mm。
- 腰部下限 −45°；预抽出和双臂抓取目标 −0.785 rad，升降 40 mm。
- 携带升降 250 mm、弯腰 −0.10 rad、箱心目标 0.96 m。
- 桌边升降 210 mm、弯腰 −0.20 rad、箱心目标 0.94 m。
- 底盘最大线/角速度 0.08 m/s、0.12 rad/s；加速度 0.06 m/s²、0.06 rad/s²。
- 落点 `(0.98, -1.55)` m；`place_on_floor_before_release:=true`。

## 与上一版悬空释放的区别

域 179 的历史成功方案在内底上方约 50 mm 开爪，不能作为本轮
“落底后松爪”的证据。新方案保持闭爪入箱至内底上方 20 mm，随后
每次下降 2 mm，检查 Gazebo 第一箱底部与大箱底部的真实接触。

开爪前要求接触数据新鲜、反力至少 10 N、箱心距 0.636 m 不超过
1.5 mm、倾角不超过 0.5°，连续满足 750 ms；双爪关节仍大于 0.04 m。
采样反力超过 300 N、位姿过期、偏离落点或倾角超限时停止，不开爪。
这不是连续力控：反力保护在下降步之间检查，不能替代实机安全力控。

轻微倾斜时，只有已检测到箱底承重至少 80 N，才允许将夹爪闭合指令
从 0.057 m 减至墙面接触位置 0.055 m，减小预紧让箱子在支撑面上放平。
完全开爪目标为 0，必须在稳定落底确认后执行。最终域 182 的连续测试
直接满足落底条件，没有调用这个减预紧分支；该分支仅在域 181 诊断过。

MoveIt 使用独立 `large_bin_floor` 碰撞对象，只允许小箱与其预期接触；
大箱壁、桌面和夹爪碰撞仍检查。箱体质量和物理尺寸没有减少，
没有焊接、附着插件或瞬移箱体。MoveIt 附着只用于规划。

## 入箱前的修正与失败记录

域 181 初次完整录像在桌边接近时因右腕安全余量不足而停止，未开爪。
追查到原腰部到位容差允许尚差约 1.2° 就启动抓取规划，导致实际浅夹取
与预期偏离。新容差为升降误差小于 1 mm、弯腰误差小于 0.002 rad。
携带后读取实际关节 FK 与实际箱心，保持测得的 TCP 相对高度，
避免在后续姿态转换中重新改变夹取深度。桌边箱心目标从 0.96 m
降到 0.94 m，保留原运输高度和腕部安全余量。

域 181 诊断续测确认倾斜箱底先接触后减预紧可放平，随后确认落底、
开爪和竖直撤爪通过。诊断日志保存在
`artifacts/pallet_bin_video_20261007_floor360_8x/`，不得拼成完整成功录像。

## 域 182 连续实测

从独立初始场景开始，无 Gazebo/RViz 界面。录制实际 Gazebo 相机图像，
按阶段自动使用抓取、全景和桌边三个视角，不使用示意动画。

实测初始双臂夹取深度约 46.0–46.1 mm，持箱 TCP 相对高度偏移
107.994 mm。实际最大弯腰约 −44.9772°；运输段采样最大箱体倾角
约 0.0968°，截至落箱及回位阶段采样最大约 0.5525°。

开爪前日志确认箱心 0.636161 m、倾角约 0.057°、传感器反力约
270.8 N，双爪仍闭合。完全开爪后箱心 0.6359997 m，仅约 0.16 mm
沉降，不是先前约 50 mm 悬空下落。最终箱心
`(0.9798105, -1.5499996, 0.6359997)` m，倾角约 0.000097°。
反力来自仿真接触消息，不是标定后的实机腕部力传感器测量。

域 182 从独立初始场景连续完成预抽出、重新定位、双臂夹取、
抬升与站起、底盘绕行、入箱、落底确认、松爪、竖直撤离、双臂回位。
日志含 `return both arms home: execution completed`、
`Complete transfer finished`，搬运节点正常退出。
最终腰部保持已检查的 210 mm / −0.20 rad 停放姿态，不盲目归零。

证据目录：`artifacts/pallet_bin_video_20261007_floor360_8x_final/`。
原速 `full_process.webm` 为 32 分 11 秒；8 倍速连续录像为 4 分 1.4 秒，
2414 帧、960×620、10 fps，无剪辑和拼接。
交付 MP4 为 `pallet_bin_floor_seated_8x.mp4`；原速 WebM 和实测数据一并保留。
`recording_summary.json` 中 `passed=true`、`speedup=8`，
最终遥测 `floor_verified_before_open=true`。目录还包含完整日志、
`telemetry.jsonl`、`fk_audit.jsonl`、阶段截图及实际相机 world。

MP4 全部 2414 帧解码检查通过，首帧、运输、落底、松爪和末帧
截图已保存；文件 9,903,784 字节，H.264 Constrained Baseline / Level 3.1 /
YUV420P，`moov` 位于 `mdat` 前，适合手机播放。
录像结束并关闭场景时，MoveGroup 在收到 SIGINT 后的析构阶段报段错误；
这发生在搬运节点正常退出、录像封装完成之后，不是搬运动作中失败。
本轮创建的仿真及采样进程随后均停止，完整停场日志一并保留。

## 复现

加载 ROS Jazzy 与工作空间环境，使用独立的 ROS 域和 Gazebo partition：

```bash
ros2 launch zhishu_robot_mtc_pick_place pallet_bin_15kg_transfer.launch.py \
  headless:=true use_rviz:=false execute:=true pre_extract_enabled:=true \
  validate_all_nine:=false full_transfer_box_index:=0 \
  waist_pitch_min:=-0.785398163397 tote_finger_length:=0.360 \
  pre_extract_waist_pitch:=-0.785 grasp_waist_pitch:=-0.785 \
  carry_waist_lift:=0.25 place_waist_lift:=0.21 place_center_height:=0.94 \
  base_max_linear:=0.08 base_max_angular:=0.12 \
  base_linear_acceleration:=0.06 base_angular_acceleration:=0.06 \
  place_on_floor_before_release:=true
```

录像相机由 `scripts/record_pallet_bin_transfer.py world` 生成，录制时传入
`--speedup 8 --finger-length-mm 360 --tcp-offset-mm 300 --waist-limit-deg -45 --floor-seated`。
原速和 8 倍速均连续保存，不剪辑、不拼接，8 倍速按相同播放帧率
每 8 帧取 1 帧。最终 MP4 使用 H.264 Baseline、YUV420P 和 faststart。

本轮仅验证首箱，不代表全部九箱；重新定位使用仿真位姿，不是真实视觉。
位置轨迹控制与接触确认不等于力控导纳。长指刚度、摩擦参数、
支撑球代理及仿真腰部力矩仍需实机核对，本轮不构成实机安全认证。
