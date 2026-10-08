# 默认箱位、200 mm工位与360 mm长指搬运验证（2026-10-08）

首箱 r0_c0 恢复默认中心 (0.704355,-0.400617,0.224) m；取箱工位底盘 x=0，车沿到托盘边沿名义间距200 mm。先沿负Y预抽出120 mm，再双臂搬入大箱。腰部允许下限−90°，实际抓取约−63.03°，手指360 mm、TCP偏移300 mm。

独立弯腰曾使长指碰箱并抬起底盘；改为预抽出后保留腰部姿态，双臂先抬高，再8段腰臂同步过渡保持高位TCP。新接近路径用MTC检查碰撞，未为新路径增加碰撞豁免；既有抓取任务的历史接触允许仍存在。

较快底盘运输曾使箱体入箱倾角约5.8°超过5°落底检查而停止。最终采用0.08 m/s、0.12 rad/s、线角加速度0.06，保留原落底接触、闭爪和倾角检查。脚本在原速及4倍速画面加入实际关节弯腰角度。

成功证据：artifacts/pallet_gap200_angle_slow_20261008_174635/。recording_summary.json中passed=true、floor_verified_before_open=true；日志包含Complete transfer finished。原速1778.4秒，4倍约444.6秒，落底后松爪与双臂回位完成。仅首箱实测，不代表三箱连续搬运已通过。视频/日志不纳入Git。

复现（先source ROS Jazzy与install/setup.bash，使用独立ROS域及Gazebo partition）：

```bash
ros2 launch zhishu_robot_mtc_pick_place pallet_bin_15kg_transfer.launch.py \
  headless:=true use_rviz:=false execute:=true pre_extract_enabled:=true \
  validate_all_nine:=false full_transfer_box_index:=0 \
  waist_pitch_min:=-1.57079632679 tote_finger_length:=0.360 \
  pre_extract_waist_pitch:=-0.785 grasp_waist_pitch:=-1.10 \
  carry_waist_lift:=0.25 place_waist_lift:=0.21 place_center_height:=0.94 \
  base_max_linear:=0.08 base_max_angular:=0.12 \
  base_linear_acceleration:=0.06 base_angular_acceleration:=0.06 \
  place_on_floor_before_release:=true base_motion_timeout:=600.0 \
  velocity_scaling:=0.25 acceleration_scaling:=0.15 \
  source_grasp_x:=0.0 pre_extract_distance:=0.12
```

录像world由scripts/record_pallet_bin_transfer.py world生成，再将world_file指定为该world。录制参数：--speedup 4 --finger-length-mm 360 --tcp-offset-mm 300 --waist-limit-deg -90 --floor-seated。

launch支持速度、加速度、底盘超时以及目标模型/已清除首排箱数参数。多箱扩展仍需为对应模型配置位姿和接触传感器；本轮没有完成三箱验收。
