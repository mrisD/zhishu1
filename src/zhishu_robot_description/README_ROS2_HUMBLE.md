# zhishu_robot_description - ROS 2 Humble 启动版

## 编译

把 `zhishu_robot_description` 放到 ROS 2 工作空间的 `src` 目录后执行：

```bash
cd ~/your_ws
colcon build --packages-select zhishu_robot_description
source install/setup.bash
```

## RViz2 查看 URDF

```bash
ros2 launch zhishu_robot_description display.launch.py
```

无 GUI 滑块版本：

```bash
ros2 launch zhishu_robot_description display.launch.py use_gui:=false
```

## Gazebo Classic 启动

```bash
ros2 launch zhishu_robot_description gazebo.launch.py
```

> 注意：此包目前是 URDF 模型显示/仿真生成版本，不包含 ros2_control 硬件插件和控制器配置。


## DH kinematics version

This package now includes a DH-oriented URDF for the left and right 7-DOF arms:

```bash
ros2 launch zhishu_robot_description display_dh.launch.py
```

DH-oriented model file:

```text
urdf/zhishu_robot_description_dh.urdf
```

In this DH version, every left/right arm revolute joint uses:

```xml
<axis xyz="0 0 1"/>
```

The original CAD meshes are preserved by moving each mesh/inertial origin into the new DH link frame. The generated MDH-style parameters and fixed base transforms are saved in:

```text
config/arm_dh_mdh_parameters.csv
config/arm_dh_base_transforms.csv
```

For kinematics, use `*_arm_dh_base_link` as each arm's fixed DH base, then apply the 7 revolute joints in order:

Left arm:

```text
left_shoulder_pitch_joint
left_shoulder_roll_joint
left_shoulder_yaw_joint
left_elbow_pitch_joint
left_wrist_yaw_joint
left_wrist_pitch_joint
left_wrist_roll_joint
```

Right arm:

```text
right_shoulder_pitch_joint
right_shoulder_roll_joint
right_shoulder_yaw_joint
right_elbow_pitch_joint
right_wrist_yaw_joint
right_wrist_pitch_joint
right_wrist_roll_joint
```

The original URDF is still kept as:

```text
urdf/zhishu_robot_description.urdf
```

### DH forward-kinematics helper

A small NumPy-based forward-kinematics checker is included:

```bash
python3 scripts/dh_forward_kinematics.py --side left --q 0 0 0 0 0 0 0
python3 scripts/dh_forward_kinematics.py --side right --q 0 0 0 0 0 0 0
```

The convention is modified DH / MDH:

```text
T_i = Rx(alpha_i) * Tx(a_i) * Rz(theta_offset_i + q_i) * Tz(d_i)
```

The full chain used by the script is:

```text
T_waist_to_hand = T_waist_to_arm_dh_base * T_1 * ... * T_7 * T_wrist_roll_to_hand
```

The wrist-to-hand fixed tool transforms are saved in:

```text
config/arm_dh_tool_transforms.csv
```
