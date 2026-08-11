# zhishu_robot_box_bt

ROS 2 Jazzy + BehaviorTree.CPP 4 搬箱任务字符串模拟功能包。

当前版本只打印模拟步骤，不连接真实相机、Nav2、Dual Arm MTC、MoveIt 或
ros2_control。行为树结构已经按照未来真实系统的模块边界划分，之后可以逐个
替换叶子节点实现。

## 模拟流程

1. 相机检测箱子并得到箱子坐标；
2. Nav2 导航到抓取位置；
3. `DualArmPickMTC` 子树模拟预抓取、开夹爪、接近、闭夹爪、attach 和抬起；
4. Nav2 保持双臂搬运姿态，导航到放置位置；
5. `DualArmPlaceMTC` 子树模拟下降、开夹爪、detach、撤离和返回安全姿态；
6. Nav2 返回待命位置；如果失败，最多尝试3次。

返回待命节点的 XML：

```xml
<RetryUntilSuccessful num_attempts="3">
  <NavigateToStandby name="navigate_to_standby"/>
</RetryUntilSuccessful>
```

`num_attempts="3"` 表示总共最多执行3次，并不是首次失败后再额外重试3次。
任意一次返回 `SUCCESS` 都会立即停止重试；连续3次返回 `FAILURE` 后，整个
行为树返回 `FAILURE`，后面的任务完成节点不会执行。

为了验证重试逻辑，当前字符串模拟程序故意让返回待命位置的前2次调用返回
`FAILURE`，第3次返回 `SUCCESS`。这只是测试行为，不代表真实底盘故障。

## 安装依赖

```bash
sudo apt update
sudo apt install ros-jazzy-behaviortree-cpp
```

## 编译

```bash
cd ~/文档/test_ws
source /opt/ros/jazzy/setup.bash

rm -rf build/zhishu_robot_box_bt
rm -rf install/zhishu_robot_box_bt

colcon build \
  --packages-select zhishu_robot_box_bt \
  --symlink-install \
  --cmake-args \
  -Dbehaviortree_cpp_DIR=/opt/ros/jazzy/share/behaviortree_cpp/cmake
```

## 运行

```bash
source /opt/ros/jazzy/setup.bash
source ~/文档/test_ws/install/setup.bash
ros2 run zhishu_robot_box_bt box_transport_bt
```

## 预期输出

```text
轮式双臂机器人搬箱行为树（字符串模拟模式）
[01][相机] 检测物料箱并得到箱子坐标
[02][Nav2] 移动底盘导航到箱子抓取位置
[03][Dual Arm MTC][抓取] 双臂运动到预抓取姿态
[04][Dual Arm MTC][抓取] 打开左右夹爪
[05][Dual Arm MTC][抓取] 双臂同步接近箱体
[06][Dual Arm MTC][抓取] 关闭左右夹爪并夹紧箱体
[07][Dual Arm MTC][抓取] 将箱体 attach 到机械臂
[08][Dual Arm MTC][抓取] 双臂同步抬起箱体并保持搬运姿态
[09][Nav2] 底盘保持双臂搬运姿态，导航到箱子放置位置
[10][Dual Arm MTC][放置] 双臂同步下降箱体
[11][Dual Arm MTC][放置] 打开左右夹爪并释放箱体
[12][Dual Arm MTC][放置] 将箱体从机械臂 detach
[13][Dual Arm MTC][放置] 双臂撤离箱体
[14][Dual Arm MTC][放置] 双臂返回安全姿态
[15][Nav2] 移动底盘返回待命位置，第 1/3 次尝试
    返回待命位置失败，行为树准备重试
[15][Nav2] 移动底盘返回待命位置，第 2/3 次尝试
    返回待命位置失败，行为树准备重试
[15][Nav2] 移动底盘返回待命位置，第 3/3 次尝试
    返回待命位置成功
[16][行为树] 搬箱任务完成
模拟行为树执行成功
```

## CMake说明

本包统一使用 plain `target_link_libraries()`，不会再与 Jazzy 的
`ament_target_dependencies()` 产生签名冲突。
