#include <chrono>
#include <exception>
#include <functional>
#include <iostream>
#include <string>
#include <thread>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "behaviortree_cpp/bt_factory.h"

using namespace std::chrono_literals;

namespace
{

BT::NodeStatus runSimulatedStep(
  const std::string & message,
  const std::chrono::milliseconds delay)
{
  std::cout << message << std::endl;
  std::this_thread::sleep_for(delay);
  return BT::NodeStatus::SUCCESS;
}

void registerSimulatedAction(
  BT::BehaviorTreeFactory & factory,
  const std::string & node_id,
  const std::string & message,
  const std::chrono::milliseconds delay = 500ms)
{
  factory.registerSimpleAction(
    node_id,
    std::bind(runSimulatedStep, message, delay));
}

}  // namespace

int main(int argc, char * argv[])
{
  try {
    BT::BehaviorTreeFactory factory;

    // Perception and first navigation stage.
    registerSimulatedAction(
      factory, "DetectBox",
      "[01][相机] 检测物料箱并得到箱子坐标");
    registerSimulatedAction(
      factory, "NavigateToPickPosition",
      "[02][Nav2] 移动底盘导航到箱子抓取位置");

    // The DualArmPickMTC subtree simulates the stages that will eventually be
    // implemented by zhishu_robot_mtc_pick_place.
    registerSimulatedAction(
      factory, "MTCPrepareDualArms",
      "[03][Dual Arm MTC][抓取] 双臂运动到预抓取姿态");
    registerSimulatedAction(
      factory, "MTCOpenGrippers",
      "[04][Dual Arm MTC][抓取] 打开左右夹爪");
    registerSimulatedAction(
      factory, "MTCApproachBox",
      "[05][Dual Arm MTC][抓取] 双臂同步接近箱体");
    registerSimulatedAction(
      factory, "MTCCloseGrippers",
      "[06][Dual Arm MTC][抓取] 关闭左右夹爪并夹紧箱体");
    registerSimulatedAction(
      factory, "MTCAttachBox",
      "[07][Dual Arm MTC][抓取] 将箱体 attach 到机械臂");
    registerSimulatedAction(
      factory, "MTCLiftBox",
      "[08][Dual Arm MTC][抓取] 双臂同步抬起箱体并保持搬运姿态");

    // Navigation while both arms keep the carrying posture.
    registerSimulatedAction(
      factory, "NavigateToPlacePosition",
      "[09][Nav2] 底盘保持双臂搬运姿态，导航到箱子放置位置");

    // The DualArmPlaceMTC subtree simulates the placement MTC stages.
    registerSimulatedAction(
      factory, "MTCLowerBox",
      "[10][Dual Arm MTC][放置] 双臂同步下降箱体");
    registerSimulatedAction(
      factory, "MTCOpenGrippersForRelease",
      "[11][Dual Arm MTC][放置] 打开左右夹爪并释放箱体");
    registerSimulatedAction(
      factory, "MTCDetachBox",
      "[12][Dual Arm MTC][放置] 将箱体从机械臂 detach");
    registerSimulatedAction(
      factory, "MTCRetreatDualArms",
      "[13][Dual Arm MTC][放置] 双臂撤离箱体");
    registerSimulatedAction(
      factory, "MTCReturnSafePose",
      "[14][Dual Arm MTC][放置] 双臂返回安全姿态");

    // Final navigation stage.
    // Demonstrate the RetryUntilSuccessful decorator: the first two calls
    // intentionally fail and the third call succeeds.
    int standby_attempt = 0;
    factory.registerSimpleAction(
      "NavigateToStandby",
      [&standby_attempt](BT::TreeNode &) {
        ++standby_attempt;
        std::cout << "[15][Nav2] 移动底盘返回待命位置，第 "
                  << standby_attempt << "/3 次尝试" << std::endl;
        std::this_thread::sleep_for(500ms);
        if (standby_attempt < 3) {
          std::cout << "    返回待命位置失败，行为树准备重试" << std::endl;
          return BT::NodeStatus::FAILURE;
        }
        std::cout << "    返回待命位置成功" << std::endl;
        return BT::NodeStatus::SUCCESS;
      });
    registerSimulatedAction(
      factory, "ReportTaskCompleted",
      "[16][行为树] 搬箱任务完成");

    std::string xml_path;
    if (argc > 1) {
      xml_path = argv[1];
    } else {
      xml_path =
        ament_index_cpp::get_package_share_directory("zhishu_robot_box_bt") +
        "/config/bt_tree.xml";
    }

    std::cout << "========================================" << std::endl;
    std::cout << "轮式双臂机器人搬箱行为树（字符串模拟模式）" << std::endl;
    std::cout << "行为树文件: " << xml_path << std::endl;
    std::cout << "说明: 当前不会调用真实相机、Nav2、MTC或控制器" << std::endl;
    std::cout << "========================================" << std::endl;

    auto tree = factory.createTreeFromFile(xml_path);
    const BT::NodeStatus result = tree.tickOnce();

    std::cout << "========================================" << std::endl;
    if (result == BT::NodeStatus::SUCCESS) {
      std::cout << "模拟行为树执行成功" << std::endl;
      return 0;
    }

    std::cerr << "模拟行为树未成功完成，最终状态: " << BT::toStr(result) << std::endl;
    return 1;
  } catch (const std::exception & error) {
    std::cerr << "行为树运行异常: " << error.what() << std::endl;
    return 1;
  }
}
