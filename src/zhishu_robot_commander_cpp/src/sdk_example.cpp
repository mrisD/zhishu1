#include "zhishu_robot_commander_cpp/sdk.hpp"

#include <rclcpp/rclcpp.hpp>

#include <chrono>
#include <memory>
#include <thread>

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>(
    "zhishu_robot_commander_sdk_example",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  std::thread spinner([&executor]() {executor.spin();});

  try {
    bool execute_motion = false;
    if (!node->get_parameter("execute_motion", execute_motion)) {
      execute_motion = node->declare_parameter("execute_motion", false);
    }
    if (!execute_motion) {
      RCLCPP_WARN(
        node->get_logger(),
        "SDK example is armed but will not move; set execute_motion:=true after a safety check");
    } else {
      zhishu_robot_commander_cpp::RobotSdk robot(node);

      auto motion = robot.moveNamedAsync(
        zhishu_robot_commander_cpp::ArmGroup::kBoth, "both_arms_home");
      auto result = motion.waitFor(std::chrono::seconds(30));
      if (result.code == zhishu_robot_commander_cpp::ErrorCode::kTimeout) {
        motion.cancel();
        result = motion.wait();
      }
      if (!result) {
        RCLCPP_ERROR(
          node->get_logger(), "Move home failed [%s]: %s",
          zhishu_robot_commander_cpp::toString(result.code), result.message.c_str());
      } else {
        result = robot.openGripper(zhishu_robot_commander_cpp::GripperGroup::kBoth);
        if (!result) {
          RCLCPP_ERROR(
            node->get_logger(), "Open grippers failed [%s]: %s",
            zhishu_robot_commander_cpp::toString(result.code), result.message.c_str());
        }
      }
    }
  } catch (const std::exception & exception) {
    RCLCPP_FATAL(node->get_logger(), "SDK example failed: %s", exception.what());
  }

  executor.cancel();
  spinner.join();
  rclcpp::shutdown();
  return 0;
}
