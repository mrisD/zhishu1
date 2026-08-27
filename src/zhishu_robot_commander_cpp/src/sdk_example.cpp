#include "zhishu_robot_commander_cpp/commander.hpp"

#include <rclcpp/rclcpp.hpp>

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
    zhishu_robot_commander_cpp::Commander commander(node);

    // Example motion: use only with fake hardware or after a real-robot safety check.
    if (!commander.moveBothArmsNamed("both_arms_home")) {
      RCLCPP_ERROR(node->get_logger(), "Failed to move both arms home");
    }
    if (!commander.openBothGrippers()) {
      RCLCPP_ERROR(node->get_logger(), "Failed to open both grippers");
    }
  } catch (const std::exception & exception) {
    RCLCPP_FATAL(node->get_logger(), "SDK example failed: %s", exception.what());
  }

  executor.cancel();
  spinner.join();
  rclcpp::shutdown();
  return 0;
}
