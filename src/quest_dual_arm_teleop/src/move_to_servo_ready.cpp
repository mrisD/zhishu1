#include <chrono>
#include <memory>
#include <thread>

#include "moveit/move_group_interface/move_group_interface.h"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  rclcpp::NodeOptions options;
  options.automatically_declare_parameters_from_overrides(true);
  auto node = std::make_shared<rclcpp::Node>("move_to_servo_ready", options);

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  std::thread spin_thread([&executor]() { executor.spin(); });

  int result = 1;
  try {
    moveit::planning_interface::MoveGroupInterface move_group(node, "both_arms");
    move_group.setStartStateToCurrentState();
    move_group.setNamedTarget("both_arms_servo_ready");
    move_group.setPlanningTime(8.0);
    move_group.setNumPlanningAttempts(10);
    move_group.setMaxVelocityScalingFactor(0.08);
    move_group.setMaxAccelerationScalingFactor(0.08);

    RCLCPP_INFO(
      node->get_logger(),
      "Planning both arms to 'both_arms_servo_ready' at 8%% velocity/acceleration scaling...");

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    const bool planned =
      (move_group.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);

    if (!planned) {
      RCLCPP_ERROR(node->get_logger(), "Failed to plan to both_arms_servo_ready; nothing executed.");
    } else {
      RCLCPP_INFO(node->get_logger(), "Plan succeeded. Executing...");
      const auto exec_result = move_group.execute(plan);
      if (exec_result == moveit::core::MoveItErrorCode::SUCCESS) {
        RCLCPP_INFO(node->get_logger(), "Reached both_arms_servo_ready successfully.");
        result = 0;
      } else {
        RCLCPP_ERROR(node->get_logger(), "Execution of both_arms_servo_ready failed.");
      }
    }
  } catch (const std::exception& ex) {
    RCLCPP_ERROR(node->get_logger(), "move_to_servo_ready exception: %s", ex.what());
  }

  executor.cancel();
  if (spin_thread.joinable()) {
    spin_thread.join();
  }
  rclcpp::shutdown();
  return result;
}
