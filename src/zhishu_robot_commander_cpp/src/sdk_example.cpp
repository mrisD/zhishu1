#include "zhishu_robot_commander_cpp/sdk.hpp"

#include <rclcpp/rclcpp.hpp>

#include <chrono>
#include <cstdint>
#include <memory>
#include <string>
#include <thread>

namespace
{
template<typename ParameterT>
ParameterT getOrDeclareParameter(
  const rclcpp::Node::SharedPtr & node,
  const std::string & name,
  const ParameterT & default_value)
{
  if (node->has_parameter(name)) {
    return node->get_parameter(name).get_value<ParameterT>();
  }
  return node->declare_parameter<ParameterT>(name, default_value);
}

zhishu_robot_commander_cpp::Commander::Options commanderOptions(
  const rclcpp::Node::SharedPtr & node)
{
  zhishu_robot_commander_cpp::Commander::Options options;
  options.left_end_effector_link = getOrDeclareParameter<std::string>(
    node, "commander.left_end_effector_link", "L_hand_link");
  options.right_end_effector_link = getOrDeclareParameter<std::string>(
    node, "commander.right_end_effector_link", "R_hand_link");
  options.left_tcp_link = getOrDeclareParameter<std::string>(
    node, "commander.left_tcp_link", "left_gripper_tcp");
  options.right_tcp_link = getOrDeclareParameter<std::string>(
    node, "commander.right_tcp_link", "right_gripper_tcp");
  options.velocity_scaling = getOrDeclareParameter<double>(
    node, "commander.velocity_scaling", 0.30);
  options.acceleration_scaling = getOrDeclareParameter<double>(
    node, "commander.acceleration_scaling", 0.30);
  options.planning_time = getOrDeclareParameter<double>(
    node, "commander.planning_time", 5.0);
  options.planning_attempts = static_cast<int>(getOrDeclareParameter<std::int64_t>(
      node, "commander.planning_attempts", 5));
  options.cartesian_eef_step = getOrDeclareParameter<double>(
    node, "commander.cartesian_eef_step", 0.005);
  options.minimum_cartesian_fraction = getOrDeclareParameter<double>(
    node, "commander.minimum_cartesian_fraction", 0.95);
  return options;
}
}  // namespace

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
    const bool execute_motion = getOrDeclareParameter<bool>(node, "execute_motion", false);
    zhishu_robot_commander_cpp::RobotSdk robot(node, commanderOptions(node));
    if (!execute_motion) {
      RCLCPP_WARN(
        node->get_logger(),
        "SDK initialized in no-motion mode; set execute_motion:=true only after a safety check");
      const auto state = robot.readState();
      if (!state.result) {
        RCLCPP_WARN(
          node->get_logger(), "Robot state is not ready [%s]: %s",
          zhishu_robot_commander_cpp::toString(state.result.code), state.result.message.c_str());
      } else {
        RCLCPP_INFO(node->get_logger(), "SDK state read succeeded; no motion was sent");
      }
    } else {
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
