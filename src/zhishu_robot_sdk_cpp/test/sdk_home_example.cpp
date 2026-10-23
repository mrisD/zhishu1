#include "zhishu_robot_sdk/sdk.hpp"

#include <rclcpp/rclcpp.hpp>

#include <chrono>
#include <cstdint>
#include <cstdlib>
#include <exception>
#include <filesystem>
#include <future>
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

zhishu_robot_sdk::Commander::Options commanderOptions(
  const rclcpp::Node::SharedPtr & node)
{
  zhishu_robot_sdk::Commander::Options options;
  options.left_end_effector_link = getOrDeclareParameter<std::string>(
    node, "commander.left_end_effector_link", "L_hand_link");
  options.right_end_effector_link = getOrDeclareParameter<std::string>(
    node, "commander.right_end_effector_link", "R_hand_link");
  options.velocity_scaling = getOrDeclareParameter<double>(
    node, "commander.velocity_scaling", 0.30);
  options.acceleration_scaling = getOrDeclareParameter<double>(
    node, "commander.acceleration_scaling", 0.30);
  options.planning_time = getOrDeclareParameter<double>(
    node, "commander.planning_time", 5.0);
  options.planning_attempts = static_cast<int>(getOrDeclareParameter<std::int64_t>(
      node, "commander.planning_attempts", 5));
  return options;
}

// Pre-flight hint only. It mirrors the RobotSdkOptions::home_pose_file default declared in
// robot_sdk.hpp; the SDK itself resolves the authoritative path.
std::string homePoseFile(const std::string & configured)
{
  if (!configured.empty()) {
    return configured;
  }
  const char * home = std::getenv("HOME");
  if (home == nullptr || *home == '\0') {
    return {};
  }
  return (std::filesystem::path(home) / ".ros" / "zhishu_robot_sdk_cpp" / "home_pose.json")
         .string();
}
}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>(
    "zhishu_robot_sdk_home_example",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  std::thread spinner([&executor]() {executor.spin();});

  int exit_code = 0;
  try {
    const bool execute_motion = getOrDeclareParameter<bool>(node, "execute_motion", false);
    const std::string mode = getOrDeclareParameter<std::string>(node, "mode", "move");
    const std::string home_file = getOrDeclareParameter<std::string>(node, "home_pose_file", "");
    const double timeout_sec = getOrDeclareParameter<double>(node, "timeout_sec", 60.0);

    if (mode != "check" && mode != "save" && mode != "move") {
      RCLCPP_ERROR(node->get_logger(), "mode must be check, save or move; received '%s'",
        mode.c_str());
      executor.cancel();
      spinner.join();
      rclcpp::shutdown();
      return 2;
    }
    if (timeout_sec <= 0.0) {
      RCLCPP_ERROR(node->get_logger(), "timeout_sec must be positive");
      executor.cancel();
      spinner.join();
      rclcpp::shutdown();
      return 2;
    }

    zhishu_robot_sdk::RobotSdkOptions options;
    options.commander = commanderOptions(node);
    options.home_pose_file = home_file;
    zhishu_robot_sdk::RobotSdk robot(node, options);

    const std::string resolved_home = homePoseFile(home_file);
    const bool record_exists =
      !resolved_home.empty() && std::filesystem::exists(resolved_home);
    RCLCPP_INFO(node->get_logger(), "mode=%s execute_motion=%s timeout=%.1fs",
      mode.c_str(), execute_motion ? "true" : "false", timeout_sec);
    RCLCPP_INFO(node->get_logger(), "home record: %s (%s)",
      resolved_home.empty() ? "<unresolved: HOME is not set>" : resolved_home.c_str(),
      record_exists ? "present -> moveHome() replays the saved 14 joints (pose fallback)" :
      "absent -> moveHome() uses the built-in both_arms_home");

    if (mode == "check") {
      const auto state = robot.readState();
      RCLCPP_INFO(node->get_logger(),
        "state: left/right arm joints valid=%s/%s left/right end-effector poses valid=%s/%s",
        state.left_arm_joints_valid ? "true" : "false",
        state.right_arm_joints_valid ? "true" : "false",
        state.left_arm_pose_valid ? "true" : "false",
        state.right_arm_pose_valid ? "true" : "false");
      if (!state.result) {
        RCLCPP_WARN(node->get_logger(), "state is not ready [%s]: %s",
          zhishu_robot_sdk::toString(state.result.code), state.result.message.c_str());
        exit_code = 1;
      }
    } else if (!execute_motion) {
      RCLCPP_WARN(node->get_logger(),
        "No motion sent: mode=%s would call %s. Set execute_motion:=true after a safety check.",
        mode.c_str(), mode == "save" ? "setHomePose()" : "moveHome()");
    } else {
      RCLCPP_WARN(node->get_logger(), "Calling %s; the robot will move or record its pose now",
        mode == "save" ? "setHomePose()" : "moveHome()");
      auto pending = std::async(std::launch::async, [&robot, mode]() {
            return mode == "save" ? robot.setHomePose() : robot.moveHome();
          });
      zhishu_robot_sdk::Result result;
      if (pending.wait_for(std::chrono::seconds(static_cast<long>(timeout_sec))) !=
        std::future_status::ready)
      {
        RCLCPP_WARN(node->get_logger(),
          "Home request did not finish within %.1fs; requesting a MoveIt stop",
          timeout_sec);
        robot.cancelActiveMotion();
        pending.wait();
        result = pending.get();
      } else {
        result = pending.get();
      }
      if (!result) {
        RCLCPP_ERROR(node->get_logger(), "Home request failed [%s]: %s",
          zhishu_robot_sdk::toString(result.code), result.message.c_str());
        exit_code = 1;
      } else {
        RCLCPP_INFO(node->get_logger(), "Home request finished: %s", result.message.c_str());
      }
      if (mode == "save" && result.ok() && !resolved_home.empty()) {
        RCLCPP_INFO(node->get_logger(), "home record written to %s", resolved_home.c_str());
      }
    }
  } catch (const std::exception & exception) {
    RCLCPP_FATAL(node->get_logger(), "Home example failed: %s", exception.what());
    exit_code = 1;
  }

  executor.cancel();
  spinner.join();
  rclcpp::shutdown();
  return exit_code;
}
