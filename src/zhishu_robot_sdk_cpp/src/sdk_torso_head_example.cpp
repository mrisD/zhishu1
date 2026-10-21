#include "zhishu_robot_sdk/sdk.hpp"

#include <rclcpp/rclcpp.hpp>

#include <iomanip>
#include <memory>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

namespace
{
template<typename ParameterT>
ParameterT getOrDeclareParameter(const rclcpp::Node::SharedPtr & node, const std::string & name,
  const ParameterT & default_value)
{
  if (node->has_parameter(name)) {
    return node->get_parameter(name).get_value<ParameterT>();
  }
  return node->declare_parameter<ParameterT>(name, default_value);
}

bool logResult(
  const rclcpp::Logger & logger, const std::string & operation,
  const zhishu_robot_sdk::Result & result)
{
  if (result) {
    RCLCPP_INFO(logger, "%s succeeded", operation.c_str());
  } else {
    RCLCPP_ERROR(
      logger, "%s failed [%s]: %s", operation.c_str(),
      zhishu_robot_sdk::toString(result.code), result.message.c_str());
  }
  return static_cast<bool>(result);
}
}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>(
    "zhishu_robot_sdk_commander_torso_head_example",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  std::thread spinner([&executor]() {executor.spin();});

  int exit_code = 0;
  try {
    const bool execute_motion = getOrDeclareParameter<bool>(node, "execute_motion", false);
    const std::string motion = getOrDeclareParameter<std::string>(node, "motion", "none");
    const auto joints = getOrDeclareParameter<std::vector<double>>(node, "joints", {});
    const std::string named_target = getOrDeclareParameter<std::string>(node, "named_target", "");

    zhishu_robot_sdk::RobotSdk robot(node);
    const auto torso_head_joints = robot.getTorsoHeadJoints();
    std::ostringstream values;
    values << std::fixed << std::setprecision(4);
    for (std::size_t i = 0; i < torso_head_joints.size(); ++i) {
      if (i != 0) {
        values << ", ";
      }
      values << torso_head_joints[i];
    }
    RCLCPP_INFO(
      node->get_logger(), "Current torso/head joints [rad]: %s", values.str().c_str());

    if (!execute_motion) {
      RCLCPP_INFO(
        node->get_logger(),
        "Read-only mode; no motion sent. Set execute_motion:=true and motion:=joints or named to opt in.");
    } else if (motion == "joints") {
      if (joints.size() != 3) {
        RCLCPP_ERROR(
          node->get_logger(), "motion:=joints requires exactly 3 explicit joint values in radians");
        exit_code = 2;
      } else {
        // Joint order: waist_pitch, head_yaw, head_pitch (radians).
        if (!logResult(node->get_logger(), "moveTorsoHeadJoints", robot.moveTorsoHeadJoints(joints))) {
          exit_code = 1;
        }
      }
    } else if (motion == "named") {
      if (named_target.empty()) {
        RCLCPP_ERROR(node->get_logger(), "motion:=named requires a non-empty named_target");
        exit_code = 2;
      } else {
        if (!logResult(
            node->get_logger(), "moveTorsoHeadNamed", robot.moveTorsoHeadNamed(named_target)))
        {
          exit_code = 1;
        }
      }
    } else {
      RCLCPP_ERROR(
        node->get_logger(),
        "execute_motion is enabled, but motion must be explicitly set to 'joints' or 'named'");
      exit_code = 2;
    }
  } catch (const std::exception & exception) {
    RCLCPP_FATAL(node->get_logger(), "Torso/head SDK example failed: %s", exception.what());
    exit_code = 1;
  }

  executor.cancel();
  spinner.join();
  rclcpp::shutdown();
  return exit_code;
}
