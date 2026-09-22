#include "zhishu_robot_commander_cpp/sdk.hpp"

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/parameter_client.hpp>

#include <array>
#include <chrono>
#include <future>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

namespace
{
// Joint order follows the left_arm MoveIt group: shoulder pitch/roll/yaw,
// elbow pitch, wrist yaw/pitch/roll.
using Pose = std::array<double, 7>;

// These poses are centered around the robot's validated raised-arm posture and
// vary both the board position and orientation for an Eye-to-Hand solve.
const std::array<Pose, 12> kCalibrationPoses{{
  {{-1.8010, -0.0919, -1.4576, 0.2305, 1.1331, 0.7153, -0.4490}},
  {{-1.8010, -0.0919, -1.4576, 0.3000, 1.0000, 0.6000, -0.3000}},
  {{-2.0822, -0.0729, -1.5280, 0.2000, 0.4000, 0.0500, -0.5000}},
  {{-2.0822, -0.0729, -1.5280, 0.1200, 0.2256, -0.0796, -0.6282}},
  {{-1.7652, 0.0691, -0.8699, 0.2000, 1.1000, 0.9000, -1.0500}},
  {{-1.7652, 0.0691, -0.8699, 0.1200, 0.9859, 1.0126, -1.2000}},
  {{-1.7240, -0.0294, -1.3465, 0.0935, 1.8382, 0.0301, -1.0577}},
  {{-1.6961, -0.3052, -0.1272, 0.2631, 1.1635, 0.0212, -0.6832}},
  {{-1.5452, -0.1728, -0.0730, 0.8000, 1.5500, 0.5000, -0.1000}},
  {{-1.5452, -0.1728, -0.0730, 0.7150, 1.6793, 0.6521, -0.2634}},
  {{-1.3670, 0.0620, -0.8758, 1.0500, 1.3500, -0.0500, 0.2000}},
  {{-1.3670, 0.0620, -0.8758, 0.9184, 1.4938, -0.1578, 0.3599}},
}};

void importMoveItDescriptions(const rclcpp::Node::SharedPtr & node)
{
  auto client = std::make_shared<rclcpp::AsyncParametersClient>(node, "/move_group");
  if (!client->wait_for_service(std::chrono::seconds(5))) {
    throw std::runtime_error("/move_group parameter service is unavailable");
  }

  const std::vector<std::string> names{
    "robot_description", "robot_description_semantic"};
  auto future = client->get_parameters(names);
  if (future.wait_for(std::chrono::seconds(5)) != std::future_status::ready) {
    throw std::runtime_error("timed out reading MoveIt robot descriptions");
  }
  const auto parameters = future.get();
  if (parameters.size() != names.size()) {
    throw std::runtime_error("failed to read MoveIt robot descriptions");
  }
  for (std::size_t index = 0; index < names.size(); ++index) {
    if (parameters[index].get_type() != rclcpp::ParameterType::PARAMETER_STRING ||
      parameters[index].as_string().empty())
    {
      throw std::runtime_error("MoveIt parameter is empty: " + names[index]);
    }
    if (node->has_parameter(names[index])) {
      node->set_parameter(parameters[index]);
    } else {
      node->declare_parameter<std::string>(names[index], parameters[index].as_string());
    }
  }
}
}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>(
    "handeye_pose_sequence",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  std::thread spinner([&executor]() {executor.spin();});

  int exit_code = 0;
  try {
    importMoveItDescriptions(node);
    zhishu_robot_commander_cpp::Commander::Options options;
    options.left_end_effector_link = "L_hand_link";
    options.velocity_scaling = 0.15;
    options.acceleration_scaling = 0.10;
    options.planning_time = 10.0;
    options.planning_attempts = 10;
    zhishu_robot_commander_cpp::RobotSdk robot(node, options);

    std::cout << "\nEye-to-Hand sample sequence\n"
              << "Verify the ArUco board is detected, then click Take Sample "
              << "in RViz after each move.\n"
              << "The head must remain at yaw=0.5, pitch=0.15 for the whole run.\n\n";

    for (std::size_t index = 0; index < kCalibrationPoses.size(); ++index) {
      std::cout << '[' << index + 1 << '/' << kCalibrationPoses.size()
                << "] Press Enter to plan and execute, or q then Enter to stop: ";
      std::string answer;
      std::getline(std::cin, answer);
      if (answer == "q" || answer == "Q") {
        break;
      }

      const auto & pose = kCalibrationPoses[index];
      const std::vector<double> joints(pose.begin(), pose.end());
      const auto result = robot.moveJoints(
        zhishu_robot_commander_cpp::ArmGroup::kLeft, joints);
      if (!result) {
        RCLCPP_ERROR(
          node->get_logger(), "Pose %zu failed [%s]: %s", index + 1,
          zhishu_robot_commander_cpp::toString(result.code), result.message.c_str());
        exit_code = 1;
        break;
      }
      std::this_thread::sleep_for(std::chrono::seconds(1));
      std::cout << "Settled. Confirm target detection and click Take Sample.\n";
    }
  } catch (const std::exception & exception) {
    RCLCPP_FATAL(node->get_logger(), "Pose sequence failed: %s", exception.what());
    exit_code = 1;
  }

  executor.cancel();
  spinner.join();
  rclcpp::shutdown();
  return exit_code;
}
