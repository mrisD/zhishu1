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
  {{-2.270, -0.296, 2.320, 1.343, 0.715, 0.977, 0.663}},
  {{-2.150, -0.380, 2.250, 1.250, 0.600, 0.900, 0.550}},
  {{-2.360, -0.220, 2.390, 1.430, 0.820, 1.050, 0.760}},
  {{-2.100, -0.260, 2.440, 1.300, 0.780, 0.850, 0.720}},
  {{-2.400, -0.340, 2.200, 1.470, 0.620, 1.100, 0.600}},
  {{-2.230, -0.430, 2.360, 1.380, 0.880, 0.930, 0.500}},
  {{-2.320, -0.180, 2.270, 1.270, 0.540, 1.030, 0.820}},
  {{-2.080, -0.330, 2.310, 1.410, 0.740, 0.800, 0.580}},
  {{-2.420, -0.250, 2.430, 1.320, 0.900, 1.120, 0.700}},
  {{-2.180, -0.400, 2.180, 1.490, 0.660, 0.880, 0.780}},
  {{-2.350, -0.200, 2.350, 1.220, 0.850, 1.000, 0.520}},
  {{-2.120, -0.300, 2.400, 1.450, 0.580, 1.080, 0.740}},
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
    options.left_end_effector_link = "left_gripper_tcp";
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
