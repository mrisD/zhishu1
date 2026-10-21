// Runtime regression consumer. Requires the dedicated GenericSystem graph;
// deliberately not registered as a hardware-independent CTest.
#include "zhishu_robot_sdk/sdk.hpp"

#include <rclcpp/rclcpp.hpp>

#include <atomic>
#include <chrono>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <thread>
#include <vector>

using namespace std::chrono_literals;

namespace
{
void require(bool condition, const char * message)
{
  if (!condition) {throw std::runtime_error(message);}
}
}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>(
    "sdk_simulation_regression",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  std::thread spinner([&]() {executor.spin();});
  int exit_code = 0;
  try {
    using namespace zhishu_robot_sdk;
    RobotSdk sdk(node);
    const auto initial = sdk.readState();
    require(initial.result.ok(), "required state feedback is unavailable");
    require(initial.left_tcp_pose_valid && initial.right_tcp_pose_valid,
      "TCP feedback is unavailable in the complete simulation model");

    const auto expect_invalid = [](const Result & r) {
        require(r.code == ErrorCode::kInvalidArgument, "invalid target was not rejected");
      };
    for (const double position : {-0.001, 0.061, std::numeric_limits<double>::infinity()}) {
      expect_invalid(sdk.moveGripper(GripperGroup::kLeft, position));
      expect_invalid(sdk.moveBothGrippers(0.0, position));
      auto a = sdk.moveGripperAsync(GripperGroup::kBoth, position);
      require(a.done(), "invalid gripper target spawned a worker");
      expect_invalid(a.result());
      auto b = sdk.moveBothGrippersAsync(position, 0.0);
      require(b.done(), "invalid dual-gripper target spawned a worker");
      expect_invalid(b.result());
    }
    expect_invalid(sdk.moveJoints(ArmGroup::kLeft, {0.0}));
    auto bad_joints = sdk.moveJointsAsync(
      ArmGroup::kBoth, std::vector<double>(14, std::numeric_limits<double>::quiet_NaN()));
    require(bad_joints.done(), "NaN joint target spawned a worker");
    expect_invalid(bad_joints.result());
    const auto after_invalid = sdk.readState();
    require(after_invalid.result.ok(), "state unavailable after rejected inputs");
    for (std::size_t i = 0; i < 7; ++i) {
      require(std::abs(after_invalid.left_arm_joints[i] - initial.left_arm_joints[i]) < .002,
        "left arm moved on a rejected input");
      require(std::abs(after_invalid.right_arm_joints[i] - initial.right_arm_joints[i]) < .002,
        "right arm moved on a rejected input");
    }
    RCLCPP_INFO(node->get_logger(), "[REGRESSION][INPUT_VALIDATION_OK]");

    // A missing named target ends quickly without contacting the motion action
    // server, exercising admission/reaping under concurrent short tasks.
    constexpr const char * missing_target = "__sdk_regression_nonexistent_target__";
    std::atomic<bool> start{false};
    std::atomic<int> busy{0}, rejected{0}, unexpected{0};
    std::vector<std::thread> callers;
    for (int t = 0; t < 8; ++t) {
      callers.emplace_back([&, t]() {
          while (!start.load()) {std::this_thread::yield();}
          for (int i = 0; i < 100; ++i) {
            Result result;
            if ((i + t) % 2 == 0) {
              result = sdk.moveNamedAsync(ArmGroup::kBoth, missing_target).waitFor(3s);
            } else {
              result = sdk.moveNamed(ArmGroup::kBoth, missing_target);
            }
            if (result.code == ErrorCode::kBusy) {++busy;}
            else if (result.code == ErrorCode::kTargetRejected) {++rejected;}
            else {++unexpected;}
          }
        });
    }
    start.store(true);
    for (auto & caller : callers) {caller.join();}
    require(busy + rejected == 800 && rejected > 0 && unexpected == 0,
      "concurrent SDK calls produced an unexpected result");
    require(!sdk.status().busy, "SDK remained busy after short tasks completed");
    RCLCPP_INFO(node->get_logger(),
      "[REGRESSION][CONCURRENCY_OK] calls=800 rejected=%d busy=%d", rejected.load(), busy.load());

    // An unavailable optional TCP must not poison valid mandatory joints.
    RobotSdkOptions options;
    options.commander.left_tcp_link = "__sdk_regression_missing_tcp__";
    RobotSdk missing_tcp(node, options);
    const auto partial = missing_tcp.readState();
    require(partial.result.ok() && partial.left_arm_joints_valid &&
      partial.right_arm_joints_valid && partial.torso_head_joints_valid,
      "optional TCP failure invalidated required feedback");
    require(!partial.left_tcp_pose_valid && partial.right_tcp_pose_valid,
      "optional feedback validity flags are incorrect");
    RCLCPP_INFO(node->get_logger(), "[REGRESSION][OPTIONAL_FEEDBACK_OK]");
    RCLCPP_INFO(node->get_logger(), "[REGRESSION][PASS]");
  } catch (const std::exception & error) {
    RCLCPP_ERROR(node->get_logger(), "[REGRESSION][FAIL] %s", error.what());
    exit_code = 1;
  }
  executor.cancel();
  spinner.join();
  rclcpp::shutdown();
  return exit_code;
}
