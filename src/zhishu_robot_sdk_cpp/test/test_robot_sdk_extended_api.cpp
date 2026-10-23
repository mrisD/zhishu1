#include <gtest/gtest.h>

#include <type_traits>
#include <utility>
#include <vector>

#include "zhishu_robot_sdk/sdk.hpp"

namespace zhishu_robot_sdk
{
namespace
{
static_assert(std::is_constructible_v<RobotSdk, const rclcpp::Node::SharedPtr &>);
static_assert(std::is_constructible_v<RobotSdk, const rclcpp::Node::SharedPtr &, const RobotSdkOptions &>);
static_assert(std::is_same_v<decltype(std::declval<RobotSdk &>().connect()), Result>);
static_assert(std::is_same_v<decltype(std::declval<RobotSdk &>().setHomePose()), Result>);
static_assert(std::is_same_v<decltype(std::declval<RobotSdk &>().moveHome()), Result>);
static_assert(std::is_same_v<decltype(std::declval<RobotSdk &>().startTrajectoryRecording()), Result>);
static_assert(std::is_same_v<decltype(std::declval<RobotSdk &>().playTrajectory("x")), Result>);
static_assert(std::is_same_v<decltype(std::declval<RobotSdk &>().moveStep(ArmGroup::kLeft, StepOffset{})), Result>);
static_assert(std::is_same_v<decltype(std::declval<RobotSdk &>().vacuumOff()), Result>);
static_assert(std::is_same_v<decltype(std::declval<RobotSdk &>().setSpeedScale(50.0)), Result>);
static_assert(std::is_same_v<decltype(std::declval<RobotSdk &>().readIO(IOAddress{}, std::declval<IOValue &>())), Result>);

TEST(RobotSdkExtendedApiTest, UnifiedApiTypesExposeContractDefaults)
{
  const RobotSdkOptions options{};
  EXPECT_DOUBLE_EQ(options.commander.velocity_scaling, 0.30);
  EXPECT_TRUE(options.trajectory_directory.empty());
  EXPECT_TRUE(options.home_pose_file.empty());
  EXPECT_EQ(RecordingOptions{}.sample_rate_hz, 30.0);
  EXPECT_EQ(TrajectoryPlaybackOptions{}.speed_percent, 50.0);
  EXPECT_FALSE(VacuumStatus{}.supported);
}
}  // namespace
}  // namespace zhishu_robot_sdk
