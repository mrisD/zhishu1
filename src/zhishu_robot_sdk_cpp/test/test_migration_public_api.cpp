#include "zhishu_robot_sdk/sdk.hpp"

#include <gtest/gtest.h>

#include <string>
#include <type_traits>
#include <utility>
#include <vector>

namespace
{
using zhishu_robot_sdk::ArmGroup;
using zhishu_robot_sdk::Commander;
using zhishu_robot_sdk::ErrorCode;
using zhishu_robot_sdk::GripperGroup;
using zhishu_robot_sdk::MotionHandle;
using zhishu_robot_sdk::Result;
using zhishu_robot_sdk::RobotSdk;

static_assert(!std::is_copy_constructible_v<RobotSdk>, "SDK ownership must remain unique");
static_assert(std::is_constructible_v<RobotSdk, const rclcpp::Node::SharedPtr &>);
static_assert(std::is_constructible_v<RobotSdk, const rclcpp::Node::SharedPtr &, const Commander::Options &>);
static_assert(std::is_same_v<
  decltype(std::declval<RobotSdk &>().moveJoints(ArmGroup::kLeft, std::declval<const std::vector<double> &>())),
  Result>);
static_assert(std::is_same_v<
  decltype(std::declval<RobotSdk &>().openGripper(GripperGroup::kRight)),
  Result>);
static_assert(std::is_same_v<
  decltype(std::declval<RobotSdk &>().moveJointsAsync(ArmGroup::kBoth, std::vector<double>{})),
  MotionHandle>);

TEST(MigrationApiTest, PublicUmbrellaHeaderExportsExistingTypes)
{
  const Result result = Result::success("migration");
  EXPECT_TRUE(result.ok());
  EXPECT_EQ(result.code, ErrorCode::kSuccess);
  EXPECT_EQ(std::string(zhishu_robot_sdk::kSdkVersion), "0.3.1");
}
}  // namespace
