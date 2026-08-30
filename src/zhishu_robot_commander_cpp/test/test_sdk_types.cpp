#include "zhishu_robot_commander_cpp/motion_handle.hpp"
#include "zhishu_robot_commander_cpp/result.hpp"
#include "zhishu_robot_commander_cpp/version.hpp"

#include <gtest/gtest.h>

namespace zhishu_robot_commander_cpp
{
namespace
{

TEST(ResultTest, SuccessConvertsToTrue)
{
  const auto result = Result::success("done");
  EXPECT_TRUE(result.ok());
  EXPECT_TRUE(static_cast<bool>(result));
  EXPECT_EQ(result.code, ErrorCode::kSuccess);
  EXPECT_STREQ(toString(result.code), "success");
}

TEST(ResultTest, FailurePreservesDetails)
{
  const auto result = Result::failure(ErrorCode::kPlanningFailed, "no plan", -1);
  EXPECT_FALSE(result.ok());
  EXPECT_EQ(result.message, "no plan");
  EXPECT_EQ(result.native_code, -1);
  EXPECT_STREQ(toString(result.code), "planning_failed");
}

TEST(MotionHandleTest, DefaultHandleIsInvalidAndSafe)
{
  MotionHandle handle;
  EXPECT_FALSE(handle.valid());
  EXPECT_FALSE(handle.done());
  EXPECT_EQ(handle.status(), MotionStatus::kInvalid);
  EXPECT_EQ(handle.result().code, ErrorCode::kNotReady);
  EXPECT_EQ(handle.wait().code, ErrorCode::kNotReady);
  EXPECT_FALSE(handle.cancel());
}

TEST(MotionStatusTest, TerminalStatesAreExplicit)
{
  EXPECT_FALSE(isTerminal(MotionStatus::kQueued));
  EXPECT_FALSE(isTerminal(MotionStatus::kRunning));
  EXPECT_FALSE(isTerminal(MotionStatus::kCancelling));
  EXPECT_TRUE(isTerminal(MotionStatus::kSucceeded));
  EXPECT_TRUE(isTerminal(MotionStatus::kFailed));
  EXPECT_TRUE(isTerminal(MotionStatus::kCancelled));
}

TEST(VersionTest, HeaderMatchesPackageMinorRelease)
{
  EXPECT_STREQ(kSdkVersion, "0.2.0");
  EXPECT_EQ(kSdkVersionMajor, 0);
  EXPECT_EQ(kSdkVersionMinor, 2);
  EXPECT_EQ(kSdkVersionPatch, 0);
}

}  // namespace
}  // namespace zhishu_robot_commander_cpp
