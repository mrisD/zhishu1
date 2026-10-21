#include "zhishu_robot_sdk/motion_handle.hpp"
#include "zhishu_robot_sdk/result.hpp"
#include "zhishu_robot_sdk/version.hpp"

#include <zhishu_robot_interfaces/msg/commander_status.hpp>

#include <gtest/gtest.h>

namespace zhishu_robot_sdk
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

TEST(CommanderStatusContractTest, ErrorCodesMatchRosMessageConstants)
{
  using Status = zhishu_robot_interfaces::msg::CommanderStatus;
  EXPECT_EQ(static_cast<int>(ErrorCode::kSuccess), Status::SUCCESS);
  EXPECT_EQ(static_cast<int>(ErrorCode::kInvalidArgument), Status::INVALID_ARGUMENT);
  EXPECT_EQ(static_cast<int>(ErrorCode::kTargetRejected), Status::TARGET_REJECTED);
  EXPECT_EQ(static_cast<int>(ErrorCode::kPlanningFailed), Status::PLANNING_FAILED);
  EXPECT_EQ(static_cast<int>(ErrorCode::kExecutionFailed), Status::EXECUTION_FAILED);
  EXPECT_EQ(static_cast<int>(ErrorCode::kBusy), Status::BUSY);
  EXPECT_EQ(static_cast<int>(ErrorCode::kCancelled), Status::CANCELLED);
  EXPECT_EQ(static_cast<int>(ErrorCode::kTimeout), Status::TIMEOUT);
  EXPECT_EQ(static_cast<int>(ErrorCode::kNotReady), Status::NOT_READY);
  EXPECT_EQ(static_cast<int>(ErrorCode::kInternalError), Status::INTERNAL_ERROR);
}

TEST(VersionTest, HeaderMatchesPackageMinorRelease)
{
  EXPECT_STREQ(kSdkVersion, "0.3.1");
  EXPECT_EQ(kSdkVersionMajor, 0);
  EXPECT_EQ(kSdkVersionMinor, 3);
  EXPECT_EQ(kSdkVersionPatch, 1);
}

}  // namespace
}  // namespace zhishu_robot_sdk
