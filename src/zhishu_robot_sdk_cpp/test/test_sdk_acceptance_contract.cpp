#include "zhishu_robot_sdk/sdk.hpp"

#include <gtest/gtest.h>

#include <string>
#include <type_traits>
#include <utility>
#include <vector>

namespace zhishu_robot_sdk
{
namespace
{

TEST(SdkAcceptanceContractTest, PublicVersionIs031)
{
  EXPECT_STREQ(kSdkVersion, "0.3.1");
  EXPECT_EQ(kSdkVersionMajor, 0);
  EXPECT_EQ(kSdkVersionMinor, 3);
  EXPECT_EQ(kSdkVersionPatch, 1);
}

TEST(SdkAcceptanceContractTest, DefaultStatusNeverClaimsReadyOrConnected)
{
  const SdkStatus sdk_status{};
  const ConnectionStatus connection_status{};
  EXPECT_FALSE(sdk_status.ready);
  EXPECT_FALSE(sdk_status.sdk_initialized);
  EXPECT_FALSE(sdk_status.ros_context_available);
  EXPECT_FALSE(sdk_status.motion_interface_ready);
  EXPECT_FALSE(connection_status.connected);
  EXPECT_FALSE(connection_status.motion_interface_ready);
}

TEST(SdkAcceptanceContractTest, RobotStateExposesOptionalFeedbackValidity)
{
  const RobotState state{};
  EXPECT_FALSE(state.left_arm_joints_valid);
  EXPECT_FALSE(state.right_arm_joints_valid);
  EXPECT_FALSE(state.torso_head_joints_valid);
  EXPECT_FALSE(state.left_gripper_joints_valid);
  EXPECT_FALSE(state.right_gripper_joints_valid);
  EXPECT_FALSE(state.left_arm_pose_valid);
  EXPECT_FALSE(state.right_arm_pose_valid);
  EXPECT_FALSE(state.left_tcp_pose_valid);
  EXPECT_FALSE(state.right_tcp_pose_valid);
}

TEST(SdkAcceptanceContractTest, TrajectoryAndScanDefaultsAreDocumented)
{
  const RecordingOptions recording{};
  const TrajectoryPlaybackOptions playback{};
  const ScanOptions scan{};
  EXPECT_DOUBLE_EQ(recording.sample_rate_hz, 30.0);
  EXPECT_DOUBLE_EQ(playback.speed_percent, 50.0);
  EXPECT_EQ(playback.arms, ArmGroup::kBoth);
  EXPECT_EQ(scan.transport, ScanTransport::kTopic);
  EXPECT_TRUE(scan.endpoint_name.empty());
  EXPECT_TRUE(scan.topic_name.empty());
  EXPECT_TRUE(scan.service_name.empty());
}

static_assert(std::is_same_v<
  decltype(std::declval<const RobotSdk &>().listTrajectories(
    std::declval<std::vector<TrajectoryInfo> &>())), Result>);

}  // namespace
}  // namespace zhishu_robot_sdk
