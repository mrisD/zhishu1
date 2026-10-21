#include "zhishu_robot_sdk/pose_math.hpp"
#include "zhishu_robot_sdk/robot_sdk.hpp"
#include <Eigen/Geometry>
#include <gtest/gtest.h>
#include <limits>
#include <cmath>

using namespace zhishu_robot_sdk;

geometry_msgs::msg::PoseStamped example()
{
  geometry_msgs::msg::PoseStamped point;
  point.header.frame_id = "wobj_test";
  point.header.stamp.sec = 12;
  point.pose.position.x = .1; point.pose.position.y = .2; point.pose.position.z = .3;
  const Eigen::Quaterniond q(Eigen::AngleAxisd(std::acos(-1.)/2, Eigen::Vector3d::UnitZ()));
  point.pose.orientation.x = q.x(); point.pose.orientation.y = q.y();
  point.pose.orientation.z = q.z(); point.pose.orientation.w = q.w();
  return point;
}

TEST(PoseMath, OffsUsesObjectAxesAndDoesNotMutate)
{
  const auto p = example();
  const auto out = Offs(p, .01, -.02, .03);
  EXPECT_NEAR(out.pose.position.x, .11, 1e-12);
  EXPECT_NEAR(out.pose.position.y, .18, 1e-12);
  EXPECT_NEAR(out.pose.position.z, .33, 1e-12);
  EXPECT_EQ(out.pose.orientation, p.pose.orientation);
  EXPECT_EQ(out.header, p.header);
  EXPECT_DOUBLE_EQ(p.pose.position.x, .1);
}

TEST(PoseMath, RelToolUsesOriginalToolAxesBeforeRotation)
{
  const auto p = example();
  const auto out = RobotSdk::RelTool(p, .01, 0, 0, 0, 0, std::acos(-1.)/2);
  EXPECT_NEAR(out.pose.position.x, .1, 1e-12);
  EXPECT_NEAR(out.pose.position.y, .21, 1e-12);
  EXPECT_NEAR(std::abs(out.pose.orientation.z), 1., 1e-12);
  EXPECT_EQ(out.header, p.header);
}

TEST(PoseMath, CombinedRotationUsesIntrinsicXThenNewYThenNewZ)
{
  const auto p = example();
  const auto out = RelTool(p, 0, 0, 0, .2, -.3, .4);
  const Eigen::Quaterniond q0(p.pose.orientation.w, 0, 0, p.pose.orientation.z);
  const Eigen::Quaterniond expected = q0 * Eigen::AngleAxisd(.2, Eigen::Vector3d::UnitX()) *
    Eigen::AngleAxisd(-.3, Eigen::Vector3d::UnitY()) * Eigen::AngleAxisd(.4, Eigen::Vector3d::UnitZ());
  const Eigen::Quaterniond actual(out.pose.orientation.w, out.pose.orientation.x,
    out.pose.orientation.y, out.pose.orientation.z);
  EXPECT_NEAR(actual.angularDistance(expected), 0., 1e-12);
}

TEST(PoseMath, HelpersComposeAndPreserveMetadata)
{
  const auto p = example();
  const auto out = RobotSdk::Offs(RelTool(p, .01, 0, 0), 0, 0, .02);
  EXPECT_NEAR(out.pose.position.y, .21, 1e-12);
  EXPECT_NEAR(out.pose.position.z, .32, 1e-12);
  EXPECT_EQ(out.header, p.header);
}

TEST(PoseMath, InvalidPoseAndOffsetsRejected)
{
  auto p = example();
  const auto nan = std::numeric_limits<double>::quiet_NaN();
  EXPECT_THROW(Offs(p, nan, 0, 0), std::invalid_argument);
  EXPECT_THROW(RelTool(p, 0, 0, 0, nan), std::invalid_argument);
  p.pose.orientation.z = p.pose.orientation.w = 0.;
  EXPECT_THROW(Offs(p, 0, 0, 0), std::invalid_argument);
  EXPECT_THROW(RelTool(p, 0, 0, 0), std::invalid_argument);
}

TEST(PoseMath, LargeFiniteQuaternionDoesNotSilentlyBecomeIdentity)
{
  auto p = example();
  p.pose.orientation.z = 1e300;
  p.pose.orientation.w = 1e300;
  const auto out = RelTool(p, .01, 0, 0);
  EXPECT_NEAR(out.pose.position.x, .1, 1e-12);
  EXPECT_NEAR(out.pose.position.y, .21, 1e-12);
}
