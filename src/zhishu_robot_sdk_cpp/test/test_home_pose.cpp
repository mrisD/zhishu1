#include "home_pose.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <filesystem>
#include <fstream>
#include <string>
#include <system_error>

namespace zhishu_robot_sdk
{
namespace detail
{
namespace
{

geometry_msgs::msg::PoseStamped armPose(double y, double qw = 1.0)
{
  geometry_msgs::msg::PoseStamped pose;
  pose.header.frame_id = "base_link";
  pose.pose.position.x = 0.35;
  pose.pose.position.y = y;
  pose.pose.position.z = 0.75;
  pose.pose.orientation.w = qw;
  return pose;
}

HomePose validHome()
{
  HomePose home;
  home.captured_at = "2026-10-21T00:00:00Z";
  home.left = armPose(0.25);
  home.right = armPose(-0.25);
  return home;
}

TEST(HomePoseTest, EncodeDecodeRoundTrip)
{
  const auto home = validHome();
  std::string json;
  std::string error;
  ASSERT_TRUE(encodeHomePose(home, json, error)) << error;
  EXPECT_NE(json.find("\"version\": 1"), std::string::npos);
  EXPECT_NE(json.find("base_link"), std::string::npos);

  HomePose decoded;
  ASSERT_TRUE(decodeHomePose(json, decoded, error)) << error;
  EXPECT_EQ(decoded.captured_at, home.captured_at);
  EXPECT_EQ(decoded.left.header.frame_id, "base_link");
  EXPECT_EQ(decoded.right.header.frame_id, "base_link");
  EXPECT_DOUBLE_EQ(decoded.left.pose.position.y, 0.25);
  EXPECT_DOUBLE_EQ(decoded.right.pose.position.y, -0.25);
  EXPECT_DOUBLE_EQ(decoded.left.pose.position.z, 0.75);
  EXPECT_DOUBLE_EQ(decoded.left.pose.orientation.w, 1.0);
}

TEST(HomePoseTest, EncodeRejectsUnusablePoses)
{
  std::string json;
  std::string error;

  auto home = validHome();
  home.left.header.frame_id.clear();
  EXPECT_FALSE(encodeHomePose(home, json, error));
  EXPECT_FALSE(error.empty());

  home = validHome();
  home.right.pose.position.x = std::nan("");
  EXPECT_FALSE(encodeHomePose(home, json, error));
  EXPECT_FALSE(error.empty());

  home = validHome();
  home.right.pose.orientation.w = 0.0;
  EXPECT_FALSE(encodeHomePose(home, json, error));
  EXPECT_FALSE(error.empty());
}

TEST(HomePoseTest, DecodeRejectsMalformedRecords)
{
  std::string json;
  std::string error;
  ASSERT_TRUE(encodeHomePose(validHome(), json, error)) << error;

  HomePose decoded;
  EXPECT_FALSE(decodeHomePose("not json", decoded, error));
  EXPECT_FALSE(decodeHomePose("[]", decoded, error));
  EXPECT_FALSE(decodeHomePose("{\"left\": {}}", decoded, error));
  EXPECT_FALSE(decodeHomePose(
    "{\"version\": 2, \"left\": {}, \"right\": {}}", decoded, error));

  auto replace = [](std::string text, const std::string & from, const std::string & to) {
      const auto position = text.find(from);
      EXPECT_NE(position, std::string::npos);
      return text.replace(position, from.size(), to);
    };
  std::string missing_right = json;
  {
    const auto position = missing_right.find("  \"right\": {");
    ASSERT_NE(position, std::string::npos);
    missing_right.erase(position);
  }
  EXPECT_FALSE(decodeHomePose(missing_right, decoded, error));

  const std::string non_finite = replace(json, "\"z\": 0.75", "\"z\": 1e999");
  EXPECT_FALSE(decodeHomePose(non_finite, decoded, error));

  const std::string zero_quaternion = replace(json, "\"w\": 1", "\"w\": 0");
  EXPECT_FALSE(decodeHomePose(zero_quaternion, decoded, error));
}

TEST(HomePoseTest, DecodeAcceptsForwardCompatibleMetadata)
{
  const std::string json =
    "{\n"
    "  \"version\": 1,\n"
    "  \"captured_at\": \"2026-10-21T00:00:00Z\",\n"
    "  \"note\": \"added by a newer writer\",\n"
    "  \"left\": {\"frame_id\": \"base_link\", \"position\": {\"x\": 0.35, \"y\": 0.25, "
    "\"z\": 0.75}, \"orientation\": {\"x\": 0.0, \"y\": 0.0, \"z\": 0.0, \"w\": 1.0}},\n"
    "  \"right\": {\"frame_id\": \"base_link\", \"position\": {\"x\": 0.35, \"y\": -0.25, "
    "\"z\": 0.75}, \"orientation\": {\"x\": 0.0, \"y\": 0.0, \"z\": 0.0, \"w\": 1.0}}\n"
    "}\n";

  HomePose decoded;
  std::string error;
  ASSERT_TRUE(decodeHomePose(json, decoded, error)) << error;
  EXPECT_DOUBLE_EQ(decoded.left.pose.position.y, 0.25);
  EXPECT_DOUBLE_EQ(decoded.right.pose.position.y, -0.25);
}

TEST(HomePoseTest, WriteAndReadFileRoundTrip)
{
  const auto path =
    (std::filesystem::path(::testing::TempDir()) / "zhishu_home_pose_unit_test.json").string();
  std::error_code ignored;
  std::filesystem::remove(path, ignored);

  std::string error;
  ASSERT_TRUE(writeHomePoseFile(path, validHome(), error)) << error;
  EXPECT_FALSE(std::filesystem::exists(path + ".tmp"));

  HomePose decoded;
  ASSERT_TRUE(readHomePoseFile(path, decoded, error)) << error;
  EXPECT_DOUBLE_EQ(decoded.left.pose.position.y, 0.25);
  EXPECT_EQ(decoded.captured_at, "2026-10-21T00:00:00Z");

  // A second save must replace the existing record instead of appending to it.
  auto updated = validHome();
  updated.left.pose.position.z = 0.5;
  ASSERT_TRUE(writeHomePoseFile(path, updated, error)) << error;
  ASSERT_TRUE(readHomePoseFile(path, decoded, error)) << error;
  EXPECT_DOUBLE_EQ(decoded.left.pose.position.z, 0.5);

  std::filesystem::remove(path, ignored);
}

TEST(HomePoseTest, ReadRejectsMissingFileAndUnusableContent)
{
  const auto directory = std::filesystem::path(::testing::TempDir());
  const auto missing = (directory / "zhishu_home_pose_absent.json").string();
  std::error_code ignored;
  std::filesystem::remove(missing, ignored);

  HomePose decoded;
  std::string error;
  EXPECT_FALSE(readHomePoseFile(missing, decoded, error));
  EXPECT_FALSE(error.empty());

  const auto corrupt = (directory / "zhishu_home_pose_corrupt.json").string();
  {
    std::ofstream stream(corrupt, std::ios::trunc);
    ASSERT_TRUE(static_cast<bool>(stream));
    stream << "{\"version\": 1}";
  }
  EXPECT_FALSE(readHomePoseFile(corrupt, decoded, error));
  std::filesystem::remove(corrupt, ignored);
}

TEST(HomePoseTest, PathResolutionUsesExplicitPathAndHomeFallback)
{
  EXPECT_EQ(resolvedHomePosePath("/tmp/zhishu_home.json"), "/tmp/zhishu_home.json");
  EXPECT_EQ(resolvedHomePosePath(""), defaultHomePosePath());
  const auto default_path = defaultHomePosePath();
  if (!default_path.empty()) {
    EXPECT_NE(default_path.find("home_pose.json"), std::string::npos);
  }
  EXPECT_FALSE(utcNowIso8601().empty());
}

}  // namespace
}  // namespace detail
}  // namespace zhishu_robot_sdk
