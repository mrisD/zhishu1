#include "zhishu_robot_commander_cpp/coordinate_frames.hpp"
#include <gtest/gtest.h>
#include <filesystem>
#include <unistd.h>

using zhishu_robot_commander_cpp::CoordinateFrames;
using zhishu_robot_commander_cpp::MotionFrames;

// Synthetic unit-test geometry only, NOT imported calibration data.
constexpr auto kFrames = R"(
version: 1
tools:
  left:
    tool1: {translation_xyz_m: [0.01, 0, 0.02], orientation_quaternion_xyzw: [0, 0, 0, 1]}
  right:
    tool1: {translation_xyz_m: [0, 0.015, 0.03], orientation_quaternion_xyzw: [0, 0, 1, 1]}
workobjects:
  wobj1:
    user_frame: {translation_xyz_m: [0.1, 0.2, 0.3], orientation_quaternion_xyzw: [0, 0, 1, 1]}
    object_frame: {translation_xyz_m: [0.02, 0, 0], orientation_quaternion_xyzw: [0, 0, 0, 1]}
)";

TEST(CoordinateFrames, BuiltinsAndMotionDefaults)
{
  CoordinateFrames frames;
  EXPECT_TRUE(frames.tool("left", "tool0").matrix().isIdentity());
  EXPECT_TRUE(frames.tool("right", "tool0").matrix().isIdentity());
  EXPECT_TRUE(frames.workobject("wobj0").matrix().isIdentity());
  EXPECT_TRUE(MotionFrames{}.tool.empty());
  EXPECT_EQ(MotionFrames{}.wobj, "wobj0");
}

TEST(CoordinateFrames, UnknownNamesNeverFallback)
{
  CoordinateFrames frames;
  EXPECT_THROW(frames.tool("right", "missing"), std::invalid_argument);
  EXPECT_THROW(frames.tool("both", "tool0"), std::invalid_argument);
  EXPECT_THROW(frames.workobject("missing"), std::invalid_argument);
}

TEST(CoordinateFrames, IndependentArmDefinitions)
{
  CoordinateFrames frames;
  frames.loadYaml(kFrames);
  EXPECT_NEAR(frames.tool("left", "tool1").translation().x(), .01, 1e-12);
  EXPECT_NEAR(frames.tool("right", "tool1").translation().y(), .015, 1e-12);
  EXPECT_NEAR(frames.tool("right", "tool1").rotation()(1, 0), 1., 1e-12);
}

TEST(CoordinateFrames, UserObjectComposition)
{
  CoordinateFrames frames;
  frames.loadYaml(kFrames);
  const auto work = frames.workobject("wobj1");
  EXPECT_TRUE(work.translation().isApprox(Eigen::Vector3d(.1, .22, .3), 1e-12));
}

TEST(CoordinateFrames, ToolTargetBecomesFlangeTarget)
{
  CoordinateFrames frames;
  frames.loadYaml(kFrames);
  Eigen::Isometry3d target = Eigen::Isometry3d::Identity();
  target.translation() = Eigen::Vector3d(.02, -.01, .05);
  target.linear() = Eigen::AngleAxisd(.4, Eigen::Vector3d::UnitX()).toRotationMatrix();
  const auto tool = frames.tool("right", "tool1"), work = frames.workobject("wobj1");
  const auto flange = CoordinateFrames::flangeTarget(work, target, tool);
  EXPECT_TRUE((flange * tool).matrix().isApprox((work * target).matrix(), 1e-12));
}

TEST(CoordinateFrames, InvalidLoadIsAtomic)
{
  CoordinateFrames frames;
  frames.loadYaml(kFrames);
  const auto before = frames.toYaml();
  for (const auto & bad : {
      "version: 2", "version: 1\ntools: []", "version: 1\nworkobjects: []",
      "version: 1\ntools: {both: {}}", "version: 1\nextra: 3",
      "version: 1\nworkobjects: {wobj0: {}}",
      "version: 1\ntools: {right: {tool0: {}}}",
      "version: 1\ntools: {right: {right_gripper_tcp: {}}}",
      "version: 1\nworkobjects: {bad: {translation_xyz_m: [0,0,0], orientation_quaternion_xyzw: [0,0,0,0]}}",
      "version: 1\nworkobjects: {bad: {translation_xyz_m: [.nan,0,0], orientation_quaternion_xyzw: [0,0,0,1]}}",
      "version: 1\nworkobjects: {bad: {translation_xyz_m: [0,0], orientation_quaternion_xyzw: [0,0,0,1]}}"})
  {
    EXPECT_ANY_THROW(frames.loadYaml(bad)) << bad;
    EXPECT_EQ(frames.toYaml(), before);
  }
}

TEST(CoordinateFrames, DuplicateNamesRejected)
{
  CoordinateFrames frames;
  EXPECT_ANY_THROW(frames.loadYaml(R"(
version: 1
tools:
  right:
    tool1: {translation_xyz_m: [0,0,0], orientation_quaternion_xyzw: [0,0,0,1]}
    tool1: {translation_xyz_m: [0,0,0], orientation_quaternion_xyzw: [0,0,0,1]}
)"));
}

TEST(CoordinateFrames, CartesianRotationKeepsOffsetToolOnItsPath)
{
  Eigen::Isometry3d tool = Eigen::Isometry3d::Identity();
  tool.translation() = Eigen::Vector3d(.04, .02, .1);
  Eigen::Isometry3d start = Eigen::Isometry3d::Identity();
  const Eigen::Isometry3d start_tcp = start * tool;
  Eigen::Isometry3d end_tcp = start_tcp;
  end_tcp.linear() = Eigen::AngleAxisd(.6, Eigen::Vector3d::UnitY()).toRotationMatrix();
  const Eigen::Isometry3d end_flange = end_tcp * tool.inverse();
  const auto waypoints = CoordinateFrames::toolLine(start, end_flange, tool, .005);
  ASSERT_GE(waypoints.size(), 30u);
  for (const auto & flange : waypoints) {
    EXPECT_TRUE((flange * tool).translation().isApprox(start_tcp.translation(), 1e-12));
  }
  EXPECT_TRUE(waypoints.back().matrix().isApprox(end_flange.matrix(), 1e-12));
}

TEST(CoordinateFrames, CartesianTranslationAndInvalidSteps)
{
  auto start = Eigen::Isometry3d::Identity(), end = Eigen::Isometry3d::Identity();
  end.translation().x() = .03;
  const auto path = CoordinateFrames::toolLine(start, end, start, .005);
  ASSERT_EQ(path.size(), 6u);
  EXPECT_NEAR(path.front().translation().x(), .005, 1e-12);
  EXPECT_ANY_THROW(CoordinateFrames::toolLine(start, end, start, 0));
  EXPECT_ANY_THROW(CoordinateFrames::toolLine(start, end, start, 1e-9));
}

TEST(CoordinateFrames, SaveRoundTripAndRefuseOverwrite)
{
  CoordinateFrames frames;
  frames.loadYaml(kFrames);
  std::string pattern = (std::filesystem::temp_directory_path() / "commander-frames-XXXXXX").string();
  std::vector<char> name(pattern.begin(), pattern.end());
  name.push_back('\0');
  ASSERT_NE(::mkdtemp(name.data()), nullptr);
  const std::filesystem::path directory(name.data());
  const auto path = (directory / "frames.yaml").string();
  frames.saveFile(path);
  EXPECT_ANY_THROW(frames.saveFile(path));
  CoordinateFrames loaded;
  loaded.loadFile(path);
  EXPECT_TRUE(loaded.workobject("wobj1").matrix().isApprox(frames.workobject("wobj1").matrix(), 1e-12));
  EXPECT_TRUE(loaded.tool("right", "tool1").matrix().isApprox(frames.tool("right", "tool1").matrix(), 1e-12));
  std::filesystem::remove(path);
  std::filesystem::remove(directory);
}
