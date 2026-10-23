#ifndef ZHISHU_ROBOT_SDK__HOME_POSE_HPP_
#define ZHISHU_ROBOT_SDK__HOME_POSE_HPP_

#include <geometry_msgs/msg/pose_stamped.hpp>

#include <cstddef>
#include <string>
#include <vector>

namespace zhishu_robot_sdk
{
namespace detail
{

/// SRDF group state used when no unified Home record has been saved yet. It is the only
/// built-in fallback; a saved arm pose is never paired with this posture for the other arm.
inline constexpr const char * kHomeNamedTarget = "both_arms_home";

/// Joint count of the unified dual-arm Home: left arm seven joints, then right arm seven.
inline constexpr std::size_t kUnifiedHomeJointCount = 14;

/// One unified dual-arm Home captured at a single instant. Joints and end-effector poses always
/// come from the same capture and therefore from the same file.
struct HomePose
{
  /// UTC capture time in ISO 8601 form; informational only.
  std::string captured_at;

  /// Left seven joints followed by right seven joints, in the both_arms group order. Empty in a
  /// legacy record written before joints were stored; moveHome() then falls back to the poses.
  std::vector<double> joints;

  geometry_msgs::msg::PoseStamped left;
  geometry_msgs::msg::PoseStamped right;
};

/// $HOME/.ros/zhishu_robot_sdk_cpp/home_pose.json. Empty when HOME is unavailable.
std::string defaultHomePosePath();

/// An explicit configured path, or defaultHomePosePath() when the configured path is empty.
std::string resolvedHomePosePath(const std::string & configured_path);

/// Current UTC time as ``YYYY-MM-DDTHH:MM:SSZ``.
std::string utcNowIso8601();

/// Serialize a unified Home record as JSON. Joints may be empty or exactly 14 finite values.
/// Rejects missing frames, non-finite values and a zero quaternion instead of writing an
/// unusable file.
bool encodeHomePose(const HomePose & home, std::string & json, std::string & error);

/// Parse a record written by encodeHomePose(). A missing "joints" block is accepted as a legacy
/// pose-only record; malformed or non-finite joints are rejected.
bool decodeHomePose(const std::string & json, HomePose & home, std::string & error);

/// Write through a temporary sibling file so an interrupted save never leaves a partial Home.
bool writeHomePoseFile(const std::string & path, const HomePose & home, std::string & error);

/// Read and validate a saved unified Home record.
bool readHomePoseFile(const std::string & path, HomePose & home, std::string & error);

}  // namespace detail
}  // namespace zhishu_robot_sdk

#endif  // ZHISHU_ROBOT_SDK__HOME_POSE_HPP_
