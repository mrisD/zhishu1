#ifndef ZHISHU_ROBOT_SDK__HOME_POSE_HPP_
#define ZHISHU_ROBOT_SDK__HOME_POSE_HPP_

#include <geometry_msgs/msg/pose_stamped.hpp>

#include <string>

namespace zhishu_robot_sdk
{
namespace detail
{

/// SRDF group state used when no unified Home record has been saved yet. It is the only
/// built-in fallback; a saved arm pose is never paired with this posture for the other arm.
inline constexpr const char * kHomeNamedTarget = "both_arms_home";

/// One unified dual-arm Home captured at a single instant. Both end-effector poses always come
/// from the same capture and therefore from the same file.
struct HomePose
{
  /// UTC capture time in ISO 8601 form; informational only.
  std::string captured_at;
  geometry_msgs::msg::PoseStamped left;
  geometry_msgs::msg::PoseStamped right;
};

/// $HOME/.ros/zhishu_robot_sdk_cpp/home_pose.json. Empty when HOME is unavailable.
std::string defaultHomePosePath();

/// An explicit configured path, or defaultHomePosePath() when the configured path is empty.
std::string resolvedHomePosePath(const std::string & configured_path);

/// Current UTC time as ``YYYY-MM-DDTHH:MM:SSZ``.
std::string utcNowIso8601();

/// Serialize a unified Home record as JSON. Rejects missing frames, non-finite values and a
/// zero quaternion instead of writing an unusable file.
bool encodeHomePose(const HomePose & home, std::string & json, std::string & error);

/// Parse a record written by encodeHomePose(). Any malformed or non-finite record is rejected;
/// a partially valid record is never reported as success.
bool decodeHomePose(const std::string & json, HomePose & home, std::string & error);

/// Write through a temporary sibling file so an interrupted save never leaves a partial Home.
bool writeHomePoseFile(const std::string & path, const HomePose & home, std::string & error);

/// Read and validate a saved unified Home record.
bool readHomePoseFile(const std::string & path, HomePose & home, std::string & error);

}  // namespace detail
}  // namespace zhishu_robot_sdk

#endif  // ZHISHU_ROBOT_SDK__HOME_POSE_HPP_
