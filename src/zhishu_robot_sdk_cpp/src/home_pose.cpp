#include "home_pose.hpp"

#include <yaml-cpp/yaml.h>

#include <chrono>
#include <cmath>
#include <cstdlib>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iterator>
#include <sstream>
#include <system_error>

namespace zhishu_robot_sdk
{
namespace detail
{
namespace
{
constexpr int kHomePoseVersion = 1;

bool validPose(
  const geometry_msgs::msg::PoseStamped & pose, const char * label, std::string & error)
{
  const auto & position = pose.pose.position;
  const auto & orientation = pose.pose.orientation;
  const double norm = std::hypot(
    std::hypot(orientation.x, orientation.y), std::hypot(orientation.z, orientation.w));
  if (pose.header.frame_id.empty()) {
    error = std::string(label) + " home pose requires a non-empty frame_id";
    return false;
  }
  if (!std::isfinite(position.x) || !std::isfinite(position.y) || !std::isfinite(position.z) ||
    !std::isfinite(orientation.x) || !std::isfinite(orientation.y) ||
    !std::isfinite(orientation.z) || !std::isfinite(orientation.w) ||
    !std::isfinite(norm) || norm < 1e-9)
  {
    error = std::string(label) +
      " home pose requires finite XYZ and a nonzero quaternion";
    return false;
  }
  return true;
}

std::string escaped(const std::string & text)
{
  std::ostringstream out;
  for (const char character : text) {
    switch (character) {
      case '"':
        out << "\\\"";
        break;
      case '\\':
        out << "\\\\";
        break;
      case '\n':
        out << "\\n";
        break;
      case '\r':
        out << "\\r";
        break;
      case '\t':
        out << "\\t";
        break;
      default:
        if (static_cast<unsigned char>(character) < 0x20) {
          out << "\\u" << std::hex << std::setw(4) << std::setfill('0')
              << static_cast<int>(static_cast<unsigned char>(character)) << std::dec;
        } else {
          out << character;
        }
    }
  }
  return out.str();
}

void appendPose(
  std::ostringstream & out, const char * key,
  const geometry_msgs::msg::PoseStamped & pose, bool last)
{
  const auto & position = pose.pose.position;
  const auto & orientation = pose.pose.orientation;
  out << "  \"" << key << "\": {\n"
      << "    \"frame_id\": \"" << escaped(pose.header.frame_id) << "\",\n"
      << "    \"position\": {\"x\": " << position.x << ", \"y\": " << position.y
      << ", \"z\": " << position.z << "},\n"
      << "    \"orientation\": {\"x\": " << orientation.x << ", \"y\": " << orientation.y
      << ", \"z\": " << orientation.z << ", \"w\": " << orientation.w << "}\n"
      << "  }" << (last ? "\n" : ",\n");
}

bool decodePose(
  const YAML::Node & node, const char * label,
  geometry_msgs::msg::PoseStamped & pose, std::string & error)
{
  if (!node || !node.IsMap()) {
    error = std::string("home pose requires a \"") + label + "\" mapping";
    return false;
  }
  const auto frame_id = node["frame_id"];
  const auto position = node["position"];
  const auto orientation = node["orientation"];
  if (!frame_id || !frame_id.IsScalar() || !position || !position.IsMap() ||
    !orientation || !orientation.IsMap())
  {
    error = std::string("home pose \"") + label +
      "\" requires frame_id, position and orientation";
    return false;
  }

  geometry_msgs::msg::PoseStamped decoded;
  try {
    decoded.header.frame_id = frame_id.as<std::string>();
    decoded.pose.position.x = position["x"].as<double>();
    decoded.pose.position.y = position["y"].as<double>();
    decoded.pose.position.z = position["z"].as<double>();
    decoded.pose.orientation.x = orientation["x"].as<double>();
    decoded.pose.orientation.y = orientation["y"].as<double>();
    decoded.pose.orientation.z = orientation["z"].as<double>();
    decoded.pose.orientation.w = orientation["w"].as<double>();
  } catch (const std::exception &) {
    error = std::string("home pose \"") + label + "\" contains a missing or non-numeric value";
    return false;
  }
  if (!validPose(decoded, label, error)) {
    return false;
  }
  pose = decoded;
  return true;
}

bool readTextFile(const std::string & path, std::string & contents, std::string & error)
{
  std::ifstream stream(path);
  if (!stream) {
    error = "cannot read " + path;
    return false;
  }
  contents.assign(
    std::istreambuf_iterator<char>(stream), std::istreambuf_iterator<char>());
  return true;
}
}  // namespace

std::string defaultHomePosePath()
{
  const char * home = std::getenv("HOME");
  if (home == nullptr || *home == '\0') {
    return {};
  }
  return (std::filesystem::path(home) / ".ros" / "zhishu_robot_sdk_cpp" / "home_pose.json")
         .string();
}

std::string resolvedHomePosePath(const std::string & configured_path)
{
  return configured_path.empty() ? defaultHomePosePath() : configured_path;
}

std::string utcNowIso8601()
{
  const std::time_t seconds =
    std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
  std::tm utc{};
  gmtime_r(&seconds, &utc);
  char buffer[32];
  std::strftime(buffer, sizeof(buffer), "%Y-%m-%dT%H:%M:%SZ", &utc);
  return buffer;
}

bool encodeHomePose(const HomePose & home, std::string & json, std::string & error)
{
  if (!validPose(home.left, "left", error) || !validPose(home.right, "right", error)) {
    return false;
  }

  // JSON is a subset of YAML, so the file is written as plain JSON and parsed with yaml-cpp.
  std::ostringstream out;
  out << std::setprecision(17);
  out << "{\n"
      << "  \"version\": " << kHomePoseVersion << ",\n"
      << "  \"captured_at\": \"" << escaped(home.captured_at) << "\",\n";
  appendPose(out, "left", home.left, false);
  appendPose(out, "right", home.right, true);
  out << "}\n";
  json = out.str();
  error.clear();
  return true;
}

bool decodeHomePose(const std::string & json, HomePose & home, std::string & error)
{
  YAML::Node root;
  try {
    root = YAML::Load(json);
  } catch (const std::exception & exception) {
    error = std::string("home pose is not valid JSON: ") + exception.what();
    return false;
  }
  if (!root.IsMap()) {
    error = "home pose must contain a JSON object";
    return false;
  }
  try {
    const auto version = root["version"];
    if (!version || !version.IsScalar() || version.as<int>() != kHomePoseVersion) {
      error = "home pose requires \"version\": 1";
      return false;
    }
  } catch (const std::exception &) {
    error = "home pose requires a numeric \"version\": 1";
    return false;
  }

  HomePose decoded;
  if (!decodePose(root["left"], "left", decoded.left, error)) {
    return false;
  }
  if (!decodePose(root["right"], "right", decoded.right, error)) {
    return false;
  }
  try {
    const auto captured_at = root["captured_at"];
    if (captured_at && captured_at.IsScalar()) {
      decoded.captured_at = captured_at.as<std::string>();
    }
  } catch (const std::exception &) {
    decoded.captured_at.clear();
  }

  home = decoded;
  error.clear();
  return true;
}

bool writeHomePoseFile(const std::string & path, const HomePose & home, std::string & error)
{
  std::string json;
  if (!encodeHomePose(home, json, error)) {
    return false;
  }
  if (path.empty()) {
    error = "home pose file path must not be empty";
    return false;
  }

  const std::filesystem::path file(path);
  std::error_code failure;
  if (file.has_parent_path()) {
    std::filesystem::create_directories(file.parent_path(), failure);
    if (failure) {
      error = "cannot create " + file.parent_path().string() + ": " + failure.message();
      return false;
    }
  }

  const std::filesystem::path temporary(path + ".tmp");
  {
    std::ofstream stream(temporary, std::ios::trunc);
    if (!stream) {
      error = "cannot write " + temporary.string();
      return false;
    }
    stream << json;
    stream.flush();
    if (!stream) {
      error = "cannot write " + temporary.string();
      return false;
    }
  }

  std::filesystem::rename(temporary, file, failure);
  if (failure) {
    // POSIX replaces an existing file atomically; other platforms may need an explicit remove.
    std::error_code ignored;
    std::filesystem::remove(file, ignored);
    failure.clear();
    std::filesystem::rename(temporary, file, failure);
  }
  if (failure) {
    std::error_code ignored;
    std::filesystem::remove(temporary, ignored);
    error = "cannot replace " + path + ": " + failure.message();
    return false;
  }

  error.clear();
  return true;
}

bool readHomePoseFile(const std::string & path, HomePose & home, std::string & error)
{
  std::string contents;
  if (!readTextFile(path, contents, error)) {
    return false;
  }
  return decodeHomePose(contents, home, error);
}

}  // namespace detail
}  // namespace zhishu_robot_sdk
