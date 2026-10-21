#include "zhishu_robot_sdk/coordinate_frames.hpp"
#include <yaml-cpp/yaml.h>
#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <regex>
#include <stdexcept>
#include <system_error>
#include <unistd.h>

namespace zhishu_robot_sdk
{
namespace
{
void keys(const YAML::Node & node, const std::vector<std::string> & allowed)
{
  if (!node.IsMap()) {throw std::invalid_argument("coordinate entry must be a map");}
  std::vector<std::string> seen;
  for (const auto & item : node) {
    const auto key = item.first.as<std::string>();
    if (std::find(allowed.begin(), allowed.end(), key) == allowed.end() ||
      std::find(seen.begin(), seen.end(), key) != seen.end())
    {
      throw std::invalid_argument("unknown or duplicate coordinate field: " + key);
    }
    seen.push_back(key);
  }
}
void nameCheck(const std::string & name)
{
  if (!std::regex_match(name, std::regex("[A-Za-z][A-Za-z0-9_-]*"))) {
    throw std::invalid_argument("invalid coordinate name: " + name);
  }
}

Eigen::Isometry3d pose(const YAML::Node & value)
{
  keys(value, {"translation_xyz_m", "orientation_quaternion_xyzw"});
  const auto p = value["translation_xyz_m"];
  const auto q = value["orientation_quaternion_xyzw"];
  if (!p.IsSequence() || p.size() != 3 || !q.IsSequence() || q.size() != 4) {
    throw std::invalid_argument("pose requires translation_xyz_m[3] and orientation_quaternion_xyzw[4]");
  }
  Eigen::Isometry3d t = Eigen::Isometry3d::Identity();
  t.translation() = Eigen::Vector3d(p[0].as<double>(), p[1].as<double>(), p[2].as<double>());
  Eigen::Quaterniond rotation(q[3].as<double>(), q[0].as<double>(), q[1].as<double>(), q[2].as<double>());
  const double norm = rotation.coeffs().stableNorm();
  if (!t.translation().allFinite() || !rotation.coeffs().allFinite() || !std::isfinite(norm) || norm < 1e-9) {
    throw std::invalid_argument("coordinate pose must be finite with a nonzero quaternion");
  }
  rotation.coeffs() /= norm;
  t.linear() = rotation.toRotationMatrix();
  return t;
}

YAML::Node encodedPose(const Eigen::Isometry3d & t)
{
  YAML::Node value;
  for (int i = 0; i < 3; ++i) {
    value["translation_xyz_m"].push_back(t.translation()[i]);
  }
  const Eigen::Quaterniond q(t.rotation());
  for (double v : {q.x(), q.y(), q.z(), q.w()}) {
    value["orientation_quaternion_xyzw"].push_back(v);
  }
  return value;
}
}  // namespace

void CoordinateFrames::loadFile(const std::string & path)
{
  std::ifstream stream(path);
  if (!stream) {throw std::invalid_argument("cannot read coordinate file: " + path);}
  const std::string contents((std::istreambuf_iterator<char>(stream)), std::istreambuf_iterator<char>());
  loadYaml(contents);
}

void CoordinateFrames::loadYaml(const std::string & text)
{
  // Parse into a candidate. Any invalid entry leaves the previous registry intact.
  const auto root = YAML::Load(text);
  if (!root.IsMap() || !root["version"] || root["version"].as<int>() != 1) {
    throw std::invalid_argument("coordinate file requires version: 1");
  }
  keys(root, {"version", "tools", "workobjects"});
  for (const auto & item : root) {
    const auto key = item.first.as<std::string>();
    if (key != "version" && key != "tools" && key != "workobjects") {
      throw std::invalid_argument("unknown coordinate registry key: " + key);
    }
  }
  CoordinateFrames candidate;
  const auto tools = root["tools"];
  if (tools && !tools.IsMap()) {throw std::invalid_argument("tools must be a map");}
  if (tools) {
    for (const auto & item : tools) {
      const auto arm = item.first.as<std::string>();
      if ((arm != "left" && arm != "right") || !item.second.IsMap()) {
        throw std::invalid_argument("tools must be grouped by left/right arm");
      }
      if (candidate.tools_.count(arm)) {throw std::invalid_argument("duplicate tool arm: " + arm);}
      auto & group = candidate.tools_[arm];
      for (const auto & entry : item.second) {
        const auto name = entry.first.as<std::string>();
        nameCheck(name);
        if (name == "tool0" || name == "left_gripper_tcp" || name == "right_gripper_tcp") {
          throw std::invalid_argument("cannot redefine built-in tool: " + name);
        }
        if (!group.emplace(name, pose(entry.second)).second) {
          throw std::invalid_argument("duplicate tool: " + name);
        }
      }
    }
  }
  const auto workobjects = root["workobjects"];
  if (workobjects && !workobjects.IsMap()) {throw std::invalid_argument("workobjects must be a map");}
  if (workobjects) {
    for (const auto & entry : workobjects) {
      const auto name = entry.first.as<std::string>();
      nameCheck(name);
      if (name == "wobj0") {throw std::invalid_argument("cannot redefine wobj0");}
      const auto value = entry.second;
      if (value["user_frame"]) {keys(value, {"user_frame", "object_frame"});}
      const auto user = value["user_frame"] ? pose(value["user_frame"]) : pose(value);
      const Eigen::Isometry3d object = value["object_frame"] ?
        pose(value["object_frame"]) : Eigen::Isometry3d::Identity();
      const Eigen::Isometry3d composed = user * object;
      if (!composed.matrix().allFinite()) {throw std::invalid_argument("workobject composition overflow");}
      if (!candidate.workobjects_.emplace(name, composed).second) {
        throw std::invalid_argument("duplicate workobject: " + name);
      }
    }
  }
  *this = std::move(candidate);
}

std::string CoordinateFrames::toYaml() const
{
  YAML::Node root;
  root["version"] = 1;
  root["tools"] = YAML::Node(YAML::NodeType::Map);
  root["workobjects"] = YAML::Node(YAML::NodeType::Map);
  for (const auto & arm : {"left", "right"}) {
    root["tools"][arm] = YAML::Node(YAML::NodeType::Map);
    const auto found = tools_.find(arm);
    if (found != tools_.end()) {
      for (const auto & entry : found->second) {root["tools"][arm][entry.first] = encodedPose(entry.second);}
    }
  }
  for (const auto & entry : workobjects_) {root["workobjects"][entry.first] = encodedPose(entry.second);}
  YAML::Emitter output;
  output.SetDoublePrecision(17);
  output << root;
  return std::string(output.c_str()) + "\n";
}

void CoordinateFrames::saveFile(const std::string & path) const
{
  if (path.empty()) {throw std::invalid_argument("coordinate save path is empty");}
  // Same-directory temporary file + atomic hard link: no partial target and
  // no overwrite race. A caller must explicitly choose a new destination.
  std::string temporary = path + ".tmp.XXXXXX";
  std::vector<char> buffer(temporary.begin(), temporary.end());
  buffer.push_back('\0');
  const int fd = ::mkstemp(buffer.data());
  if (fd < 0) {throw std::system_error(errno, std::generic_category(), "create coordinate temporary file");}
  temporary = buffer.data();
  try {
    const auto text = toYaml();
    std::size_t position = 0;
    while (position < text.size()) {
      const auto n = ::write(fd, text.data() + position, text.size() - position);
      if (n <= 0) {throw std::system_error(errno, std::generic_category(), "write coordinate file");}
      position += static_cast<std::size_t>(n);
    }
    if (::fsync(fd) != 0) {throw std::system_error(errno, std::generic_category(), "sync coordinate file");}
    if (::link(temporary.c_str(), path.c_str()) != 0) {
      throw std::system_error(errno, std::generic_category(), "save coordinate file (existing files are not overwritten)");
    }
  } catch (...) {
    ::close(fd); ::unlink(temporary.c_str()); throw;
  }
  ::close(fd); ::unlink(temporary.c_str());
}

Eigen::Isometry3d CoordinateFrames::tool(const std::string & arm, const std::string & name) const
{
  if (arm != "left" && arm != "right") {throw std::invalid_argument("arm must be left or right");}
  if (name == "tool0") {return Eigen::Isometry3d::Identity();}
  const auto group = tools_.find(arm);
  if (group != tools_.end()) {
    const auto found = group->second.find(name);
    if (found != group->second.end()) {return found->second;}
  }
  throw std::invalid_argument("unknown " + arm + " tool: " + name);
}

Eigen::Isometry3d CoordinateFrames::workobject(const std::string & name) const
{
  if (name == "wobj0") {return Eigen::Isometry3d::Identity();}
  const auto found = workobjects_.find(name);
  if (found == workobjects_.end()) {throw std::invalid_argument("unknown workobject: " + name);}
  return found->second;
}

Eigen::Isometry3d CoordinateFrames::flangeTarget(
  const Eigen::Isometry3d & base_T_object, const Eigen::Isometry3d & object_T_tool,
  const Eigen::Isometry3d & flange_T_tool)
{
  return base_T_object * object_T_tool * flange_T_tool.inverse();
}

std::vector<Eigen::Isometry3d> CoordinateFrames::toolLine(
  const Eigen::Isometry3d & start_flange, const Eigen::Isometry3d & end_flange,
  const Eigen::Isometry3d & flange_T_tool, double step)
{
  if (!std::isfinite(step) || step <= 0.) {throw std::invalid_argument("Cartesian step must be positive");}
  const Eigen::Isometry3d start = start_flange * flange_T_tool, end = end_flange * flange_T_tool;
  const Eigen::Quaterniond q0(start.rotation()), q1(end.rotation());
  const double count = std::max({1., std::ceil((end.translation() - start.translation()).norm() / step),
    std::ceil(q0.angularDistance(q1) / .02)});
  if (count > 10000.) {throw std::invalid_argument("Cartesian request exceeds 10000 waypoints");}
  std::vector<Eigen::Isometry3d> poses;
  for (int i = 1; i <= static_cast<int>(count); ++i) {
    const double alpha = i / count;
    Eigen::Isometry3d target = Eigen::Isometry3d::Identity();
    target.translation() = (1. - alpha) * start.translation() + alpha * end.translation();
    target.linear() = q0.slerp(alpha, q1).toRotationMatrix();
    poses.push_back(target * flange_T_tool.inverse());
  }
  return poses;
}
}  // namespace zhishu_robot_sdk
