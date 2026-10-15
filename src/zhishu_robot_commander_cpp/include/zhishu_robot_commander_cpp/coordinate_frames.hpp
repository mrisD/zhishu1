#ifndef ZHISHU_ROBOT_COMMANDER_CPP__COORDINATE_FRAMES_HPP_
#define ZHISHU_ROBOT_COMMANDER_CPP__COORDINATE_FRAMES_HPP_

#include <Eigen/Geometry>
#include <map>
#include <string>
#include <vector>

namespace zhishu_robot_commander_cpp
{
// Empty tool means the API's documented default: tool0 for pose, gripper TCP
// for TCP APIs. No persistent active tool/workobject selection exists.
struct MotionFrames
{
  std::string tool;
  std::string wobj{"wobj0"};
};

class CoordinateFrames
{
public:
  void loadFile(const std::string & path);
  void loadYaml(const std::string & yaml);
  void saveFile(const std::string & path) const;  // Never overwrite an existing file.
  std::string toYaml() const;
  Eigen::Isometry3d tool(const std::string & arm, const std::string & name) const;
  Eigen::Isometry3d workobject(const std::string & name) const;

  static Eigen::Isometry3d flangeTarget(
    const Eigen::Isometry3d & base_T_object, const Eigen::Isometry3d & object_T_tool,
    const Eigen::Isometry3d & flange_T_tool);
  static std::vector<Eigen::Isometry3d> toolLine(
    const Eigen::Isometry3d & start_flange, const Eigen::Isometry3d & end_flange,
    const Eigen::Isometry3d & flange_T_tool, double step);

private:
  std::map<std::string, std::map<std::string, Eigen::Isometry3d>> tools_;
  std::map<std::string, Eigen::Isometry3d> workobjects_;
};
}  // namespace zhishu_robot_commander_cpp
#endif
