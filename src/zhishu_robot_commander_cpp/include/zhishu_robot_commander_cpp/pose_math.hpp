#ifndef ZHISHU_ROBOT_COMMANDER_CPP__POSE_MATH_HPP_
#define ZHISHU_ROBOT_COMMANDER_CPP__POSE_MATH_HPP_
#include <geometry_msgs/msg/pose_stamped.hpp>

namespace zhishu_robot_commander_cpp
{
// Pure target functions. Position units: metres. Angles: radians.
// Point must be expressed in the workobject used by its eventual motion.
geometry_msgs::msg::PoseStamped Offs(
  const geometry_msgs::msg::PoseStamped & point, double dx_m, double dy_m, double dz_m);
geometry_msgs::msg::PoseStamped RelTool(
  const geometry_msgs::msg::PoseStamped & point, double dx_m, double dy_m, double dz_m,
  double rx_rad = 0., double ry_rad = 0., double rz_rad = 0.);
}  // namespace zhishu_robot_commander_cpp
#endif
