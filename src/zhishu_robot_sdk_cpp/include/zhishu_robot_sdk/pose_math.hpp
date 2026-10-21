#ifndef ZHISHU_ROBOT_SDK__POSE_MATH_HPP_
#define ZHISHU_ROBOT_SDK__POSE_MATH_HPP_
#include <geometry_msgs/msg/pose_stamped.hpp>
#include "zhishu_robot_sdk/visibility_control.h"

namespace zhishu_robot_sdk
{
// Pure target functions. Position units: metres. Angles: radians.
// Point must be expressed in the workobject used by its eventual motion.
ZHISHU_ROBOT_SDK_PUBLIC geometry_msgs::msg::PoseStamped Offs(
  const geometry_msgs::msg::PoseStamped & point, double dx_m, double dy_m, double dz_m);
ZHISHU_ROBOT_SDK_PUBLIC geometry_msgs::msg::PoseStamped RelTool(
  const geometry_msgs::msg::PoseStamped & point, double dx_m, double dy_m, double dz_m,
  double rx_rad = 0., double ry_rad = 0., double rz_rad = 0.);
}  // namespace zhishu_robot_sdk
#endif
