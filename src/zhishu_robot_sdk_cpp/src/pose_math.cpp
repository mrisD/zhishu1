#include "zhishu_robot_sdk/pose_math.hpp"
#include <Eigen/Geometry>
#include <cmath>
#include <stdexcept>

namespace zhishu_robot_sdk
{
namespace
{
Eigen::Quaterniond checked(const geometry_msgs::msg::PoseStamped & point)
{
  const auto & p = point.pose.position;
  const auto & q = point.pose.orientation;
  Eigen::Quaterniond rotation(q.w, q.x, q.y, q.z);
  const double norm = rotation.coeffs().stableNorm();
  if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z) ||
    !rotation.coeffs().allFinite() || !std::isfinite(norm) || norm < 1e-9)
  {
    throw std::invalid_argument("target must be finite with a nonzero quaternion");
  }
  rotation.coeffs() /= norm;
  return rotation;
}
void finite(std::initializer_list<double> values)
{
  for (const auto value : values) {
    if (!std::isfinite(value)) {throw std::invalid_argument("offsets and angles must be finite");}
  }
}
}  // namespace

geometry_msgs::msg::PoseStamped Offs(
  const geometry_msgs::msg::PoseStamped & point, double dx_m, double dy_m, double dz_m)
{
  (void)checked(point);
  finite({dx_m, dy_m, dz_m});
  auto result = point;
  result.pose.position.x += dx_m;
  result.pose.position.y += dy_m;
  result.pose.position.z += dz_m;
  (void)checked(result);
  return result;
}

geometry_msgs::msg::PoseStamped RelTool(
  const geometry_msgs::msg::PoseStamped & point, double dx_m, double dy_m, double dz_m,
  double rx_rad, double ry_rad, double rz_rad)
{
  const auto original = checked(point);
  finite({dx_m, dy_m, dz_m, rx_rad, ry_rad, rz_rad});
  auto result = point;
  // Translation is evaluated in the ORIGINAL target tool orientation.
  const Eigen::Vector3d delta = original * Eigen::Vector3d(dx_m, dy_m, dz_m);
  result.pose.position.x += delta.x();
  result.pose.position.y += delta.y();
  result.pose.position.z += delta.z();
  // ABB TRM RelTool: intrinsic X, then NEW Y, then NEW Z (not fixed XYZ RPY).
  const Eigen::Quaterniond rotation = (original * Eigen::AngleAxisd(rx_rad, Eigen::Vector3d::UnitX()) *
    Eigen::AngleAxisd(ry_rad, Eigen::Vector3d::UnitY()) *
    Eigen::AngleAxisd(rz_rad, Eigen::Vector3d::UnitZ())).normalized();
  result.pose.orientation.x = rotation.x(); result.pose.orientation.y = rotation.y();
  result.pose.orientation.z = rotation.z(); result.pose.orientation.w = rotation.w();
  (void)checked(result);
  return result;
}
}  // namespace zhishu_robot_sdk
