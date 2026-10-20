#pragma once
#include <Eigen/Geometry>
#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <vector>

namespace pallet_placement {
// Horizontal distance between a conservative body projection and the actual
// tabletop rectangle. Deliberately independent of tabletop/body height.
inline double horizontalTableGap(const std::vector<Eigen::Vector3d>& points,
                                const Eigen::Isometry3d& world_from_link)
{
  Eigen::Vector2d low = Eigen::Vector2d::Constant(std::numeric_limits<double>::infinity());
  Eigen::Vector2d high = -low;
  for (const auto& point : points) {
    const Eigen::Vector2d p = (world_from_link * point).head<2>();
    low = low.cwiseMin(p);
    high = high.cwiseMax(p);
  }
  if (points.empty()) return -std::numeric_limits<double>::infinity();
  const Eigen::Vector2d table_low(.655, -2.025), table_high(1.405, -1.075);
  const Eigen::Vector2d gap = (table_low - high).cwiseMax(low - table_high).cwiseMax(0.0);
  return gap.norm();
}

inline std::vector<Eigen::Vector3d> envelopeCorners(const Eigen::Vector3d& low,
                                                  const Eigen::Vector3d& high)
{
  std::vector<Eigen::Vector3d> points;
  for (int i = 0; i < 8; ++i)
    points.emplace_back(i & 1 ? high.x() : low.x(), i & 2 ? high.y() : low.y(),
                        i & 4 ? high.z() : low.z());
  return points;
}
}  // namespace pallet_placement
