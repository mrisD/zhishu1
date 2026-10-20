#pragma once
#include <array>
#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <vector>

namespace pallet_placement {
inline bool stationaryZeroDuration(const std::vector<double>& times,
                                  const std::vector<std::vector<double>>& points) {
  if (times.empty() || times.size()!=points.size() || points.front().empty()) return false;
  for (std::size_t i=0;i<points.size();++i) {
    if (!std::isfinite(times[i]) || times[i]!=0 || points[i].size()!=points.front().size()) return false;
    for (std::size_t j=0;j<points[i].size();++j)
      if (!std::isfinite(points[i][j]) || std::abs(points[i][j]-points.front()[j])>1e-10) return false;
  }
  return true;
}

// Bounded incremental common-mode admittance. References are measured at each
// task boundary; this is not an absolute external-wrench/gravity estimator.
inline bool admittanceCorrectionSafe(double nominal, double corrected) {
  return std::isfinite(nominal) && std::isfinite(corrected) && std::abs(corrected-nominal)<=.002;
}

inline bool carryStepSafe(double actual, double target, double velocity_limit, double horizon) {
  return std::isfinite(actual) && std::isfinite(target) && std::isfinite(velocity_limit) &&
    std::isfinite(horizon) && velocity_limit>0 && horizon>0 && horizon<=1.0 &&
    std::abs(target-actual)<=std::min(.02,.2*velocity_limit*horizon);
}

struct ProcessAdmittance {
  std::array<double, 6> position{}, velocity{};
  void advance(const std::array<double, 6>& residual, double seconds) {
    if (!std::isfinite(seconds) || seconds <= 0 || seconds > 1.0)
      throw std::invalid_argument("Invalid admittance interval");
    for (double f : residual) if (!std::isfinite(f))
      throw std::invalid_argument("Non-finite admittance wrench");
    const int steps = static_cast<int>(std::ceil(seconds / .005));
    const double dt = seconds / steps;
    for (int n=0; n<steps; ++n) for (int i=0; i<6; ++i) {
      const bool linear = i<3;
      const double deadband=linear?3.0:.25, limit=linear?.0005:.0005;
      const double f=std::copysign(std::max(0.0,std::abs(residual[i])-deadband),residual[i]);
      const double acceleration=(std::clamp(f,linear?-40.0:-3.0,linear?40.0:3.0)
        -(linear?250.0:40.0)*velocity[i]-(linear?600.0:80.0)*position[i])/(linear?20.0:5.0);
      velocity[i]=std::clamp(velocity[i]+dt*acceleration,linear?-.0005:-.001,linear?.0005:.001);
      position[i]=std::clamp(position[i]+dt*velocity[i],-limit,limit);
      if ((position[i]>=limit && velocity[i]>0)||(position[i]<=-limit && velocity[i]<0)) velocity[i]=0;
    }
  }
};
}
