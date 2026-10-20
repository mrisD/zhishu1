#pragma once

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace pallet_placement
{
constexpr double kWeight = 15.0 * 9.81;

inline bool releaseReady(bool preload_unloaded, bool fresh, double support,
                         double height_error, double tilt_degrees)
{
  return preload_unloaded && fresh && std::isfinite(support) &&
    std::isfinite(height_error) && std::isfinite(tilt_degrees) &&
    support >= 0.90 * kWeight && support <= 1.10 * kWeight &&
    std::abs(height_error) <= 0.0015 && std::abs(tilt_degrees) <= 0.5;
}

inline double integrateVerticalReference(double reference, double step, double initial_floor_gap)
{
  if (!std::isfinite(reference) || !std::isfinite(step) || !std::isfinite(initial_floor_gap))
    throw std::invalid_argument("Non-finite floor position reference");
  // At most 0.5 mm of commanded compression past the measured floor height.
  return std::clamp(reference + step, -initial_floor_gap - 0.0005, 0.002);
}

// Positive world Z relieves excessive pressure; negative Z transfers weight.
inline double verticalStep(double support, double height_above_floor)
{
  if (!std::isfinite(support) || !std::isfinite(height_above_floor))
    throw std::invalid_argument("Non-finite floor admittance feedback");
  if (support < 5.0 && height_above_floor > 0.003) return -0.002;
  if (support < 5.0 && height_above_floor > 0.0005) return -0.00025;
  if (std::abs(support - kWeight) <= 5.0) return 0.0;
  return std::clamp((support - kWeight) / 600000.0 * 0.1, -0.00005, 0.00005);
}
}  // namespace pallet_placement
