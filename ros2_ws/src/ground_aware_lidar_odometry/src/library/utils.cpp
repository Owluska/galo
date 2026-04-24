
#include "ground_aware_lidar_odometry/utils.hpp"

bool IsFinitePoint(float x, float y, float z) {
  return std::isfinite(x) && std::isfinite(y) && std::isfinite(z);
}