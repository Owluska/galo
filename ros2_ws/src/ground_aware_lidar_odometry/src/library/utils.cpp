
#include "ground_aware_lidar_odometry/utils.hpp"

bool IsFinitePoint(float x, float y, float z) {
  return std::isfinite(x) && std::isfinite(y) && std::isfinite(z);
}

float GetDelayMs(const time_pt& start, const time_pt& end) {
  auto milliseconds =
      std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
  return static_cast<float>(milliseconds.count());
}