
#include "ground_aware_lidar_odometry/utils.hpp"

bool IsFinitePoint(float x, float y, float z) {
  return std::isfinite(x) && std::isfinite(y) && std::isfinite(z);
}

float GetDelayMs(const time_pt& start, const time_pt& end) {
  auto milliseconds =
      std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
  return static_cast<float>(milliseconds.count());
}

std::string VectorToString(const Eigen::VectorXd& vec) {
  // Arguments: precision, flags, coeffSeparator, rowSeparator, rowPrefix,
  // rowSuffix, matPrefix, matSuffix
  Eigen::IOFormat CommaInitFmt(Eigen::StreamPrecision, Eigen::DontAlignCols,
                               ", ", ", ", "", "", "[", "]");

  std::stringstream ss;
  ss << vec.format(CommaInitFmt);
  return ss.str();
}

double NormalizeAngle0To2Pi(double a) {
  a = std::fmod(a, 2.0 * M_PI);
  // if (a < 0.0) a += 2.0 * M_PI;
  return a;
}

double NormalizeAngle(double a) { return std::atan2(std::sin(a), std::cos(a)); }

std::tuple<double, double, double> EulersFromMatrixSimple(
    const Eigen::Matrix3d& R) {
  double roll = std::atan2(R(2, 1), R(2, 2));
  double pitch =
      std::atan2(-R(2, 0), std::sqrt(R(2, 1) * R(2, 1) + R(2, 2) * R(2, 2)));
  double yaw = std::atan2(R(1, 0), R(0, 0));
  return std::make_tuple(roll, pitch, yaw);
}

double DegToRad(double deg) { return deg * M_PI / 180.0; }