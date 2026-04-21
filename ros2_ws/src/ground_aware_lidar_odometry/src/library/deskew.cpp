#include "ground_aware_lidar_odometry/deskew.hpp"
DeskewAlgorithm::DeskewAlgorithm(const DeskewParams& params,
                                 const rclcpp::Logger& logger)
    : prms_(params), logger_(logger) {};

void DeskewAlgorithm::UpdateQueue(double last_speed, double yaw_rate,
                                  double time) {
  imu_queue_.push_back({time, yaw_rate, last_speed});
  while (imu_queue_.size() > static_cast<size_t>(prms_.imu_queue_size)) {
    imu_queue_.pop_front();
  }
}
bool DeskewAlgorithm::isFinitePoint(float x, float y, float z) const {
  return std::isfinite(x) && std::isfinite(y) && std::isfinite(z);
}

PlanarMotion DeskewAlgorithm::IntegratePlanarMotion(double v, double yaw_rate,
                                                    double dt) const {
  // Inputs:
  // v         - forward velocity in body frame (m/s)
  // omega     - yaw rate (rad/s)
  // T         - scan duration (s)

  // Outputs (motion over full scan, in body frame at scan start):
  // dx, dy    - translation in XY plane
  // dyaw      - yaw change

  // If turning (|omega| > epsilon):
  // dx   = (v / omega) * sin(omega * T)
  // dy   = (v / omega) * (1 - cos(omega * T))
  // dyaw = omega * T

  // If moving straight (omega ~ 0):
  // dx   = v * T
  // dy   = 0
  // dyaw = 0
  PlanarMotion m;
  m.dx = (v / yaw_rate) * std::sin(yaw_rate * dt);
  m.dy = (v / yaw_rate) * (1.0 - std::cos(yaw_rate * dt));
  m.dyaw = yaw_rate * dt;
  return m;
}

double DeskewAlgorithm::PointTimeFromIndex(size_t index, size_t n_points,
                                           double scan_header_time) const {
  if (n_points <= 1) return scan_header_time;
  const double alpha =
      static_cast<double>(index) / static_cast<double>(n_points - 1);
  const double scan_start = prms_.stamp_is_scan_end_
                                ? scan_header_time - prms_.scan_period_
                                : scan_header_time;
  return scan_start + alpha * prms_.scan_period_;
}

sensor_msgs::msg::PointCloud2 DeskewAlgorithm::ProcessCloud(
    const sensor_msgs::msg::PointCloud2::SharedPtr msg) const {
  sensor_msgs::msg::PointCloud2 out = *msg;
  rclcpp::Time msg_time(msg->header.stamp);

  double scan_time = msg_time.seconds();
  if (imu_queue_.size() < 2) {
    return out;
  }
  RCLCPP_INFO(
      logger_,
      "Scan time: %.6f | IMU buffer: %zu | first imu: %.6f | last imu: %.6f",
      scan_time, imu_queue_.size(), imu_queue_.front().time,
      imu_queue_.back().time);
  const size_t n =
      static_cast<size_t>(msg->width) * static_cast<size_t>(msg->height);

  sensor_msgs::PointCloud2Iterator<float> x_it(out, "x");
  sensor_msgs::PointCloud2Iterator<float> y_it(out, "y");
  size_t imu_idx = 0;
  for (size_t idx = 0; idx < n; ++idx, ++x_it, ++y_it) {
    const float x = *x_it;
    const float y = *y_it;
    if (!isFinitePoint(x, y)) continue;
    double point_time = PointTimeFromIndex(idx, n, scan_time);
    // Motion from point time to scan reference time
    double deskew_dt = prms_.stamp_is_scan_end_ ? (scan_time - point_time)
                                                : (point_time - scan_time);
    if (deskew_dt < 0.0) {
      RCLCPP_WARN(logger_, "Negative deskew_dt: %.6f", deskew_dt);
      continue;
    }
    while (imu_idx + 1 < imu_queue_.size() &&
           imu_queue_[imu_idx + 1].time < point_time) {
      imu_idx++;
    }
    if (imu_idx + 1 >= imu_queue_.size()) break;

    const auto& imu0 = imu_queue_[imu_idx];
    const auto& imu1 = imu_queue_[imu_idx + 1];

    double imu_dt = imu1.time - imu0.time;
    if (imu_dt <= 1e-6) continue;

    double alpha = (point_time - imu0.time) / imu_dt;
    alpha = std::clamp(alpha, 0.0, 1.0);
    double rate = (1.0 - alpha) * imu0.rate + alpha * imu1.rate;
    double speed = (1.0 - alpha) * imu0.last_speed + alpha * imu1.last_speed;

    auto m = IntegratePlanarMotion(speed, rate, deskew_dt);

    const double cos_yaw = std::cos(m.dyaw);
    const double sin_yaw = std::sin(m.dyaw);

    Eigen::Vector2f pt(x, y);
    Eigen::Vector2f rotated;
    rotated.x() = static_cast<float>(cos_yaw * pt.x() - sin_yaw * pt.y());
    rotated.y() = static_cast<float>(sin_yaw * pt.x() + cos_yaw * pt.y());

    rotated.x() += static_cast<float>(m.dx);
    rotated.y() += static_cast<float>(m.dy);

    *x_it = rotated.x();
    *y_it = rotated.y();
    if (idx == 0 || idx == n / 2 || idx == n - 1) {
      RCLCPP_INFO(logger_,
                  "idx=%zu t=%.6f dt=%.4f alpha=%.2f rate=%.3f dx=%.3f dy=%.3f "
                  "dyaw=%.3f",
                  idx, point_time, deskew_dt, alpha, rate, m.dx, m.dy, m.dyaw);
    }
  }
  return out;
}