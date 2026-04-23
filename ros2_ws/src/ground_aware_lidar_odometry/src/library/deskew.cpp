#include "ground_aware_lidar_odometry/deskew.hpp"
DeskewAlgorithm::DeskewAlgorithm(const DeskewParams& params,
                                 const rclcpp::Logger& logger,
                                 const rclcpp::Clock& clock)
    : prms_(params), logger_(logger), clock_(clock) {
  imu_queue_.Resize(prms_.imu_queue_size);
  lidar_queue_.Resize(prms_.lidar_queue_size);
};

bool DeskewAlgorithm::isFinitePoint(float x, float y, float z) const {
  return std::isfinite(x) && std::isfinite(y) && std::isfinite(z);
}

double DeskewAlgorithm::PointTimeFromIndex(double relative_time,
                                           double scan_header_time) const {
  const double scan_start = prms_.stamp_is_scan_end_
                                ? scan_header_time - prms_.scan_period_
                                : scan_header_time;
  return scan_start + relative_time * prms_.scan_period_;
}

void DeskewAlgorithm::UnwrapAzimuthParams(
    const sensor_msgs::msg::PointCloud2& msg) {
  sensor_msgs::msg::PointCloud2 msg_local = msg;
  const size_t n = static_cast<size_t>(msg_local.width) *
                   static_cast<size_t>(msg_local.height);
  sensor_msgs::PointCloud2Iterator<float> x_it(msg_local, "x");
  sensor_msgs::PointCloud2Iterator<float> y_it(msg_local, "y");
  azs_.Reset(n);
  for (size_t idx = 0; idx < n; ++idx, ++x_it, ++y_it) {
    const float x = *x_it;
    const float y = *y_it;

    if (!isFinitePoint(x, y)) continue;

    // Calculate base azimuth [0, 2PI)
    double az = std::atan2(y, x);
    if (az < 0.0) az += 2.0 * M_PI;

    if (azs_.last_az >= 0.0) {
      double diff = az - azs_.last_az;
      if (diff < -M_PI)
        azs_.offset += 2.0 * M_PI;
      else if (diff > M_PI)
        azs_.offset -= 2.0 * M_PI;
    }
    azs_.last_az = az;

    double u_az = az + azs_.offset;
    azs_.unwrapped_az[idx] = u_az;

    // Track min and max on the fly
    azs_.min_az = std::min(azs_.min_az, u_az);
    azs_.max_az = std::max(azs_.max_az, u_az);
  }
}

std::optional<sensor_msgs::msg::PointCloud2>
DeskewAlgorithm::ProcessCloudsQueue() {
  if (imu_queue_.Size() < 2 || lidar_queue_.Size() < 1) {
    return {};
  }
  // RCLCPP_WARN(logger_, "29 %d ", lidar_queue_.Size());
  sensor_msgs::msg::PointCloud2 out = *lidar_queue_.PeerFront();
  UnwrapAzimuthParams(out);
  double az_span = azs_.max_az - azs_.min_az;
  if (az_span <= 1e-6) {
    RCLCPP_WARN(logger_, "Invalid azimuth span, skipping deskew");
    return {};
  }
  rclcpp::Time msg_time(out.header.stamp);
  double scan_time = msg_time.seconds();
  const size_t n =
      static_cast<size_t>(out.width) * static_cast<size_t>(out.height);
  double last_pt_relative_time =
      (azs_.unwrapped_az[n - 1] - azs_.min_az) / (azs_.max_az - azs_.min_az);
  double last_point_time = PointTimeFromIndex(last_pt_relative_time, scan_time);
  if (imu_queue_.PeerBack().time < last_point_time) {
    RCLCPP_DEBUG(logger_,
                 "Waiting for future IMU data. last imu=%.6f last point=%.6f",
                 imu_queue_.PeerBack().time, last_point_time);
    return {};
  }

  sensor_msgs::PointCloud2Iterator<float> x_it(out, "x");
  sensor_msgs::PointCloud2Iterator<float> y_it(out, "y");
  size_t imu_idx = 0;

  for (size_t idx = 0; idx < n; ++idx, ++x_it, ++y_it) {
    const float x = *x_it;
    const float y = *y_it;

    if (!isFinitePoint(x, y)) {
      // RCLCPP_WARN(logger_, "Infinite point");
      continue;
    }
    double relative_time =
        (azs_.unwrapped_az[idx] - azs_.min_az) / (azs_.max_az - azs_.min_az);

    double point_time = PointTimeFromIndex(relative_time, scan_time);
    // Motion from point time to scan reference time
    double deskew_dt = prms_.stamp_is_scan_end_ ? (scan_time - point_time)
                                                : (point_time - scan_time);
    if (deskew_dt < 0.0) {
      RCLCPP_WARN(logger_, "Negative deskew_dt: %.6f", deskew_dt);
      continue;
    }
    while (imu_idx + 1 < imu_queue_.Size() &&
           imu_queue_[imu_idx + 1].time < point_time) {
      imu_idx++;
    }
    if (imu_idx + 1 >= imu_queue_.Size()) {
      RCLCPP_WARN(logger_, "Queue index is out of range: %d",
                  static_cast<int>(imu_idx));
      RCLCPP_INFO(logger_,
                  "Scan time: %.6f | point_time %.6f | IMU buffer: %zu | first "
                  "imu: %.6f | last imu: %.6f",
                  scan_time, point_time, imu_queue_.Size(),
                  imu_queue_.PeerFront().time, imu_queue_.PeerBack().time);
      return {};
    }

    const auto& imu0 = imu_queue_[imu_idx];
    const auto& imu1 = imu_queue_[imu_idx + 1];

    double imu_dt = imu1.time - imu0.time;
    if (imu_dt <= 1e-6) {
      RCLCPP_WARN(logger_, "Too little imu_dt: %.6f", imu_dt);
      continue;
    }
    double alpha = (point_time - imu0.time) / imu_dt;
    alpha = std::clamp(alpha, 0.0, 1.0);
    double rate = (1.0 - alpha) * imu0.rate + alpha * imu1.rate;
    double dyaw = -rate * deskew_dt;
    const double cos_yaw = std::cos(dyaw);
    const double sin_yaw = std::sin(dyaw);

    float rx = cos_yaw * x - sin_yaw * y;
    float ry = sin_yaw * x + cos_yaw * y;

    *x_it = rx;
    *y_it = ry;

    RCLCPP_INFO_THROTTLE(
        logger_, clock_, prms_.log_throttle,
        "idx=%zu t=%.6f dt=%.4f alpha=%.2f rate=%.3f dyaw=%.4f", idx,
        point_time, deskew_dt, alpha, rate, dyaw);
  }
  lidar_queue_.PopFront();
  return out;
}