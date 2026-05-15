#include "ground_aware_lidar_odometry/deskew.hpp"
DeskewAlgorithm::DeskewAlgorithm(const DeskewParams& params,
                                 const rclcpp::Logger& logger,
                                 const rclcpp::Clock& clock)
    : prms_(params), logger_(logger), clock_(clock) {
  imu_queue_.Resize(prms_.imu_queue_size);
  lidar_queue_.Resize(prms_.lidar_queue_size);
  speed_queue_.Resize(prms_.speed_queue_size);
};

double DeskewAlgorithm::PointTimeFromIndex(double relative_time,
                                           double scan_header_time) const {
  const double scan_start = prms_.stamp_is_scan_end_
                                ? scan_header_time - prms_.scan_period_
                                : scan_header_time;
  return scan_start + relative_time * prms_.scan_period_;
}

void DeskewAlgorithm::UnwrapAzimuthParams(
    const sensor_msgs::msg::PointCloud2& msg) {
  const size_t n =
      static_cast<size_t>(msg.width) * static_cast<size_t>(msg.height);

  sensor_msgs::PointCloud2ConstIterator<float> x_it(msg, "x");
  sensor_msgs::PointCloud2ConstIterator<float> y_it(msg, "y");

  azs_.Reset(n);

  for (size_t idx = 0; idx < n; ++idx, ++x_it, ++y_it) {
    if (!IsFinitePoint(*x_it, *y_it)) continue;

    double az = std::atan2(*y_it, *x_it);
    if (az < 0.0) az += 2.0 * M_PI;

    // Phase unwrapping
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

    azs_.min_az = std::min(azs_.min_az, u_az);
    azs_.max_az = std::max(azs_.max_az, u_az);
  }
}

double DeskewAlgorithm::InterpolateSpeed(double t) const {
  if (speed_queue_.Size() == 0) return 0.0;

  if (t <= speed_queue_.PeerFront().time) {
    return speed_queue_.PeerFront().speed;
  }

  if (t >= speed_queue_.PeerBack().time) {
    return speed_queue_.PeerBack().speed;
  }

  size_t i = 0;
  while (i + 1 < speed_queue_.Size() && speed_queue_[i + 1].time < t) {
    ++i;
  }

  const auto& s0 = speed_queue_[i];
  const auto& s1 = speed_queue_[i + 1];

  const double dt = s1.time - s0.time;
  if (dt <= prms_.min_time_epsilon) return s0.speed;

  double a = (t - s0.time) / dt;
  a = std::clamp(a, 0.0, 1.0);

  return (1.0 - a) * s0.speed + a * s1.speed;
}

std::optional<sensor_msgs::msg::PointCloud2>
DeskewAlgorithm::ProcessCloudsQueue(const Eigen::Matrix3d& R_lidar_body) {
  if (imu_queue_.Size() < 2 || lidar_queue_.Size() < 1) {
    return {};
  }
  sensor_msgs::msg::PointCloud2 out;
  size_t n = 0;
  double scan_time = 0;
  double scan_start_time = 0.0;
  double scan_end_time = 0.0;
  const double imu_start_time = imu_queue_.PeerFront().time;
  const double imu_last_time = imu_queue_.PeerBack().time;
  bool found_cloud = false;
  while (lidar_queue_.Size() > 0) {
    const sensor_msgs::msg::PointCloud2& cloud = *lidar_queue_.PeerFront();
    scan_time = rclcpp::Time(cloud.header.stamp).seconds();
    scan_start_time = prms_.stamp_is_scan_end_ ? scan_time - prms_.scan_period_
                                               : scan_time;
    scan_end_time = prms_.stamp_is_scan_end_ ? scan_time
                                             : scan_time + prms_.scan_period_;
    n = static_cast<size_t>(cloud.width) * static_cast<size_t>(cloud.height);
    if (n == 0) {
      lidar_queue_.PopFront();
      RCLCPP_DEBUG(logger_, "Popped empty lidar cloud");
      continue;
    }

    UnwrapAzimuthParams(cloud);

    if (scan_end_time > imu_last_time) {
      RCLCPP_DEBUG(logger_,
                   "Waiting for future IMU data. last imu=%.6f scan end=%.6f",
                   imu_last_time, scan_end_time);
      return {};
    }

    if (scan_start_time < imu_start_time) {
      lidar_queue_.PopFront();
      RCLCPP_DEBUG(logger_,
                   "Popped stale lidar data. first imu=%.6f scan start=%.6f",
                   imu_start_time, scan_start_time);
      continue;
    }

    out = cloud;
    found_cloud = true;
    break;
  }

  if (!found_cloud) {
    return {};
  }

  // double az_span = azs_.max_az - azs_.min_az;
  if (azs_.GetRange() <= prms_.azimuth_range_epsilon) {
    lidar_queue_.PopFront();
    RCLCPP_WARN(logger_, "Invalid azimuth span, skipping deskew");
    return {};
  }

  if (speed_queue_.Size() < 1) {
    RCLCPP_DEBUG(logger_, "Waiting for speed data");
    return {};
  }

  if (speed_queue_.PeerBack().time < scan_end_time) {
    RCLCPP_DEBUG(
        logger_,
        "Waiting for future speed data. last speed=%.6f scan end=%.6f",
        speed_queue_.PeerBack().time, scan_end_time);
    return {};
  }

  lidar_queue_.PopFront();

  sensor_msgs::PointCloud2Iterator<float> x_it(out, "x");
  sensor_msgs::PointCloud2Iterator<float> y_it(out, "y");

  size_t imu_idx = 0;
  for (size_t idx = 0; idx < n; ++idx, ++x_it, ++y_it) {
    const float x = *x_it;
    const float y = *y_it;

    if (!IsFinitePoint(x, y)) {
      // RCLCPP_WARN(logger_, "Infinite point");
      continue;
    }
    double relative_time = azs_.GetRelativeTime(idx);
    if (!std::isfinite(relative_time)) {
      continue;
    }

    if (relative_time < -prms_.relative_time_tolerance ||
        relative_time > 1.0 + prms_.relative_time_tolerance) {
      continue;
    }
    double point_time = PointTimeFromIndex(relative_time, scan_time);

    const double signed_dt = point_time - scan_time;
    const double abs_dt = std::abs(signed_dt);

    if (abs_dt > prms_.scan_period_ + prms_.min_time_epsilon) {
      RCLCPP_WARN_THROTTLE(logger_, clock_, prms_.log_throttle,
                           "Bad deskew_dt %.6f rel=%.3f az=%.3f range=%.3f",
                           signed_dt, relative_time, azs_.unwrapped_az[idx],
                           azs_.GetRange());
      continue;
    }

    while (imu_idx + 1 < imu_queue_.Size() &&
           imu_queue_[imu_idx + 1].time < point_time) {
      imu_idx++;
    }

    if (imu_idx + 1 >= imu_queue_.Size()) {
      RCLCPP_WARN(logger_, "Queue index is out of range: %d",
                  static_cast<int>(imu_idx));
      return {};
    }

    const auto& imu0 = imu_queue_[imu_idx];
    const auto& imu1 = imu_queue_[imu_idx + 1];

    double imu_dt = imu1.time - imu0.time;
    if (imu_dt <= prms_.min_time_epsilon) {
      RCLCPP_WARN(logger_, "Too little imu_dt: %.6f", imu_dt);
      continue;
    }

    double alpha = (point_time - imu0.time) / imu_dt;
    alpha = std::clamp(alpha, 0.0, 1.0);

    double rate = (1.0 - alpha) * imu0.rate + alpha * imu1.rate;

    double dyaw = -rate * signed_dt;

    const double cos_yaw = std::cos(dyaw);
    const double sin_yaw = std::sin(dyaw);

    double speed = InterpolateSpeed(point_time);

    Eigen::Vector3d v_body(speed, 0.0, 0.0);
    Eigen::Vector3d v_lidar = R_lidar_body * v_body;

    double dx = v_lidar.x() * signed_dt;
    double dy = v_lidar.y() * signed_dt;

    float rx = cos_yaw * x - sin_yaw * y + dx;
    float ry = sin_yaw * x + cos_yaw * y + dy;

    *x_it = rx;
    *y_it = ry;
    if (prms_.debug) {
      RCLCPP_INFO_THROTTLE(
          logger_, clock_, prms_.log_throttle,
          "idx=%zu t=%.6f dt=%.4f alpha=%.2f rate=%.3f dyaw=%.4f", idx,
          point_time, signed_dt, alpha, rate, dyaw);
    }
  }
  return out;
}
