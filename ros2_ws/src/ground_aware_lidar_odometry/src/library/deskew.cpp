#include "ground_aware_lidar_odometry/deskew.hpp"

#include <cstring>
#include <omp.h>

namespace {

std::optional<size_t> FieldOffset(const sensor_msgs::msg::PointCloud2& msg,
                                  const std::string& field_name) {
  for (const auto& field : msg.fields) {
    if (field.name == field_name) {
      return static_cast<size_t>(field.offset);
    }
  }
  return std::nullopt;
}

template <typename T>
std::vector<T> CopyQueue(const FiniteDeque<T>& queue) {
  std::vector<T> out;
  out.reserve(queue.Size());
  for (size_t i = 0; i < queue.Size(); ++i) {
    out.push_back(queue[i]);
  }
  return out;
}

template <typename T>
size_t FindInterval(const std::vector<T>& samples, double t) {
  if (samples.size() < 2) return samples.size();
  size_t lo = 0;
  size_t hi = samples.size() - 1;
  while (lo + 1 < hi) {
    const size_t mid = lo + (hi - lo) / 2;
    if (samples[mid].time < t) {
      lo = mid;
    } else {
      hi = mid;
    }
  }
  return lo;
}

double InterpolateSpeed(const std::vector<SpeedStamped>& samples, double t,
                        double min_time_epsilon) {
  if (samples.empty()) return 0.0;
  if (t <= samples.front().time) return samples.front().speed;
  if (t >= samples.back().time) return samples.back().speed;

  const size_t i = FindInterval(samples, t);
  if (i + 1 >= samples.size()) return samples.back().speed;

  const auto& s0 = samples[i];
  const auto& s1 = samples[i + 1];
  const double dt = s1.time - s0.time;
  if (dt <= min_time_epsilon) return s0.speed;

  double a = (t - s0.time) / dt;
  a = std::clamp(a, 0.0, 1.0);
  return (1.0 - a) * s0.speed + a * s1.speed;
}

}  // namespace

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

std::optional<DeskewInput> DeskewAlgorithm::TakeReadyCloud() {
  if (imu_queue_.Size() < 2 || lidar_queue_.Size() < 1) {
    return {};
  }

  const double imu_start_time = imu_queue_.PeerFront().time;
  const double imu_last_time = imu_queue_.PeerBack().time;

  while (lidar_queue_.Size() > 0) {
    const sensor_msgs::msg::PointCloud2& cloud = *lidar_queue_.PeerFront();
    const double scan_time = rclcpp::Time(cloud.header.stamp).seconds();
    const double scan_start_time = prms_.stamp_is_scan_end_
                                       ? scan_time - prms_.scan_period_
                                       : scan_time;
    const double scan_end_time = prms_.stamp_is_scan_end_
                                     ? scan_time
                                     : scan_time + prms_.scan_period_;
    const size_t n =
        static_cast<size_t>(cloud.width) * static_cast<size_t>(cloud.height);
    if (n == 0) {
      lidar_queue_.PopFront();
      RCLCPP_DEBUG(logger_, "Popped empty lidar cloud");
      continue;
    }

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

    if (speed_queue_.Size() < 1) {
      RCLCPP_DEBUG(logger_, "Waiting for speed data");
      return {};
    }

    const double speed_age = scan_end_time - speed_queue_.PeerBack().time;
    if (speed_age > prms_.max_speed_age) {
      RCLCPP_DEBUG(logger_,
                   "Waiting for recent speed data. last speed=%.6f scan end=%.6f age=%.3f max=%.3f",
                   speed_queue_.PeerBack().time, scan_end_time, speed_age,
                   prms_.max_speed_age);
      return {};
    }

    DeskewInput input;
    input.cloud = cloud;
    input.scan_time = scan_time;
    input.imu_samples = CopyQueue(imu_queue_);
    input.speed_samples = CopyQueue(speed_queue_);
    lidar_queue_.PopFront();
    return input;
  }

  return {};
}

std::optional<sensor_msgs::msg::PointCloud2> DeskewAlgorithm::DeskewCloud(
    const DeskewInput& input, const Eigen::Matrix3d& R_lidar_body) {
  sensor_msgs::msg::PointCloud2 out = input.cloud;
  const size_t n = static_cast<size_t>(out.width) * static_cast<size_t>(out.height);
  if (n == 0) return {};

  UnwrapAzimuthParams(out);

  if (azs_.GetRange() <= prms_.azimuth_range_epsilon) {
    RCLCPP_WARN(logger_, "Invalid azimuth span, skipping deskew");
    return {};
  }

  const auto x_offset = FieldOffset(out, "x");
  const auto y_offset = FieldOffset(out, "y");
  if (!x_offset || !y_offset) {
    RCLCPP_WARN(logger_, "PointCloud2 is missing x/y fields, skipping deskew");
    return {};
  }

  const Eigen::Vector3d lidar_forward = R_lidar_body.col(0);
  const uint32_t point_step = out.point_step;
  uint8_t* const data = out.data.data();
  const double scan_time = input.scan_time;
  const double scan_start = prms_.stamp_is_scan_end_
                                ? scan_time - prms_.scan_period_
                                : scan_time;
  const double relative_time_tolerance = prms_.relative_time_tolerance;
  const double scan_period = prms_.scan_period_;
  const double min_time_epsilon = prms_.min_time_epsilon;
  const auto& imu_samples = input.imu_samples;
  const auto& speed_samples = input.speed_samples;

  if (prms_.num_threads > 0) {
    omp_set_num_threads(prms_.num_threads);
  }

#pragma omp parallel for schedule(static)
  for (std::ptrdiff_t signed_idx = 0; signed_idx < static_cast<std::ptrdiff_t>(n);
       ++signed_idx) {
    const size_t idx = static_cast<size_t>(signed_idx);
    uint8_t* const point = data + idx * point_step;

    float x = 0.0f;
    float y = 0.0f;
    std::memcpy(&x, point + *x_offset, sizeof(float));
    std::memcpy(&y, point + *y_offset, sizeof(float));

    if (!IsFinitePoint(x, y)) continue;

    const double relative_time = azs_.GetRelativeTime(idx);
    if (!std::isfinite(relative_time)) continue;
    if (relative_time < -relative_time_tolerance ||
        relative_time > 1.0 + relative_time_tolerance) {
      continue;
    }

    const double point_time = scan_start + relative_time * scan_period;
    const double signed_dt = point_time - scan_time;
    if (std::abs(signed_dt) > scan_period + min_time_epsilon) continue;

    const size_t imu_idx = FindInterval(imu_samples, point_time);
    if (imu_idx + 1 >= imu_samples.size()) continue;

    const auto& imu0 = imu_samples[imu_idx];
    const auto& imu1 = imu_samples[imu_idx + 1];
    const double imu_dt = imu1.time - imu0.time;
    if (imu_dt <= min_time_epsilon) continue;

    double alpha = (point_time - imu0.time) / imu_dt;
    alpha = std::clamp(alpha, 0.0, 1.0);
    const double rate = (1.0 - alpha) * imu0.rate + alpha * imu1.rate;
    const double dyaw = -rate * signed_dt;

    const double cos_yaw = std::cos(dyaw);
    const double sin_yaw = std::sin(dyaw);
    const double speed = ::InterpolateSpeed(speed_samples, point_time,
                                            min_time_epsilon);

    const double dx = lidar_forward.x() * speed * signed_dt;
    const double dy = lidar_forward.y() * speed * signed_dt;

    const float rx = static_cast<float>(cos_yaw * x - sin_yaw * y + dx);
    const float ry = static_cast<float>(sin_yaw * x + cos_yaw * y + dy);

    std::memcpy(point + *x_offset, &rx, sizeof(float));
    std::memcpy(point + *y_offset, &ry, sizeof(float));
  }

  return out;
}

std::optional<sensor_msgs::msg::PointCloud2>
DeskewAlgorithm::ProcessCloudsQueue(const Eigen::Matrix3d& R_lidar_body) {
  std::optional<DeskewInput> input = TakeReadyCloud();
  if (!input) return {};
  return DeskewCloud(*input, R_lidar_body);
}
