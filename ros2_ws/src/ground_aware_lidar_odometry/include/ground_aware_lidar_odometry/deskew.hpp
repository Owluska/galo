#pragma once
#include <math.h>

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <deque>
#include <limits>
#include <memory>
#include <optional>
#include <vector>

#include "ground_aware_lidar_odometry/utils.hpp"

struct YawRateStamped {
  double time;
  double rate;
};

struct PointCloudAzimuthPrms {
  double min_az = std::numeric_limits<double>::max();
  double max_az = std::numeric_limits<double>::lowest();

  double offset = 0.0;
  double last_az = -1.0;  // -1 is a safe flag since az is always in [0, 2PI]
  std::vector<double> unwrapped_az;
  void Reset(size_t n) {
    unwrapped_az.assign(n, std::numeric_limits<double>::quiet_NaN());
    min_az = std::numeric_limits<double>::max();
    max_az = std::numeric_limits<double>::lowest();

    offset = 0.0;
    last_az = -1.0;  // -1 is a safe flag since az is always in [0, 2PI]
  }

  double GetRelativeTime(size_t idx) {
    // 1. Counter-Clockwise (or Chronological)
    // 2. Clockwise (or Reverse Chronological)
    // As angle decreases, time increases.
    // So max_az is t=0, and min_az is t=1.
    const double az = unwrapped_az[idx];
    if (!std::isfinite(az)) {
      return std::numeric_limits<double>::quiet_NaN();
    }
    const double total_range = max_az - min_az;
    if (total_range <= 1e-6) {
      return std::numeric_limits<double>::quiet_NaN();
    }
    // Option A: CCW
    double rel_ccw = (az - min_az) / total_range;

    // Option B: CW
    double rel_cw = (max_az - az) / total_range;

    // Choose the one that lies in [0,1]
    if (rel_ccw >= 0.0 && rel_ccw <= 1.0) return rel_ccw;

    return rel_cw;
  }

  double GetRange() { return max_az - min_az; }
};

struct DeskewParams {
  int imu_queue_size = 100;
  int lidar_queue_size = 10;
  double scan_period_ = 0.1;
  bool stamp_is_scan_end_ = true;
  int log_throttle = 2000;  // ms
  int debug = 0;
};

class DeskewAlgorithm {
 private:
  FiniteDeque<YawRateStamped> imu_queue_;
  FiniteDeque<CloudMsg::SharedPtr> lidar_queue_;
  DeskewParams prms_;
  rclcpp::Logger logger_;
  rclcpp::Clock clock_;
  PointCloudAzimuthPrms azs_;
  double PointTimeFromIndex(double relative_time,
                            double scan_header_time) const;

  void UnwrapAzimuthParams(const CloudMsg& msg);

 public:
  DeskewAlgorithm(const DeskewParams& params, const rclcpp::Logger& logger,
                  const rclcpp::Clock& clock);

  void UpdateImuQueue(double yaw_rate, double time) {
    YawRateStamped rate;
    rate.rate = yaw_rate;
    rate.time = time;
    imu_queue_.Update(rate);
  }

  void UpdateLidarQueue(const CloudMsg::SharedPtr msg) {
    lidar_queue_.Update(msg);
  }
  std::optional<CloudMsg> ProcessCloudsQueue();
};
