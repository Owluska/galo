#pragma once
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <deque>
#include <limits>
#include <memory>
#include <optional>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"

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
};

template <typename T>
class FiniteDeque {
 public:
  FiniteDeque() : size_(0) {}
  void Resize(int s) { size_ = static_cast<size_t>(s); }
  void Update(const T& item) {
    queue_.push_back(item);
    while (queue_.size() > size_) {
      queue_.pop_front();
    }
  }
  size_t Size() const { return queue_.size(); }

  T PeerFront() const { return queue_.front(); }
  T PeerBack() const { return queue_.back(); }

  void PopFront() { queue_.pop_front(); }

  const T& operator[](size_t index) const { return queue_[index]; }
  T& operator[](size_t index) { return queue_[index]; }

 private:
  size_t size_;
  std::deque<T> queue_;
};

class DeskewAlgorithm {
 private:
  FiniteDeque<YawRateStamped> imu_queue_;
  FiniteDeque<sensor_msgs::msg::PointCloud2::SharedPtr> lidar_queue_;
  DeskewParams prms_;
  rclcpp::Logger logger_;
  rclcpp::Clock clock_;
  PointCloudAzimuthPrms azs_;
  double PointTimeFromIndex(double relative_time,
                            double scan_header_time) const;

  bool isFinitePoint(float x, float y, float z = .0f) const;

  void UnwrapAzimuthParams(const sensor_msgs::msg::PointCloud2& msg);

 public:
  DeskewAlgorithm(const DeskewParams& params, const rclcpp::Logger& logger,
                  const rclcpp::Clock& clock);

  void UpdateImuQueue(double yaw_rate, double time) {
    YawRateStamped rate;
    rate.rate = yaw_rate;
    rate.time = time;
    imu_queue_.Update(rate);
  }

  void UpdateLidarQueue(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    lidar_queue_.Update(msg);
  }
  std::optional<sensor_msgs::msg::PointCloud2> ProcessCloudsQueue();
};
