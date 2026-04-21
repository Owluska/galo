#pragma once
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <deque>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"

struct PlanarMotion {
  double dx;
  double dy;
  double dyaw;
};

struct YawRateStamped {
  double time;
  double rate;
  double last_speed;
};

struct DeskewParams {
  int imu_queue_size = 100;
  double scan_period_ = 0.1;
  bool stamp_is_scan_end_ = false;
};

class DeskewAlgorithm {
 private:
  std::deque<YawRateStamped> imu_queue_;
  DeskewParams prms_;
  rclcpp::Logger logger_;
  double PointTimeFromIndex(size_t index, size_t n_points,
                            double scan_header_time) const;
  PlanarMotion IntegratePlanarMotion(double v, double yaw_rate,
                                     double dt) const;

  bool isFinitePoint(float x, float y, float z = .0f) const;

 public:
  DeskewAlgorithm(const DeskewParams& params, const rclcpp::Logger& logger);

  void UpdateQueue(double last_speed, double yaw_rate, double time);

  sensor_msgs::msg::PointCloud2 ProcessCloud(
      const sensor_msgs::msg::PointCloud2::SharedPtr msg) const;
};
