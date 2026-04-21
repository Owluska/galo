#pragma once
#include <memory>

#include "ground_aware_lidar_odometry/deskew.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "user_msgs/msg/pure_state.hpp"

class GALONode : public rclcpp::Node {
 public:
  GALONode();

 private:
  DeskewParams deskew_prms_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr lidar_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<user_msgs::msg::PureState>::SharedPtr pure_state_sub_;

  std::shared_ptr<DeskewAlgorithm> deskew_algo_;
  double last_speed_;

  void LidarCb(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
  void ImuCb(const sensor_msgs::msg::Imu::SharedPtr msg);
  void PureStateCb(const user_msgs::msg::PureState::SharedPtr msg);
};