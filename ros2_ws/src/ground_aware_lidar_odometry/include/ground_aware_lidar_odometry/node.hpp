#pragma once
#include <deque>
#include <memory>
#include <thread>

#include "ground_aware_lidar_odometry/deskew.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

class GALONode : public rclcpp::Node {
 public:
  GALONode();

 private:
  DeskewParams deskew_prms_;
  std::mutex mut_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr lidar_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr deskew_cld_pub_;

  std::shared_ptr<DeskewAlgorithm> deskew_algo_;

  void LidarCb(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
  void ImuCb(const sensor_msgs::msg::Imu::SharedPtr msg);
};