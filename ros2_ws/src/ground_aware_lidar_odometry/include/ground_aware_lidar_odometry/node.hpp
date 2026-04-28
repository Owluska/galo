#pragma once
#include <deque>
#include <memory>
#include <thread>
#include <tuple>

#include "geometry_msgs/msg/point.hpp"
#include "ground_aware_lidar_odometry/deskew.hpp"
#include "ground_aware_lidar_odometry/segmentation.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

struct GALONodeParams {
  int debug = 1;
};

class GALONode : public rclcpp::Node {
 public:
  GALONode(const GALONodeParams& node_params);

 private:
  GALONodeParams node_params_;
  DeskewParams deskew_prms_;
  GroundSegmentationParams segementation_params_;
  GroundPatchParams ground_patch_params_;
  GroundRegistrationParams ground_registration_params_;
  std::mutex mut_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr lidar_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr deskew_cld_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr colored_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr
      ground_patches_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr translation_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr eulers_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr imu_eulers_pub_;
  Eigen::Quaterniond imu_q_prev_;
  Eigen::Matrix3d R_imu_delta;
  DeskewAlgorithm deskew_algo_;
  Segmentation segmentation_;
  GroundPatchExtractor ground_patches_extractor_;
  GroundRegistration ground_registration_;
  std::vector<TimeMeasurments_t> time_measurments;
  std::vector<GroundPatch> ground_map_;

  void LidarCb(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
  void ImuCb(const sensor_msgs::msg::Imu::SharedPtr msg);
  void PrintTimeMeasurments(const std::vector<TimeMeasurments_t>& measurments);
  std::tuple<double, double, double> EulersFromMatrixSimple(
      const Eigen::Matrix3d& R);
};