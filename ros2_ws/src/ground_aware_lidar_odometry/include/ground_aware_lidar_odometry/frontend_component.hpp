#pragma once

#include <chrono>
#include <string>

#include "ground_aware_lidar_odometry/frontend.hpp"
#include "ground_aware_lidar_odometry/msg/frame_features.hpp"
#include "rclcpp/callback_group.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

namespace ground_aware_lidar_odometry {

class GaloFrontendComponent : public rclcpp::Node {
 public:
  explicit GaloFrontendComponent(const rclcpp::NodeOptions& options);

 private:
  struct Params {
    int debug = 1;
    double elapsed_time_thresh = 50.0;
    int log_throttle = 1000;
    std::string deskewed_cloud_topic = "/GALO/deskewed_cloud";
    std::string frame_features_topic = "/GALO/frame_features";
    std::string colored_cloud_topic = "/GALO/colored_cloud";
    std::string ground_patches_topic = "/GALO/ground_patch_normals";
  };

  Params params_;
  SegmentationParams segmentation_params_;
  GroundPatchParams ground_patch_params_;
  PlanarRegistrationParams planar_registration_params_;
  GaloFrontend frontend_;
  std::chrono::steady_clock::time_point last_timing_info_time_{};

  rclcpp::CallbackGroup::SharedPtr callback_group_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Publisher<ground_aware_lidar_odometry::msg::FrameFeatures>::SharedPtr
      features_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr colored_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr
      ground_patches_pub_;

  void CloudCb(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
  void PrintTimeMeasurements(const std::vector<TimeMeasurments_t>& measurements);

  static Params LoadParams(rclcpp::Node& node);
  static SegmentationParams LoadSegmentationParams(
      rclcpp::Node& node);
  static GroundPatchParams LoadGroundPatchParams(rclcpp::Node& node);
  static PlanarRegistrationParams LoadPlanarRegistrationParams(
      rclcpp::Node& node);
};

}  // namespace ground_aware_lidar_odometry
