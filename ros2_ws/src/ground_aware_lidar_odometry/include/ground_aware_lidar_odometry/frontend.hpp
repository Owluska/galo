#pragma once

#include <optional>
#include <vector>

#include "ground_aware_lidar_odometry/segmentation.hpp"

struct FrameFeatures {
  std_msgs::msg::Header header;
  std::vector<PlanarLine> planar_lines;
  std::vector<GroundPatch> ground_patches;
  double lidar_time = 0.0;
};

struct FrontendResult {
  FrameFeatures features;
  std::vector<TimeMeasurments_t> measurements;
  std::optional<sensor_msgs::msg::PointCloud2> colored_cloud;
  std::optional<visualization_msgs::msg::MarkerArray> ground_markers;
};

class GaloFrontend {
 public:
  GaloFrontend(const SegmentationParams& segmentation_params,
               const GroundPatchParams& ground_patch_params,
               const PlanarRegistrationParams& planar_registration_params,
               const rclcpp::Logger& logger, const rclcpp::Clock& clock);

  FrontendResult Extract(const CloudMsg& cloud, bool include_debug_outputs);

 private:
  Segmentation segmentation_;
  GroundPatchExtractor ground_patches_extractor_;
  PlanarRegistration planar_registration_;
};
