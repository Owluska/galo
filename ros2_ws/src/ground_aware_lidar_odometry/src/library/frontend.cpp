#include "ground_aware_lidar_odometry/frontend.hpp"

GaloFrontend::GaloFrontend(
    const SegmentationParams& segmentation_params,
    const GroundPatchParams& ground_patch_params,
    const PlanarRegistrationParams& planar_registration_params,
    const rclcpp::Logger& logger, const rclcpp::Clock& clock)
    : segmentation_(segmentation_params),
      ground_patches_extractor_(ground_patch_params),
      planar_registration_(planar_registration_params, logger, clock) {}

FrontendResult GaloFrontend::Extract(const CloudMsg& cloud,
                                     bool include_debug_outputs) {
  FrontendResult result;
  result.features.header = cloud.header;
  result.features.lidar_time = rclcpp::Time(cloud.header.stamp).seconds();

  SegmentationResult ground_segmentation;
  {
    TimeMeasurments_t meas("ground_segmentation");
    ground_segmentation = segmentation_.SegmentGround(cloud);
    meas.SetEnd();
    result.measurements.push_back(meas);
  }

  {
    TimeMeasurments_t meas("planar_line_extraction");
    const auto planar_points =
        planar_registration_.ExtractPoints(cloud, ground_segmentation.labels);
    result.features.planar_lines = planar_registration_.ExtractLines(
        planar_points, result.features.lidar_time);
    meas.SetEnd();
    result.measurements.push_back(meas);
  }

  {
    TimeMeasurments_t meas("ground_extraction");
    result.features.ground_patches =
        ground_patches_extractor_.Extract(cloud, ground_segmentation.labels);
    meas.SetEnd();
    result.measurements.push_back(meas);
  }

  if (include_debug_outputs) {
    result.colored_cloud =
        segmentation_.MakeColoredCloud(cloud, ground_segmentation);
    result.ground_markers =
        ground_patches_extractor_.MakeGroundPatchMarkers(cloud.header);
  }

  return result;
}
