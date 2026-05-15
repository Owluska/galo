#include "ground_aware_lidar_odometry/frontend.hpp"

GaloFrontend::GaloFrontend(
    const GroundSegmentationParams& segmentation_params,
    const GroundPatchParams& ground_patch_params,
    const PlanarRegistrationParams& planar_registration_params,
    const rclcpp::Logger& logger, const rclcpp::Clock& clock)
    : segmentation_(segmentation_params),
      ground_patches_extractor_(ground_patch_params),
      planar_registration_(planar_registration_params, logger, clock) {}

FrontendResult GaloFrontend::Extract(const CloudMsg& cloud,
                                     bool include_debug_outputs) {
  FrontendResult result;
  result.features.cloud = cloud;
  result.features.lidar_time = rclcpp::Time(cloud.header.stamp).seconds();

  SegmentationResult segmentation_result;
  {
    TimeMeasurments_t meas("segmentation");
    segmentation_result = segmentation_.Classify(cloud);
    meas.SetEnd();
    result.measurements.push_back(meas);
  }

  {
    TimeMeasurments_t meas("objects_extraction");
    result.features.planar_points =
        planar_registration_.ExtractPoints(cloud, segmentation_result.labels);
    if (!result.features.planar_points.empty()) {
      result.features.planar_points =
          planar_registration_.Filter(result.features.planar_points);
    }
    meas.SetEnd();
    result.measurements.push_back(meas);
  }

  {
    TimeMeasurments_t meas("ground_extraction");
    result.features.ground_patches =
        ground_patches_extractor_.Extract(cloud, segmentation_result.labels);
    meas.SetEnd();
    result.measurements.push_back(meas);
  }

  if (include_debug_outputs) {
    result.colored_cloud = segmentation_.MakeColoredCloud(segmentation_result);
    result.ground_markers =
        ground_patches_extractor_.MakeGroundPatchMarkers(cloud.header);
  }

  return result;
}
