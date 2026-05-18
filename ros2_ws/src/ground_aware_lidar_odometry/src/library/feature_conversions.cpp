#include "ground_aware_lidar_odometry/feature_conversions.hpp"

namespace ground_aware_lidar_odometry {

msg::FrameFeatures ToMsg(const ::FrameFeatures& features) {
  msg::FrameFeatures msg;
  msg.header = features.header;

  msg.planar_points.reserve(features.planar_points.size());
  for (const auto& point : features.planar_points) {
    ground_aware_lidar_odometry::msg::Point2D point_msg;
    point_msg.x = point.x();
    point_msg.y = point.y();
    msg.planar_points.push_back(point_msg);
  }

  msg.ground_patches.reserve(features.ground_patches.size());
  for (const auto& patch : features.ground_patches) {
    ground_aware_lidar_odometry::msg::GroundPatch patch_msg;
    patch_msg.centroid.x = patch.centroid.x();
    patch_msg.centroid.y = patch.centroid.y();
    patch_msg.centroid.z = patch.centroid.z();
    patch_msg.normal.x = patch.normal.x();
    patch_msg.normal.y = patch.normal.y();
    patch_msg.normal.z = patch.normal.z();
    for (int r = 0; r < 3; ++r) {
      for (int c = 0; c < 3; ++c) {
        patch_msg.covariance[static_cast<size_t>(3 * r + c)] =
            patch.covariance(r, c);
      }
    }
    patch_msg.time = patch.time;
    patch_msg.surface_variation = patch.surface_variation;
    patch_msg.weight = patch.weight;
    patch_msg.support = patch.support;
    patch_msg.cell_x = patch.key.x;
    patch_msg.cell_y = patch.key.y;
    msg.ground_patches.push_back(patch_msg);
  }

  return msg;
}

::FrameFeatures FromMsg(const msg::FrameFeatures& msg) {
  ::FrameFeatures features;
  features.header = msg.header;
  features.lidar_time = rclcpp::Time(msg.header.stamp).seconds();

  features.planar_points.reserve(msg.planar_points.size());
  for (const auto& point_msg : msg.planar_points) {
    features.planar_points.emplace_back(point_msg.x, point_msg.y);
  }

  features.ground_patches.reserve(msg.ground_patches.size());
  for (const auto& patch_msg : msg.ground_patches) {
    GroundPatch patch;
    patch.centroid = Eigen::Vector3d(
        patch_msg.centroid.x, patch_msg.centroid.y, patch_msg.centroid.z);
    patch.normal = Eigen::Vector3d(
        patch_msg.normal.x, patch_msg.normal.y, patch_msg.normal.z);
    for (int r = 0; r < 3; ++r) {
      for (int c = 0; c < 3; ++c) {
        patch.covariance(r, c) =
            patch_msg.covariance[static_cast<size_t>(3 * r + c)];
      }
    }
    patch.time = patch_msg.time;
    patch.surface_variation = patch_msg.surface_variation;
    patch.weight = patch_msg.weight;
    patch.support = patch_msg.support;
    patch.key.x = patch_msg.cell_x;
    patch.key.y = patch_msg.cell_y;
    features.ground_patches.push_back(patch);
  }

  return features;
}

}  // namespace ground_aware_lidar_odometry
