#include "ground_aware_lidar_odometry/feature_conversions.hpp"

namespace ground_aware_lidar_odometry {

msg::FrameFeatures ToMsg(const ::FrameFeatures& features) {
  msg::FrameFeatures msg;
  msg.header = features.header;

  msg.planar_lines.reserve(features.planar_lines.size());
  for (const auto& line : features.planar_lines) {
    ground_aware_lidar_odometry::msg::PlanarLine line_msg;
    line_msg.center.x = line.center.x();
    line_msg.center.y = line.center.y();
    line_msg.direction.x = line.direction.x();
    line_msg.direction.y = line.direction.y();
    line_msg.normal.x = line.normal.x();
    line_msg.normal.y = line.normal.y();
    line_msg.z = line.z;
    line_msg.length = line.length;
    line_msg.fit_error = line.fit_error;
    line_msg.support = line.support;
    line_msg.time = line.time;
    msg.planar_lines.push_back(line_msg);
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

  features.planar_lines.reserve(msg.planar_lines.size());
  for (const auto& line_msg : msg.planar_lines) {
    PlanarLine line;
    line.center = Eigen::Vector2d(line_msg.center.x, line_msg.center.y);
    line.direction = Eigen::Vector2d(line_msg.direction.x, line_msg.direction.y);
    if (line.direction.squaredNorm() > 1e-12) line.direction.normalize();
    line.normal = Eigen::Vector2d(line_msg.normal.x, line_msg.normal.y);
    if (line.normal.squaredNorm() > 1e-12) line.normal.normalize();
    line.z = line_msg.z;
    line.length = line_msg.length;
    line.fit_error = line_msg.fit_error;
    line.support = line_msg.support;
    line.time = line_msg.time;
    features.planar_lines.push_back(line);
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
