#include "ground_aware_lidar_odometry/node.hpp"

GALONodeParams GALONode::LoadNodeParams(rclcpp::Node& node) {
  GALONodeParams p;

  p.debug = DeclareAndGet<int>(node, "node.debug", p.debug);
  p.max_ground_map_frames = DeclareAndGet<int>(
      node, "node.max_ground_map_frames", p.max_ground_map_frames);
  p.max_planar_map_frames = DeclareAndGet<int>(
      node, "node.max_planar_map_frames", p.max_planar_map_frames);

  p.gt_cov_.x_precision = DeclareAndGet<double>(
      node, "gnss_covariance.x_precision", p.gt_cov_.x_precision);
  p.gt_cov_.y_precision = DeclareAndGet<double>(
      node, "gnss_covariance.y_precision", p.gt_cov_.y_precision);
  p.gt_cov_.z_precision = DeclareAndGet<double>(
      node, "gnss_covariance.z_precision", p.gt_cov_.z_precision);
  p.gt_cov_.roll_precision = DeclareAndGet<double>(
      node, "gnss_covariance.roll_precision", p.gt_cov_.roll_precision);
  p.gt_cov_.pitch_precision = DeclareAndGet<double>(
      node, "gnss_covariance.pitch_precision", p.gt_cov_.pitch_precision);
  p.gt_cov_.yaw_precision = DeclareAndGet<double>(
      node, "gnss_covariance.yaw_precision", p.gt_cov_.yaw_precision);

  p.est_cov_.base_xy = DeclareAndGet<double>(node, "lidar_covariance.base_xy",
                                             p.est_cov_.base_xy);
  p.est_cov_.z =
      DeclareAndGet<double>(node, "lidar_covariance.z", p.est_cov_.z);
  p.est_cov_.scale =
      DeclareAndGet<double>(node, "lidar_covariance.scale", p.est_cov_.scale);
  p.est_cov_.roll =
      DeclareAndGet<double>(node, "lidar_covariance.roll", p.est_cov_.roll);
  p.est_cov_.pitch =
      DeclareAndGet<double>(node, "lidar_covariance.pitch", p.est_cov_.pitch);

  return p;
}

DeskewParams GALONode::LoadDeskewParams(rclcpp::Node& node) {
  DeskewParams p;

  p.imu_queue_size =
      DeclareAndGet<int>(node, "deskew.imu_queue_size", p.imu_queue_size);
  p.lidar_queue_size =
      DeclareAndGet<int>(node, "deskew.lidar_queue_size", p.lidar_queue_size);
  p.scan_period_ =
      DeclareAndGet<double>(node, "deskew.scan_period", p.scan_period_);
  p.stamp_is_scan_end_ = DeclareAndGet<bool>(node, "deskew.stamp_is_scan_end",
                                             p.stamp_is_scan_end_);
  p.log_throttle =
      DeclareAndGet<int>(node, "deskew.log_throttle", p.log_throttle);
  p.debug = DeclareAndGet<int>(node, "deskew.debug", p.debug);

  return p;
}

GroundSegmentationParams GALONode::LoadGroundSegmentationParams(
    rclcpp::Node& node) {
  GroundSegmentationParams p;

  p.cell_size =
      DeclareAndGet<double>(node, "ground_segmentation.cell_size", p.cell_size);
  p.min_range =
      DeclareAndGet<double>(node, "ground_segmentation.min_range", p.min_range);
  p.max_range =
      DeclareAndGet<double>(node, "ground_segmentation.max_range", p.max_range);
  p.ground_height_threshold =
      DeclareAndGet<double>(node, "ground_segmentation.ground_height_threshold",
                            p.ground_height_threshold);
  p.min_points_per_cell = DeclareAndGet<int>(
      node, "ground_segmentation.min_points_per_cell", p.min_points_per_cell);
  p.ground_min_height = DeclareAndGet<double>(
      node, "ground_segmentation.ground_min_height", p.ground_min_height);
  return p;
}

GroundPatchParams GALONode::LoadGroundPatchParams(rclcpp::Node& node) {
  GroundPatchParams p;

  p.cell_size =
      DeclareAndGet<double>(node, "ground_patch.cell_size", p.cell_size);
  p.min_points =
      DeclareAndGet<int>(node, "ground_patch.min_points", p.min_points);
  p.max_thickness = DeclareAndGet<double>(node, "ground_patch.max_thickness",
                                          p.max_thickness);
  p.min_normal_z =
      DeclareAndGet<double>(node, "ground_patch.min_normal_z", p.min_normal_z);
  p.max_surface_variation = DeclareAndGet<double>(
      node, "ground_patch.max_surface_variation", p.max_surface_variation);

  return p;
}

GroundRegistrationParams GALONode::LoadGroundRegistrationParams(
    rclcpp::Node& node) {
  GroundRegistrationParams p;

  p.max_match_distance = DeclareAndGet<double>(
      node, "ground_registration.max_match_distance", p.max_match_distance);
  p.min_normal_dot = DeclareAndGet<double>(
      node, "ground_registration.min_normal_dot", p.min_normal_dot);
  p.max_iterations = DeclareAndGet<int>(
      node, "ground_registration.max_iterations", p.max_iterations);
  p.min_matches = DeclareAndGet<int>(node, "ground_registration.min_matches",
                                     p.min_matches);

  p.max_dz =
      DeclareAndGet<double>(node, "ground_registration.max_dz", p.max_dz);
  p.max_roll =
      DeclareAndGet<double>(node, "ground_registration.max_roll", p.max_roll);
  p.max_pitch =
      DeclareAndGet<double>(node, "ground_registration.max_pitch", p.max_pitch);

  p.damping_z =
      DeclareAndGet<double>(node, "ground_registration.damping_z", p.damping_z);
  p.damping_roll = DeclareAndGet<double>(
      node, "ground_registration.damping_roll", p.damping_roll);
  p.damping_pitch = DeclareAndGet<double>(
      node, "ground_registration.damping_pitch", p.damping_pitch);

  p.min_x_span_for_pitch = DeclareAndGet<double>(
      node, "ground_registration.min_x_span_for_pitch", p.min_x_span_for_pitch);
  p.min_y_span_for_roll = DeclareAndGet<double>(
      node, "ground_registration.min_y_span_for_roll", p.min_y_span_for_roll);

  p.imu_roll_weight = DeclareAndGet<double>(
      node, "ground_registration.imu_roll_weight", p.imu_roll_weight);
  p.imu_pitch_weight = DeclareAndGet<double>(
      node, "ground_registration.imu_pitch_weight", p.imu_pitch_weight);

  p.use_imu_prior = DeclareAndGet<bool>(
      node, "ground_registration.use_imu_prior", p.use_imu_prior);

  p.log_throttle = DeclareAndGet<int>(node, "ground_registration.log_throttle",
                                      p.log_throttle);
  p.max_match_z_difference =
      DeclareAndGet<double>(node, "ground_registration.max_match_z_difference",
                            p.max_match_z_difference);

  return p;
}

PlanarRegistrationParams GALONode::LoadPlanarRegistrationParams(
    rclcpp::Node& node) {
  PlanarRegistrationParams p;

  p.voxel_size = DeclareAndGet<double>(node, "planar_registration.voxel_size",
                                       p.voxel_size);
  p.max_match_distance = DeclareAndGet<double>(
      node, "planar_registration.max_match_distance", p.max_match_distance);
  p.min_points_per_voxel = DeclareAndGet<int>(
      node, "planar_registration.min_points_per_voxel", p.min_points_per_voxel);
  p.max_iterations = DeclareAndGet<int>(
      node, "planar_registration.max_iterations", p.max_iterations);
  p.min_matches = DeclareAndGet<int>(node, "planar_registration.min_matches",
                                     p.min_matches);

  p.damping =
      DeclareAndGet<double>(node, "planar_registration.damping", p.damping);

  p.max_dx =
      DeclareAndGet<double>(node, "planar_registration.max_dx", p.max_dx);
  p.max_dy =
      DeclareAndGet<double>(node, "planar_registration.max_dy", p.max_dy);
  p.max_dyaw =
      DeclareAndGet<double>(node, "planar_registration.max_dyaw", p.max_dyaw);

  p.max_dx_step = DeclareAndGet<double>(node, "planar_registration.max_dx_step",
                                        p.max_dx_step);
  p.max_dy_step = DeclareAndGet<double>(node, "planar_registration.max_dy_step",
                                        p.max_dy_step);
  p.max_dyaw_step = DeclareAndGet<double>(
      node, "planar_registration.max_dyaw_step", p.max_dyaw_step);

  p.convergence_eps = DeclareAndGet<double>(
      node, "planar_registration.convergence_eps", p.convergence_eps);

  return p;
}
