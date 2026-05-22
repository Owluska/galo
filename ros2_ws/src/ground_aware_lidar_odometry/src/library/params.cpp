#include "ground_aware_lidar_odometry/node.hpp"

GaloOdometryParams GaloOdometryComponent::LoadNodeParams(rclcpp::Node& node) {
  GaloOdometryParams p;

  p.debug = DeclareAndGet<int>(node, "node.debug", p.debug);
  p.max_ground_map_frames = DeclareAndGet<int>(
      node, "node.max_ground_map_frames", p.max_ground_map_frames);
  p.elapsed_time_thresh = DeclareAndGet<double>(
      node, "node.elapsed_time_thresh", p.elapsed_time_thresh);
  p.max_planar_map_frames = DeclareAndGet<int>(
      node, "node.max_planar_map_frames", p.max_planar_map_frames);
  p.map_stale_threshold_ms = DeclareAndGet<double>(
      node, "node.map_stale_threshold_ms", p.map_stale_threshold_ms);
  p.merge_alpha_xy =
      DeclareAndGet<double>(node, "pose_merge.alpha_xy", p.merge_alpha_xy);
  p.merge_alpha_z =
      DeclareAndGet<double>(node, "pose_merge.alpha_z", p.merge_alpha_z);
  p.merge_alpha_rp =
      DeclareAndGet<double>(node, "pose_merge.alpha_rp", p.merge_alpha_rp);
  p.merge_alpha_yaw =
      DeclareAndGet<double>(node, "pose_merge.alpha_yaw", p.merge_alpha_yaw);
  p.fallback_alpha_xy = DeclareAndGet<double>(
      node, "pose_merge.fallback_alpha_xy", p.fallback_alpha_xy);
  p.fallback_alpha_z_valid_ground =
      DeclareAndGet<double>(node, "pose_merge.fallback_alpha_z_valid_ground",
                            p.fallback_alpha_z_valid_ground);
  p.imu_orientation_queue_size = DeclareAndGet<int>(
      node, "node.imu_orientation_queue_size", p.imu_orientation_queue_size);
  p.wheel_data_queue_size = DeclareAndGet<int>(
      node, "node.wheel_data_queue_size", p.wheel_data_queue_size);
  p.min_gnss_quality =
      DeclareAndGet<int>(node, "node.min_gnss_quality", p.min_gnss_quality);
  p.gnss_yaw_position_max_dt = DeclareAndGet<double>(
      node, "node.gnss_yaw_position_max_dt", p.gnss_yaw_position_max_dt);
  p.initialization_log_throttle = DeclareAndGet<int>(
      node, "node.initialization_log_throttle", p.initialization_log_throttle);
  p.registration_log_throttle = DeclareAndGet<int>(
      node, "node.registration_log_throttle", p.registration_log_throttle);
  p.min_ground_patches_for_map_update =
      DeclareAndGet<int>(node, "node.min_ground_patches_for_map_update",
                         p.min_ground_patches_for_map_update);
  p.pose_dt_min =
      DeclareAndGet<double>(node, "node.pose_dt_min", p.pose_dt_min);
  p.pose_dt_max =
      DeclareAndGet<double>(node, "node.pose_dt_max", p.pose_dt_max);
  p.gnss_correction_period_sec =
      DeclareAndGet<double>(node, "node.gnss_correction_period_sec",
                            p.gnss_correction_period_sec);

  p.imu_frame =
      DeclareAndGet<std::string>(node, "frames.imu_frame", p.imu_frame);
  p.lidar_frame =
      DeclareAndGet<std::string>(node, "frames.lidar_frame", p.lidar_frame);
  p.pos_antenna_frame = DeclareAndGet<std::string>(
      node, "frames.pos_antenna_frame", p.pos_antenna_frame);
  p.orientation_antenna_frame = DeclareAndGet<std::string>(
      node, "frames.orientation_antenna_frame", p.orientation_antenna_frame);
  p.map_frame =
      DeclareAndGet<std::string>(node, "frames.map_frame", p.map_frame);
  p.body_frame =
      DeclareAndGet<std::string>(node, "frames.body_frame", p.body_frame);
  p.gnss_map_frame = DeclareAndGet<std::string>(node, "frames.gnss_map_frame",
                                                p.gnss_map_frame);

  p.lidar_topic =
      DeclareAndGet<std::string>(node, "topics.lidar", p.lidar_topic);
  p.imu_topic = DeclareAndGet<std::string>(node, "topics.imu", p.imu_topic);
  p.gnss_topic = DeclareAndGet<std::string>(node, "topics.gnss", p.gnss_topic);
  p.gnss_orientation_topic = DeclareAndGet<std::string>(
      node, "topics.gnss_orientation", p.gnss_orientation_topic);
  p.pure_state_topic =
      DeclareAndGet<std::string>(node, "topics.pure_state", p.pure_state_topic);
  p.wheel_speed_topic = DeclareAndGet<std::string>(node, "topics.wheel_speed",
                                                   p.wheel_speed_topic);
  p.wheel_angle_topic = DeclareAndGet<std::string>(node, "topics.wheel_angle",
                                                   p.wheel_angle_topic);
  p.deskewed_cloud_topic = DeclareAndGet<std::string>(
      node, "topics.deskewed_cloud", p.deskewed_cloud_topic);
  p.frame_features_topic = DeclareAndGet<std::string>(
      node, "topics.frame_features", p.frame_features_topic);
  p.colored_cloud_topic = DeclareAndGet<std::string>(
      node, "topics.colored_cloud", p.colored_cloud_topic);
  p.ground_patches_topic = DeclareAndGet<std::string>(
      node, "topics.ground_patches", p.ground_patches_topic);
  p.translation_topic = DeclareAndGet<std::string>(node, "topics.translation",
                                                   p.translation_topic);
  p.gt_eulers_topic =
      DeclareAndGet<std::string>(node, "topics.gt_eulers", p.gt_eulers_topic);
  p.est_eulers_topic =
      DeclareAndGet<std::string>(node, "topics.est_eulers", p.est_eulers_topic);
  p.pure_state_eulers_topic = DeclareAndGet<std::string>(
      node, "topics.pure_state_eulers", p.pure_state_eulers_topic);
  p.true_pose_topic =
      DeclareAndGet<std::string>(node, "topics.true_pose", p.true_pose_topic);
  p.estimate_pose_topic = DeclareAndGet<std::string>(
      node, "topics.estimate_pose", p.estimate_pose_topic);
  p.speed_topic =
      DeclareAndGet<std::string>(node, "topics.speed", p.speed_topic);

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
  p.est_cov_.residual_norm = DeclareAndGet<double>(
      node, "lidar_covariance.residual_norm", p.est_cov_.residual_norm);
  p.est_cov_.residual_scale_min =
      DeclareAndGet<double>(node, "lidar_covariance.residual_scale_min",
                            p.est_cov_.residual_scale_min);
  p.est_cov_.residual_scale_max =
      DeclareAndGet<double>(node, "lidar_covariance.residual_scale_max",
                            p.est_cov_.residual_scale_max);
  p.est_cov_.match_count_norm = DeclareAndGet<double>(
      node, "lidar_covariance.match_count_norm", p.est_cov_.match_count_norm);
  p.est_cov_.match_count_scale_min =
      DeclareAndGet<double>(node, "lidar_covariance.match_count_scale_min",
                            p.est_cov_.match_count_scale_min);
  p.est_cov_.match_count_scale_max =
      DeclareAndGet<double>(node, "lidar_covariance.match_count_scale_max",
                            p.est_cov_.match_count_scale_max);
  p.est_cov_.yaw_base_deg = DeclareAndGet<double>(
      node, "lidar_covariance.yaw_base_deg", p.est_cov_.yaw_base_deg);

  return p;
}

GroundRegistrationParams GaloOdometryComponent::LoadGroundRegistrationParams(
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
  p.k_nearest_neighbors = DeclareAndGet<int>(
      node, "ground_registration.k_nearest_neighbors", p.k_nearest_neighbors);
  p.range_weight_coeff = DeclareAndGet<double>(
      node, "ground_registration.range_weight_coeff", p.range_weight_coeff);
  p.condition_lambda_floor =
      DeclareAndGet<double>(node, "ground_registration.condition_lambda_floor",
                            p.condition_lambda_floor);
  p.min_condition_eigenvalue = DeclareAndGet<double>(
      node, "ground_registration.min_condition_eigenvalue",
      p.min_condition_eigenvalue);
  p.max_condition_number = DeclareAndGet<double>(
      node, "ground_registration.max_condition_number", p.max_condition_number);
  p.convergence_eps = DeclareAndGet<double>(
      node, "ground_registration.convergence_eps", p.convergence_eps);
  p.early_stop_worsen_rel_tol = DeclareAndGet<double>(
      node, "ground_registration.early_stop_worsen_rel_tol",
      p.early_stop_worsen_rel_tol);
  p.early_stop_worsen_abs_tol = DeclareAndGet<double>(
      node, "ground_registration.early_stop_worsen_abs_tol",
      p.early_stop_worsen_abs_tol);

  return p;
}

PlanarRegistrationParams GaloOdometryComponent::LoadPlanarRegistrationParams(
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
  p.early_stop_worsen_rel_tol = DeclareAndGet<double>(
      node, "planar_registration.early_stop_worsen_rel_tol",
      p.early_stop_worsen_rel_tol);
  p.early_stop_worsen_abs_tol = DeclareAndGet<double>(
      node, "planar_registration.early_stop_worsen_abs_tol",
      p.early_stop_worsen_abs_tol);
  p.grid_reserve = DeclareAndGet<int>(node, "planar_registration.grid_reserve",
                                      p.grid_reserve);

  return p;
}

PredictionParams GaloOdometryComponent::LoadPredictionParams(rclcpp::Node& node) {
  PredictionParams p;

  p.rear_track_ =
      DeclareAndGet<double>(node, "prediction.rear_track", p.rear_track_);
  p.wheelbase_ =
      DeclareAndGet<double>(node, "prediction.wheelbase", p.wheelbase_);
  p.max_diff_residual = DeclareAndGet<double>(
      node, "prediction.max_diff_residual", p.max_diff_residual);
  p.max_jump = DeclareAndGet<double>(node, "prediction.max_jump", p.max_jump);
  p.min_speed_for_turn_check = DeclareAndGet<double>(
      node, "prediction.min_speed_for_turn_check", p.min_speed_for_turn_check);
  p.tau = DeclareAndGet<double>(node, "prediction.tau", p.tau);
  p.min_prediction_dt = DeclareAndGet<double>(
      node, "prediction.min_prediction_dt", p.min_prediction_dt);
  p.max_prediction_dt = DeclareAndGet<double>(
      node, "prediction.max_prediction_dt", p.max_prediction_dt);
  p.max_wheel_data_age = DeclareAndGet<double>(
      node, "prediction.max_wheel_data_age", p.max_wheel_data_age);
  p.min_valid_speed = DeclareAndGet<double>(node, "prediction.min_valid_speed",
                                            p.min_valid_speed);
  p.max_steering_correction = DeclareAndGet<double>(
      node, "prediction.max_steering_correction", p.max_steering_correction);

  return p;
}

GnssLocalizationParams GaloOdometryComponent::LoadGnssParams(rclcpp::Node& node) {
  GnssLocalizationParams p;

  p.base_lat =
      DeclareAndGet<double>(node, "gnss_location.base_lat", p.base_lat);
  p.base_lon =
      DeclareAndGet<double>(node, "gnss_location.base_lon", p.base_lon);
  p.northp = DeclareAndGet<bool>(node, "gnss_location.northp", p.northp);
  p.zone = DeclareAndGet<int>(node, "gnss_location.utm_zone", p.zone);
  return p;
}

GroundRegistrationGatePrms GaloOdometryComponent::LoadGroundGateParams(rclcpp::Node& node) {
  GroundRegistrationGatePrms p;

  p.max_dpitch = DeclareAndGet<double>(
      node, "ground_registration_gate_params.max_dpitch", p.max_dpitch);

  p.max_droll = DeclareAndGet<double>(
      node, "ground_registration_gate_params.max_droll", p.max_droll);

  p.max_dz = DeclareAndGet<double>(
      node, "ground_registration_gate_params.max_dz", p.max_dz);

  p.max_residual = DeclareAndGet<double>(
      node, "ground_registration_gate_params.max_residual", p.max_residual);

  p.min_matches = DeclareAndGet<int>(
      node, "ground_registration_gate_params.min_matches", p.min_matches);
  return p;
}
