#include "ground_aware_lidar_odometry/node.hpp"

#include "rclcpp/executors/multi_threaded_executor.hpp"

GALONode::GALONode()
    : Node("galo"),
      node_params_(LoadNodeParams(*this)),
      segementation_params_(LoadGroundSegmentationParams(*this)),
      ground_patch_params_(LoadGroundPatchParams(*this)),
      ground_registration_params_(LoadGroundRegistrationParams(*this)),
      planar_registration_params_(LoadPlanarRegistrationParams(*this)),
      ground_reg_gate_params_(LoadGroundGateParams(*this)),
      gnss_loc_params_(LoadGnssParams(*this)),
      prediction_params_(LoadPredictionParams(*this)),
      position_predictor_(prediction_params_),
      gnss_converter_(gnss_loc_params_),
      segmentation_(segementation_params_),
      ground_patches_extractor_(ground_patch_params_),
      ground_registration_(ground_registration_params_, this->get_logger(),
                           *this->get_clock()),
      planar_registration_(planar_registration_params_, this->get_logger(),
                           *this->get_clock()) {
  imu_frame = node_params_.imu_frame;
  lidar_frame = node_params_.lidar_frame;
  pos_antena_frame = node_params_.pos_antenna_frame;
  orientation_antenna_frame = node_params_.orientation_antenna_frame;
  map_frame = node_params_.map_frame;
  body_frame = node_params_.body_frame;
  imu_orientation_queue_.Resize(node_params_.imu_orientation_queue_size);

  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  lidar_callback_group_ =
      this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  other_callback_group_ =
      this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  rclcpp::SubscriptionOptions lidar_sub_options;
  lidar_sub_options.callback_group = lidar_callback_group_;

  rclcpp::SubscriptionOptions other_sub_options;
  other_sub_options.callback_group = other_callback_group_;

  RCLCPP_INFO(this->get_logger(), "Loaded GALO parameters from ROS params");
  lidar_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      node_params_.deskewed_cloud_topic, 10,
      std::bind(&GALONode::LidarCb, this, std::placeholders::_1),
      lidar_sub_options);

  imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
      node_params_.imu_topic, 1,
      std::bind(&GALONode::ImuCb, this, std::placeholders::_1),
      other_sub_options);
  gnss_sub_ = this->create_subscription<qarl_msgs::msg::NmeaGGA>(
      node_params_.gnss_topic, 1,
      std::bind(&GALONode::GnssCb, this, std::placeholders::_1),
      other_sub_options);
  gnss_orientation_sub_ =
      this->create_subscription<qarl_msgs::msg::OrientationStamped>(
          node_params_.gnss_orientation_topic, 1,
          std::bind(&GALONode::GnssYawCb, this, std::placeholders::_1),
          other_sub_options);
  pure_state_sub_ = this->create_subscription<common_msgs::msg::PureState>(
      node_params_.pure_state_topic, 1,
      std::bind(&GALONode::PureStateCb, this, std::placeholders::_1),
      other_sub_options);
  wheel_speed_sub_ = this->create_subscription<common_msgs::msg::WheelSpeed>(
      node_params_.wheel_speed_topic, 1,
      std::bind(&GALONode::WheelSpeedCb, this, std::placeholders::_1),
      other_sub_options);
  wa_sub_ = this->create_subscription<qarl_msgs::msg::WAngleFeedback>(
      node_params_.wheel_angle_topic, 1,
      std::bind(&GALONode::WheelAngleCb, this, std::placeholders::_1),
      other_sub_options);
  deskew_cld_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
      node_params_.deskewed_cloud_topic, 1);
  colored_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
      node_params_.colored_cloud_topic, 1);
  ground_patches_pub_ =
      this->create_publisher<visualization_msgs::msg::MarkerArray>(
          node_params_.ground_patches_topic, 10);
  translation_pub_ = this->create_publisher<geometry_msgs::msg::Point>(
      node_params_.translation_topic, 1);
  gt_eulers_pub_ = this->create_publisher<geometry_msgs::msg::Point>(
      node_params_.gt_eulers_topic, 1);
  est_eulers_pub_ = this->create_publisher<geometry_msgs::msg::Point>(
      node_params_.est_eulers_topic, 1);
  pure_state_eulers_pub_ = this->create_publisher<geometry_msgs::msg::Point>(
      node_params_.pure_state_eulers_topic, 1);
  gnss_imu_pose_pub_ =
      this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
          node_params_.true_pose_topic, 1);
  lidar_pose_pub_ =
      this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
          node_params_.estimate_pose_topic, 1);
  speed_pub_ = this->create_publisher<std_msgs::msg::Float32>(
      node_params_.speed_topic, 1);

  if (node_params_.gnss_correction_period_sec > 0.0) {
    auto correction_period =
        std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::duration<double>(
                node_params_.gnss_correction_period_sec));
    gnss_correction_timer_ = this->create_wall_timer(
        correction_period, std::bind(&GALONode::GnssCorrectionTimerCb, this),
        lidar_callback_group_);
    RCLCPP_INFO(this->get_logger(),
                "GNSS LiDAR odometry correction timer period: %.3f s",
                node_params_.gnss_correction_period_sec);
  }
}

void GALONode::WheelAngleCb(
    const qarl_msgs::msg::WAngleFeedback::SharedPtr msg) {
  std::lock_guard<std::mutex> lock(mut_);
  last_wa_ = msg->wangle;
  WheelSpeedAngleData speed_data;
}

void GALONode::WheelSpeedCb(const common_msgs::msg::WheelSpeed::SharedPtr msg) {
  std::lock_guard<std::mutex> lock(mut_);
  wheel_data.wheel_angle = last_wa_;
  wheel_data.left_speed = msg->rear_left;
  wheel_data.right_speed = msg->rear_right;
  wheel_data.wheel_time = rclcpp::Time(msg->header.stamp).seconds();
  wheel_data.has_wheel_data = true;
  RearWheelSpeedResult speed_res =
      position_predictor_.EstimateRearAxleSpeed(wheel_data);
  (void)speed_res;
}

void GALONode::PureStateCb(const common_msgs::msg::PureState::SharedPtr msg) {
  Eigen::Quaterniond q =
      Eigen::Quaterniond(msg->pose.orientation.w, msg->pose.orientation.x,
                         msg->pose.orientation.y, msg->pose.orientation.z);
  q.normalize();
  auto [roll, pitch, yaw] = EulersFromMatrixSimple(q.toRotationMatrix());

  geometry_msgs::msg::Point eulers_msg;
  eulers_msg.x = roll;
  eulers_msg.y = pitch;
  eulers_msg.z = yaw;
  pure_state_eulers_pub_->publish(eulers_msg);
}

void GALONode::GnssCorrectionTimerCb() {
  while (rclcpp::ok() && !TryInitializeOdomFromGnss(true)) {
    RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(),
        node_params_.initialization_log_throttle,
        "Waiting for GNSS position/yaw to correct LiDAR odometry");
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  }
}

void GALONode::GnssCb(const qarl_msgs::msg::NmeaGGA::SharedPtr msg) {
  // Only use high-quality GNSS fixes.
  // gps_qual >= 4 usually means RTK fixed / high-confidence solution.
  if (msg->gps_qual < node_params_.min_gnss_quality) {
    return;
  }

  Eigen::Vector3d gnss_local =
      gnss_converter_.ToLocal(msg->lon, msg->lat, msg->altitude);

  std::lock_guard<std::mutex> lock(mut_);
  gnss_data_.nmea_time = rclcpp::Time(msg->header.stamp).seconds();
  gnss_data_.gnss_local_ = gnss_local;
  gnss_data_.has_gnss_position_ = true;
}

void GALONode::GnssYawCb(
    const qarl_msgs::msg::OrientationStamped::SharedPtr msg) {
  // We need:
  //
  // 1. Static TFs:
  //      imu <- lidar
  //      imu <- map
  //      pos_antenna <- lidar
  //
  // 2. IMU orientation history, because roll/pitch come from IMU.
  //
  // 3. GNSS origin, because GNSS local coordinates are only meaningful
  //    after the first valid GNSS position has initialized the origin.
  bool has_imu_history = false;
  {
    std::lock_guard<std::mutex> lock(mut_);
    has_imu_history = imu_orientation_queue_.Size() >= 2;
  }
  if (!GetExtrinsicTf(*tf_buffer_, imu_frame, lidar_frame) ||
      !has_imu_history) {
    return;
  }

  const double yaw_time = rclcpp::Time(msg->header.stamp).seconds();

  // Find IMU orientation at the GNSS heading timestamp.
  //
  // This gives us roll/pitch at approximately the same time as the GNSS yaw.
  std::optional<Eigen::Quaterniond> q_closest;
  {
    std::lock_guard<std::mutex> lock(mut_);
    q_closest = GetImuOrientationAt(yaw_time);
  }

  if (!q_closest) {
    return;
  }

  Eigen::Vector3d gnss_local;
  {
    std::lock_guard<std::mutex> lock(mut_);

    if (!gnss_data_.has_gnss_position_) {
      return;
    }
    double pos_time = gnss_data_.nmea_time;

    if (std::abs(yaw_time - pos_time) > node_params_.gnss_yaw_position_max_dt) {
      RCLCPP_WARN(this->get_logger(),
                  "GNSS position/yaw timestamp mismatch: %.3f s",
                  std::abs(yaw_time - pos_time));
      return;
    }

    // Position of pos_antenna in the local GNSS/map frame.
    gnss_local = gnss_data_.gnss_local_;
  }

  // --------------------------------------------------------------------------
  // Convert Trimble heading to ROS ENU yaw
  // --------------------------------------------------------------------------
  //
  // Trimble heading convention:
  //   heading = clockwise from North
  //
  // ROS ENU yaw convention:
  //   yaw = counter-clockwise from East
  double gnss_yaw = gnss_converter_.YawFromGnssHeading(msg->orientation.yaw);
  Eigen::Matrix3d R_map_base =
      q_closest->toRotationMatrix() * q_body_imu.toRotationMatrix().transpose();

  auto [roll_imu, pitch_imu, yaw_imu_unused] =
      EulersFromMatrixSimple(R_map_base);

  (void)yaw_imu_unused;

  // --------------------------------------------------------------------------
  // Build map <- base orientation
  // --------------------------------------------------------------------------
  //
  // Final vehicle/base attitude:
  //
  //   roll  = IMU roll
  //   pitch = IMU pitch
  //   yaw   = GNSS dual-antenna yaw
  //
  // Rotation convention:
  //
  //   R = Rz(yaw) * Ry(pitch) * Rx(roll)
  Eigen::AngleAxisd yaw_rot(gnss_yaw, Eigen::Vector3d::UnitZ());
  Eigen::AngleAxisd roll_rot(roll_imu, Eigen::Vector3d::UnitX());
  Eigen::AngleAxisd pitch_rot(pitch_imu, Eigen::Vector3d::UnitY());

  Eigen::Matrix3d R_gt = yaw_rot.toRotationMatrix() *
                         pitch_rot.toRotationMatrix() *
                         roll_rot.toRotationMatrix();
  Eigen::Vector3d t_body_antenna_mid = 0.5 * (t_body_pos + t_body_orientation);
  Eigen::Vector3d t_gt = gnss_local - R_gt * t_body_antenna_mid;

  // --------------------------------------------------------------------------
  // Publish GT pose
  // --------------------------------------------------------------------------
  Eigen::Vector<double, 6> sigmas = Eigen::Vector<double, 6>(
      node_params_.gt_cov_.x_precision, node_params_.gt_cov_.y_precision,
      node_params_.gt_cov_.z_precision,
      node_params_.gt_cov_.roll_precision * M_PI / 180.0,
      node_params_.gt_cov_.pitch_precision * M_PI / 180.0,
      node_params_.gt_cov_.yaw_precision * M_PI / 180.0);
  geometry_msgs::msg::PoseWithCovarianceStamped pose_msg =
      BuildPoseWithCovarianceMsg(msg->header.stamp, node_params_.gnss_map_frame,
                                 R_gt, t_gt, sigmas);
  gnss_imu_pose_pub_->publish(pose_msg);

  auto [roll_gt, pitch_gt, yaw_gt] = EulersFromMatrixSimple(R_gt);
  geometry_msgs::msg::Point eulers_msg;
  eulers_msg.x = roll_gt;
  eulers_msg.y = pitch_gt;
  eulers_msg.z = NormalizeAngle0To2Pi(yaw_gt);
  gt_eulers_pub_->publish(eulers_msg);

  // --------------------------------------------------------------------------
  // Store GNSS yaw for odometry initialization
  // --------------------------------------------------------------------------
  // TryInitializeOdomFromGnss() should use this yaw as map <- base/pos_antenna,
  // and then compose with pos_antenna <- lidar.
  {
    std::lock_guard<std::mutex> lock(mut_);

    gnss_data_.yaw = gnss_yaw;
    gnss_data_.has_gnss_yaw = true;
    gnss_data_.yaw_time = yaw_time;
    latest_R_base_ = R_gt;
    has_latest_R_base_ = true;
  }
}

void GALONode::ImuCb(const sensor_msgs::msg::Imu::SharedPtr msg) {
  Eigen::Quaterniond imu_q_curr(msg->orientation.w, msg->orientation.x,
                                msg->orientation.y, msg->orientation.z);
  imu_q_curr.normalize();

  rclcpp::Time msg_time(msg->header.stamp);

  {
    std::lock_guard<std::mutex> lock(mut_);
    ImuOrientationStamped item;
    item.time = rclcpp::Time(msg->header.stamp).seconds();
    item.q = imu_q_curr;
    imu_orientation_queue_.Update(item);
    if (has_imu_prev_) {
      R_imu_delta = imu_q_prev_.toRotationMatrix().transpose() *
                    imu_q_curr.toRotationMatrix();
    }

    imu_q_prev_ = imu_q_curr;
    has_imu_prev_ = true;

    (void)msg_time;
  }
}

void GALONode::LidarCb(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
  // Main LiDAR pipeline:
  //
  // Main odometry backend pipeline:
  //
  //   1. Receive deskewed cloud
  //   2. Segment ground/non-ground
  //   3. Extract planar/non-ground points
  //   4. Extract ground patches
  //   5. Initialize from GNSS if needed
  //   6. Run planar ICP for x/y/yaw
  //   7. Run ground registration for z/roll/pitch
  //   8. Merge results
  //   9. Publish odometry pose
  //   10. Add current frame into local map
  ProcessCloud(msg);

  // Print throttled timing summary.
  PrintTimeMeasurments(time_measurments);
}

void GALONode::ProcessCloud(
    const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
  time_measurments.clear();

  const FrameFeatures features = ExtractFrameFeatures(*msg);
  EstimatePoseFromFeatures(features);
}

FrameFeatures GALONode::ExtractFrameFeatures(const CloudMsg& cloud) {
  FrameFeatures features;
  features.cloud = cloud;
  features.lidar_time = rclcpp::Time(cloud.header.stamp).seconds();

  SegmentationResult segmentation_result;
  {
    TimeMeasurments_t meas("segmentaion");
    segmentation_result = segmentation_.Classify(cloud);
    meas.SetEnd();
    time_measurments.push_back(meas);
    if (node_params_.debug) {
      auto colored_cld = segmentation_.MakeColoredCloud(segmentation_result);
      colored_pub_->publish(colored_cld);
    }
  }
  {
    TimeMeasurments_t meas("objects_extration");
    features.planar_points = planar_registration_.ExtractPoints(
        cloud, segmentation_result.labels);
    if (features.planar_points.size()) {
      features.planar_points = planar_registration_.Filter(
          features.planar_points);
    }
    meas.SetEnd();
    time_measurments.push_back(meas);
  }
  {
    TimeMeasurments_t meas("ground_extration");
    features.ground_patches =
        ground_patches_extractor_.Extract(cloud, segmentation_result.labels);
    meas.SetEnd();
    time_measurments.push_back(meas);
  }

  if (node_params_.debug) {
    auto patches_marker =
        ground_patches_extractor_.MakeGroundPatchMarkers(cloud.header);
    ground_patches_pub_->publish(patches_marker);
  }

  return features;
}

void GALONode::EstimatePoseFromFeatures(const FrameFeatures& features) {
  const double lidar_time = features.lidar_time;
  if (!TryInitializeOdomFromGnss()) {
    RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(),
        node_params_.initialization_log_throttle,
        "Waiting for GNSS position/yaw to initialize LiDAR odometry");
    return;
  }
  PruneStaleMapFrames(lidar_time);
  RebuildGroundMap();
  RebuildObjectsMap();

  if (ground_map_.empty() || objects_map_.empty()) {
    RCLCPP_WARN(this->get_logger(),
                "Either objects or ground map is empty %zu %zu",
                ground_map_.size(), objects_map_.size());

    auto patches_in_map =
        TransformPatchesToMap(features.ground_patches, R_map_lidar_,
                              t_map_lidar);
    ground_map_frames_.push_back(GroundPatchFrame{patches_in_map, lidar_time});

    auto planar_points_in_map =
        TransformPointsToMap(features.planar_points, R_map_lidar_,
                             t_map_lidar);
    objects_map_frames_.push_back(PlanarMapFrame{planar_points_in_map,
                                                 lidar_time});
    PruneStaleMapFrames(lidar_time);
    RebuildGroundMap();
    RebuildObjectsMap();
    RCLCPP_WARN(
        this->get_logger(),
        "Initialized first map frame at pose t=[%.3f %.3f %.3f], yaw=%.3f",
        t_map_lidar.x(), t_map_lidar.y(), t_map_lidar.z(),
        std::atan2(R_map_lidar_(1, 0), R_map_lidar_(0, 0)));
    return;
  }
  bool has_imu_history = false;
  {
    std::lock_guard<std::mutex> lock(mut_);
    has_imu_history = imu_orientation_queue_.Size() >= 2;
  }
  if (!GetExtrinsicTf(*tf_buffer_, imu_frame, lidar_frame) ||
      !has_imu_history) {
    return;
  }
  GnssData gnss_data_local;
  {
    std::lock_guard<std::mutex> lock(mut_);
    gnss_data_local = gnss_data_;
  }

  std::optional<Eigen::Quaterniond> q_lidar;
  {
    std::lock_guard<std::mutex> lock(mut_);
    q_lidar = GetImuOrientationAt(lidar_time);
  }

  if (!q_lidar) {
    return;
  }

  if (!has_lidar_imu_prev_) {
    imu_q_lidar_prev_ = *q_lidar;
    has_lidar_imu_prev_ = true;
    return;
  }
  // IMU relative rotation between previous and current LiDAR scan timestamps.
  // With q = R_world_imu / R_map_imu style body-to-world orientation,
  // R_prev.transpose() * R_curr maps vectors from current IMU frame to
  // previous IMU frame.
  Eigen::Matrix3d R_imu_delta_lidar =
      imu_q_lidar_prev_.toRotationMatrix().transpose() *
      q_lidar->toRotationMatrix();
  // Static extrinsic rotation between LiDAR and IMU frames.
  // q_imu_lidar comes from lookupTransform(imu_frame, lidar_frame), so R_ex
  // maps: lidar frame -> imu frame.
  Eigen::Matrix3d R_ex = q_imu_lidar.toRotationMatrix();
  // Convert IMU-frame delta into LiDAR-frame delta.
  // Result maps current LiDAR frame -> previous LiDAR frame.
  Eigen::Matrix3d R_lidar_delta = R_ex.transpose() * R_imu_delta_lidar * R_ex;
  imu_q_lidar_prev_ = *q_lidar;

  // Predicted absolute map rotation from previous pose + IMU delta
  Eigen::Matrix3d R_imu_prior_map = R_map_lidar_ * R_lidar_delta;

  // Planar initial guess from current global pose
  PredictedPose pred;
  pred.t = t_map_lidar;
  pred.yaw = std::atan2(R_map_lidar_(1, 0), R_map_lidar_(0, 0));
  pred.speed = 0.0;
  WheelSpeedAngleData wheel_data_loc;
  {
    std::lock_guard<std::mutex> lock(mut_);
    wheel_data_loc = wheel_data;
  }

  if (has_prev_lidar_pose_for_prediction_) {
    std::lock_guard<std::mutex> lock(mut_);
    pred = position_predictor_.PredictFromWheelModel(
        wheel_data_loc, R_map_lidar_, t_map_lidar, prev_lidar_pose_time_,
        lidar_time);
  }

  std_msgs::msg::Float32 speed_msg;
  speed_msg.data = pred.speed;
  speed_pub_->publish(speed_msg);

  PlanarRegistrationResult planar_reg_res;
  {
    double c = std::cos(pred.yaw);
    double s = std::sin(pred.yaw);

    Eigen::Matrix2d R_planar_initial;
    R_planar_initial << c, -s, s, c;

    Eigen::Vector2d t_planar_initial(pred.t.x(), pred.t.y());
    TimeMeasurments_t meas("planar_registration");
    planar_reg_res = planar_registration_.Align(
        objects_map_, features.planar_points, R_planar_initial,
        t_planar_initial);
    meas.SetEnd();
    time_measurments.push_back(meas);
  }

  if (!planar_reg_res.valid) {
    RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(),
        node_params_.registration_log_throttle,
        "Planar registration invalid, skipping odometry update");
    return;
  }
  GroundRegistrationResult ground_reg_result;
  {
    Eigen::Vector3d t_ground_initial;
    t_ground_initial.x() = planar_reg_res.t.x();
    t_ground_initial.y() = planar_reg_res.t.y();
    t_ground_initial.z() = pred.t.z();

    double yaw_planar =
        std::atan2(planar_reg_res.R(1, 0), planar_reg_res.R(0, 0));

    auto [roll_imu, pitch_imu, yaw_unused] =
        EulersFromMatrixSimple(R_imu_prior_map);
    (void)yaw_unused;

    Eigen::Matrix3d R_ground_initial =
        Eigen::AngleAxisd(yaw_planar, Eigen::Vector3d::UnitZ())
            .toRotationMatrix() *
        Eigen::AngleAxisd(pitch_imu, Eigen::Vector3d::UnitY())
            .toRotationMatrix() *
        Eigen::AngleAxisd(roll_imu, Eigen::Vector3d::UnitX())
            .toRotationMatrix();
    TimeMeasurments_t meas("ground_registration");
    ground_reg_result =
        ground_registration_.Align(ground_map_, features.ground_patches,
                                   R_imu_prior_map, R_ground_initial,
                                   t_ground_initial);
    meas.SetEnd();
    time_measurments.push_back(meas);
  }

  if (!planar_reg_res.valid) {
    RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(),
        node_params_.registration_log_throttle,
        "Planar registration invalid, skipping odometry update");
    return;
  }
  bool is_ground_ok = CheckGroundRegistration(ground_reg_result);

  Eigen::Matrix3d R_abs;
  Eigen::Vector3d t_abs;
  if (is_ground_ok) {
    R_abs = MergeGroundAndPlanarRotation(
        ground_reg_result.R, R_map_lidar_, planar_reg_res.R,
        node_params_.merge_alpha_rp, node_params_.merge_alpha_yaw);
    t_abs = MergeGroundAndPlanarTranslation(
        ground_reg_result.t, pred.t, planar_reg_res.t,
        node_params_.merge_alpha_xy, node_params_.merge_alpha_z);
  } else {
    // Ground failed: trust planar x/y/yaw, keep previous z, take roll/pitch
    // from IMU prior.
    RCLCPP_WARN(this->get_logger(),
                "ground valid=%d matches=%d residual=%.3f dz=%.3f ok=%d",
                ground_reg_result.valid, ground_reg_result.num_matches,
                ground_reg_result.mean_abs_residual,
                ground_reg_result.t.z() - t_map_lidar.z(), is_ground_ok);
    double yaw = std::atan2(planar_reg_res.R(1, 0), planar_reg_res.R(0, 0));

    auto [roll_prior, pitch_prior, yaw_prior_unused] =
        EulersFromMatrixSimple(R_imu_prior_map);
    (void)yaw_prior_unused;
    // auto [roll_prior, pitch_prior, yaw_prior_unused] =
    //     EulersFromMatrixSimple(R_map_lidar_);
    // (void)yaw_prior_unused;

    R_abs =
        Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix() *
        Eigen::AngleAxisd(pitch_prior, Eigen::Vector3d::UnitY())
            .toRotationMatrix() *
        Eigen::AngleAxisd(roll_prior, Eigen::Vector3d::UnitX())
            .toRotationMatrix();

    const double alpha_xy = node_params_.fallback_alpha_xy;
    const double alpha_z = ground_reg_result.valid
                               ? node_params_.fallback_alpha_z_valid_ground
                               : 0.0;
    t_abs.x() = pred.t.x() + alpha_xy * (planar_reg_res.t.x() - pred.t.x());
    t_abs.y() = pred.t.y() + alpha_xy * (planar_reg_res.t.y() - pred.t.y());
    t_abs.z() =
        t_map_lidar.z() + alpha_z * (ground_reg_result.t.z() - t_map_lidar.z());
  }
  R_map_lidar_ = R_abs;
  t_map_lidar = t_abs;
  double dt_pose = lidar_time - prev_lidar_pose_time_;
  if (has_prev_lidar_pose_for_prediction_ &&
      dt_pose > node_params_.pose_dt_min &&
      dt_pose < node_params_.pose_dt_max) {
    velocity_map_lidar_ = (t_abs - prev_t_map_lidar_) / dt_pose;
  }

  prev_t_map_lidar_ = t_abs;
  prev_lidar_pose_time_ = lidar_time;
  has_prev_lidar_pose_for_prediction_ = true;
  auto planar_points_in_map =
      TransformPointsToMap(features.planar_points, R_map_lidar_, t_map_lidar);

  objects_map_frames_.push_back(PlanarMapFrame{planar_points_in_map,
                                               lidar_time});

  while (objects_map_frames_.size() >
         static_cast<size_t>(node_params_.max_planar_map_frames)) {
    objects_map_frames_.pop_front();
  }

  if (features.ground_patches.size() >=
          static_cast<size_t>(node_params_.min_ground_patches_for_map_update) &&
      planar_reg_res.valid) {
    auto patches_in_map =
        TransformPatchesToMap(features.ground_patches, R_map_lidar_,
                              t_map_lidar);

    ground_map_frames_.push_back(GroundPatchFrame{patches_in_map, lidar_time});

    while (ground_map_frames_.size() >
           static_cast<size_t>(node_params_.max_ground_map_frames)) {
      ground_map_frames_.pop_front();
    }
  }
  PruneStaleMapFrames(lidar_time);
  RebuildGroundMap();
  RebuildObjectsMap();
  // T_map_base_est = T_map_lidar_est * T_lidar_base
  Eigen::Matrix3d R_lidar_body = q_lidar_body.toRotationMatrix();

  Eigen::Matrix3d R_map_base_est = R_abs * R_lidar_body;
  Eigen::Vector3d t_map_base_est = t_abs + R_abs * t_lidar_body;

  const auto& [sigma_xy, sigma_yaw] =
      node_params_.est_cov_.GetXYYawSigmas(planar_reg_res);

  Eigen::Vector<double, 6> sigmas = Eigen::Vector<double, 6>(
      sigma_xy, sigma_xy, node_params_.est_cov_.z, node_params_.est_cov_.roll,
      node_params_.est_cov_.pitch, sigma_yaw);
  geometry_msgs::msg::PoseWithCovarianceStamped pose_msg =
      BuildPoseWithCovarianceMsg(features.cloud.header.stamp,
                                 node_params_.gnss_map_frame, R_map_base_est,
                                 t_map_base_est, sigmas);
  lidar_pose_pub_->publish(pose_msg);

  auto [roll_abs, pitch_abs, yaw_abs] = EulersFromMatrixSimple(R_map_base_est);
  geometry_msgs::msg::Point eulers_msg;
  eulers_msg.x = roll_abs;
  eulers_msg.y = pitch_abs;
  eulers_msg.z = yaw_abs;
  est_eulers_pub_->publish(eulers_msg);
}

bool GALONode::GetTransformation(tf2_ros::Buffer& tf_buffer,
                                 const std::string& target_frame,
                                 const std::string& source_frame,
                                 Eigen::Quaterniond& q, Eigen::Vector3d& t) {
  try {
    geometry_msgs::msg::TransformStamped tf_msg = tf_buffer.lookupTransform(
        target_frame, source_frame, tf2::TimePointZero);
    q = Eigen::Quaterniond(
        tf_msg.transform.rotation.w, tf_msg.transform.rotation.x,
        tf_msg.transform.rotation.y, tf_msg.transform.rotation.z);
    q.normalize();

    t = Eigen::Vector3d(tf_msg.transform.translation.x,
                        tf_msg.transform.translation.y,
                        tf_msg.transform.translation.z);
    return true;
  } catch (const tf2::TransformException& ex) {
    RCLCPP_WARN(this->get_logger(), "Failed to get TF %s <- %s: %s",
                target_frame.c_str(), source_frame.c_str(), ex.what());
    return false;
  }
}

bool GALONode::GetOrientation(tf2_ros::Buffer& tf_buffer,
                              const std::string& target_frame,
                              const std::string& source_frame,
                              Eigen::Quaterniond& q) {
  try {
    geometry_msgs::msg::TransformStamped tf_msg = tf_buffer.lookupTransform(
        target_frame, source_frame, tf2::TimePointZero);
    q = Eigen::Quaterniond(
        tf_msg.transform.rotation.w, tf_msg.transform.rotation.x,
        tf_msg.transform.rotation.y, tf_msg.transform.rotation.z);
    q.normalize();
    return true;
  } catch (const tf2::TransformException& ex) {
    RCLCPP_WARN(this->get_logger(), "Failed to get TF %s <- %s: %s",
                target_frame.c_str(), source_frame.c_str(), ex.what());
    return false;
  }
}

bool GALONode::GetTranslation(tf2_ros::Buffer& tf_buffer,
                              const std::string& target_frame,
                              const std::string& source_frame,
                              Eigen::Vector3d& t) {
  try {
    geometry_msgs::msg::TransformStamped tf_msg = tf_buffer.lookupTransform(
        target_frame, source_frame, tf2::TimePointZero);
    t = Eigen::Vector3d(tf_msg.transform.translation.x,
                        tf_msg.transform.translation.y,
                        tf_msg.transform.translation.z);
    return true;
  } catch (const tf2::TransformException& ex) {
    RCLCPP_WARN(this->get_logger(), "Failed to get TF %s <- %s: %s",
                target_frame.c_str(), source_frame.c_str(), ex.what());
    return false;
  }
}

bool GALONode::GetExtrinsicTf(tf2_ros::Buffer& tf_buffer,
                              const std::string& imu_frame,
                              const std::string& lidar_frame) {
  std::lock_guard<std::mutex> lock(mut_);
  if (has_imu_lidar_extrinsic_) return has_imu_lidar_extrinsic_;

  bool res =
      GetOrientation(tf_buffer, imu_frame, lidar_frame, q_imu_lidar) &&
      GetTransformation(tf_buffer, map_frame, lidar_frame, q_map_lidar,
                        t_map_lidar) &&
      GetTransformation(tf_buffer, lidar_frame, body_frame, q_lidar_body,
                        t_lidar_body) &&
      GetTransformation(tf_buffer, body_frame, imu_frame, q_body_imu,
                        t_body_imu) &&
      GetTranslation(tf_buffer, body_frame, pos_antena_frame, t_body_pos) &&
      GetTranslation(tf_buffer, body_frame, orientation_antenna_frame,
                     t_body_orientation);
  has_imu_lidar_extrinsic_ = res;
  return res;
}

bool GALONode::TryInitializeOdomFromGnss(bool force_correction) {
  constexpr double kMaxGnssDataAgeSec = 0.2;

  if (has_lidar_odom_initialized_from_gnss_ && !force_correction) {
    return true;
  }

  if (!GetExtrinsicTf(*tf_buffer_, imu_frame, lidar_frame)) {
    return false;
  }

  Eigen::Vector3d gnss_local;
  Eigen::Matrix3d R_base = Eigen::Matrix3d::Identity();

  {
    std::lock_guard<std::mutex> lock(mut_);

    if (!gnss_data_.has_gnss_position_ || !gnss_data_.has_gnss_yaw ||
        !has_latest_R_base_) {
      return false;
    }

    const double now = this->get_clock()->now().seconds();
    const double position_age = now - gnss_data_.nmea_time;
    const double yaw_age = now - gnss_data_.yaw_time;
    if (position_age < 0.0 || position_age > kMaxGnssDataAgeSec ||
        yaw_age < 0.0 || yaw_age > kMaxGnssDataAgeSec) {
      RCLCPP_WARN_THROTTLE(
          this->get_logger(), *this->get_clock(),
          node_params_.initialization_log_throttle,
          "GNSS data is stale for LiDAR odometry correction: "
          "position_age=%.3f s yaw_age=%.3f s",
          position_age, yaw_age);
      return false;
    }

    gnss_local = gnss_data_.gnss_local_;
    R_base = latest_R_base_;
  }

  Eigen::Vector3d t_body_antenna_mid = 0.5 * (t_body_pos + t_body_orientation);
  Eigen::Vector3d t_map_base = gnss_local - R_base * t_body_antenna_mid;

  Eigen::Matrix3d R_lidar_body = q_lidar_body.toRotationMatrix();
  Eigen::Matrix3d R_body_lidar = R_lidar_body.transpose();
  Eigen::Vector3d t_body_lidar = -R_body_lidar * t_lidar_body;

  R_map_lidar_ = R_base * R_body_lidar;
  t_map_lidar = t_map_base + R_base * t_body_lidar;

  has_lidar_odom_initialized_from_gnss_ = true;
  prev_t_map_lidar_ = t_map_lidar;
  velocity_map_lidar_ = Eigen::Vector3d::Zero();
  has_prev_lidar_pose_for_prediction_ = false;

  auto [base_roll, base_pitch, base_yaw] = EulersFromMatrixSimple(R_base);
  auto [lidar_roll, lidar_pitch, lidar_yaw] =
      EulersFromMatrixSimple(R_map_lidar_);

  RCLCPP_INFO(this->get_logger(),
              "%s LiDAR odometry from GNSS+IMU: "
              "t=[%.3f %.3f %.3f], "
              "base_rpy=[%.3f %.3f %.3f] deg, "
              "lidar_rpy=[%.3f %.3f %.3f] deg",
              force_correction ? "Corrected" : "Initialized", t_map_lidar.x(),
              t_map_lidar.y(), t_map_lidar.z(), base_roll * 180.0 / M_PI,
              base_pitch * 180.0 / M_PI, base_yaw * 180.0 / M_PI,
              lidar_roll * 180.0 / M_PI, lidar_pitch * 180.0 / M_PI,
              lidar_yaw * 180.0 / M_PI);

  return true;
}

void GALONode::PrintTimeMeasurments(
    const std::vector<TimeMeasurments_t>& measurments) {
  if (measurments.empty()) return;
  bool has_big_meas = false;
  for (const auto& m : measurments) {
    if (GetDelayMs(m.start, m.end) >= node_params_.elapsed_time_thresh) {
      has_big_meas = true;
      break;
    }
  }
  if (!node_params_.debug && has_big_meas) return;
  std::stringstream ss;
  ss << "Time measurments (ms): ";

  bool first = true;
  for (auto m : measurments) {
    if (first)
      first = false;
    else
      ss << ", ";
    ss << m.label << " - " << GetDelayMs(m.start, m.end);
  }
  RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(),
                       node_params_.registration_log_throttle, "%s",
                       ss.str().c_str());
}

std::optional<Eigen::Quaterniond> GALONode::GetImuOrientationAt(
    double query_time) const {
  if (imu_orientation_queue_.Size() < 2) {
    return {};
  }

  if (query_time < imu_orientation_queue_.PeerFront().time ||
      query_time > imu_orientation_queue_.PeerBack().time) {
    return {};
  }

  size_t idx = 0;
  while (idx + 1 < imu_orientation_queue_.Size() &&
         imu_orientation_queue_[idx + 1].time < query_time) {
    ++idx;
  }

  const auto& a = imu_orientation_queue_[idx];
  const auto& b = imu_orientation_queue_[idx + 1];

  const double dt = b.time - a.time;
  if (dt <= 1e-9) {
    return a.q;
  }

  const double alpha = (query_time - a.time) / dt;
  return a.q.slerp(alpha, b.q);
}

std::vector<GroundPatch> GALONode::TransformPatchesToMap(
    const std::vector<GroundPatch>& patches, const Eigen::Matrix3d& R,
    const Eigen::Vector3d& t) {
  std::vector<GroundPatch> out;
  out.reserve(patches.size());

  for (auto p : patches) {
    p.centroid = R * p.centroid + t;
    p.normal = R * p.normal;
    p.normal.normalize();
    out.push_back(p);
  }
  return out;
}

std::vector<Eigen::Vector2d> GALONode::TransformPointsToMap(
    const std::vector<Eigen::Vector2d>& points, const Eigen::Matrix3d& R,
    const Eigen::Vector3d& t) {
  std::vector<Eigen::Vector2d> out;
  out.reserve(points.size());
  const double yaw = std::atan2(R(1, 0), R(0, 0));

  const double c = std::cos(yaw);
  const double s = std::sin(yaw);

  Eigen::Matrix2d R_2d;
  R_2d << c, -s, s, c;

  Eigen::Vector2d t_2d(t.x(), t.y());
  for (auto p : points) {
    p = R_2d * p + t_2d;
    out.push_back(p);
  }
  return out;
}

void GALONode::RebuildGroundMap() {
  ground_map_.clear();

  for (const auto& frame : ground_map_frames_) {
    ground_map_.insert(ground_map_.end(), frame.patches.begin(),
                       frame.patches.end());
  }
}

void GALONode::RebuildObjectsMap() {
  objects_map_.clear();

  for (const auto& frame : objects_map_frames_) {
    objects_map_.insert(objects_map_.end(), frame.points.begin(),
                        frame.points.end());
  }
}

void GALONode::PruneStaleMapFrames(double current_time) {
  const double max_age_sec = node_params_.map_stale_threshold_ms * 1e-3;
  if (max_age_sec <= 0.0) return;

  size_t pruned_ground_frames = 0;
  while (!ground_map_frames_.empty() &&
         current_time - ground_map_frames_.front().time > max_age_sec) {
    ground_map_frames_.pop_front();
    ++pruned_ground_frames;
  }

  size_t pruned_object_frames = 0;
  while (!objects_map_frames_.empty() &&
         current_time - objects_map_frames_.front().time > max_age_sec) {
    objects_map_frames_.pop_front();
    ++pruned_object_frames;
  }

  if (pruned_ground_frames > 0 || pruned_object_frames > 0) {
    RCLCPP_INFO_THROTTLE(
        this->get_logger(), *this->get_clock(),
        node_params_.registration_log_throttle,
        "Pruned stale map frames: ground=%zu objects=%zu "
        "threshold=%.1f ms remaining_ground=%zu remaining_objects=%zu",
        pruned_ground_frames, pruned_object_frames,
        node_params_.map_stale_threshold_ms, ground_map_frames_.size(),
        objects_map_frames_.size());
  }
}

Eigen::Matrix3d GALONode::MergeGroundAndPlanarRotation(
    const Eigen::Matrix3d& R_ground, const Eigen::Matrix3d& R_prior,
    const Eigen::Matrix2d& R_planar, double alpha_rp, double alpha_y) {
  auto [r_prior, p_prior, y_prior] = EulersFromMatrixSimple(R_prior);
  auto [r_ground, p_ground, y_ground] = EulersFromMatrixSimple(R_ground);
  (void)y_ground;

  double r_final = r_prior + alpha_rp * NormalizeAngle(r_ground - r_prior);
  double p_final = p_prior + alpha_rp * NormalizeAngle(p_ground - p_prior);

  double y_planar = std::atan2(R_planar(1, 0), R_planar(0, 0));
  double y_final = y_prior + alpha_y * NormalizeAngle(y_planar - y_prior);

  Eigen::AngleAxisd roll_rot(r_final, Eigen::Vector3d::UnitX());
  Eigen::AngleAxisd pitch_rot(p_final, Eigen::Vector3d::UnitY());
  Eigen::AngleAxisd yaw_rot(y_final, Eigen::Vector3d::UnitZ());

  return yaw_rot.toRotationMatrix() * pitch_rot.toRotationMatrix() *
         roll_rot.toRotationMatrix();
}

Eigen::Vector3d GALONode::MergeGroundAndPlanarTranslation(
    const Eigen::Vector3d& t_ground, const Eigen::Vector3d& t_prior,
    const Eigen::Vector2d& t_planar, double alpha_xy, double alpha_z) {
  Eigen::Vector3d t;
  t.x() = t_prior.x() + alpha_xy * (t_planar.x() - t_prior.x());
  t.y() = t_prior.y() + alpha_xy * (t_planar.y() - t_prior.y());

  double z_final = t_prior.z() + alpha_z * (t_ground.z() - t_prior.z());
  t.z() = z_final;
  return t;
}

geometry_msgs::msg::PoseWithCovarianceStamped
GALONode::BuildPoseWithCovarianceMsg(const builtin_interfaces::msg::Time& time,
                                     const std::string& frame,
                                     const Eigen::Matrix3d& R,
                                     const Eigen::Vector3d& t,
                                     const Eigen::Vector<double, 6>& sigmas) {
  geometry_msgs::msg::PoseWithCovarianceStamped pose_msg;
  pose_msg.header.stamp = time;
  pose_msg.header.frame_id = frame;

  pose_msg.pose.pose.position.x = t.x();
  pose_msg.pose.pose.position.y = t.y();
  pose_msg.pose.pose.position.z = t.z();

  auto [roll, pitch, yaw] = EulersFromMatrixSimple(R);

  tf2::Quaternion q;
  q.setRPY(roll, pitch, yaw);
  q.normalize();

  pose_msg.pose.pose.orientation = tf2::toMsg(q);

  auto& cov = pose_msg.pose.covariance;
  std::fill(cov.begin(), cov.end(), 0.0);

  cov[0] = std::pow(sigmas(0), 2);
  cov[7] = std::pow(sigmas(1), 2);
  cov[14] = std::pow(sigmas(2), 2);

  cov[21] = std::pow(sigmas(3), 2);
  cov[28] = std::pow(sigmas(4), 2);
  cov[35] = std::pow(sigmas(5), 2);
  return pose_msg;
}

bool GALONode::CheckGroundRegistration(const GroundRegistrationResult& res) {
  if (!res.valid || res.num_matches < ground_reg_gate_params_.min_matches ||
      res.mean_abs_residual > ground_reg_gate_params_.max_residual) {
    return false;
  }
  auto [roll_prev, pitch_prev, yaw_prev] = EulersFromMatrixSimple(R_map_lidar_);
  auto [roll_g, pitch_g, yaw_g] = EulersFromMatrixSimple(res.R);

  double droll = std::abs(
      std::atan2(std::sin(roll_g - roll_prev), std::cos(roll_g - roll_prev)));
  double dpitch = std::abs(std::atan2(std::sin(pitch_g - pitch_prev),
                                      std::cos(pitch_g - pitch_prev)));
  double dz = std::abs(res.t.z() - t_map_lidar.z());

  return droll < DEG2RAD(ground_reg_gate_params_.max_droll) &&
         dpitch < DEG2RAD(ground_reg_gate_params_.max_dpitch) &&
         dz < ground_reg_gate_params_.max_dz;
}

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<GALONode>();
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
