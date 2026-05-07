#include "ground_aware_lidar_odometry/node.hpp"

GALONode::GALONode()
    : Node("galo"),
      node_params_(LoadNodeParams(*this)),
      deskew_prms_(LoadDeskewParams(*this)),
      segementation_params_(LoadGroundSegmentationParams(*this)),
      ground_patch_params_(LoadGroundPatchParams(*this)),
      ground_registration_params_(LoadGroundRegistrationParams(*this)),
      planar_registration_params_(LoadPlanarRegistrationParams(*this)),
      gnss_loc_params_(LoadGnssParams(*this)),
      gnss_converter_(gnss_loc_params_),
      segmentation_(segementation_params_),
      ground_patches_extractor_(ground_patch_params_),
      ground_registration_(ground_registration_params_, this->get_logger(),
                           *this->get_clock()),
      planar_registration_(planar_registration_params_, this->get_logger(),
                           *this->get_clock()),
      deskew_algo_(deskew_prms_, this->get_logger(), *this->get_clock()) {
  imu_frame = DeclareAndGet<std::string>(*this, "frames.imu_frame", imu_frame);
  lidar_frame =
      DeclareAndGet<std::string>(*this, "frames.lidar_frame", lidar_frame);
  imu_orientation_queue_.Resize(2000);

  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  RCLCPP_INFO(this->get_logger(), "Loaded GALO parameters from ROS params");
  lidar_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "/Sensor/lidar_front/rslidar_points", 10,
      std::bind(&GALONode::LidarCb, this, std::placeholders::_1));

  imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
      "/Sensor/imu_front/data", 1,
      std::bind(&GALONode::ImuCb, this, std::placeholders::_1));
  gnss_sub_ = this->create_subscription<qarl_msgs::msg::NmeaGGA>(
      "/Sensor/gnss/trimble_nmea_gga", 1,
      std::bind(&GALONode::GnssCb, this, std::placeholders::_1));
  gnss_orientation_sub_ =
      this->create_subscription<qarl_msgs::msg::OrientationStamped>(
          "/Sensor/gnss/orientation", 1,
          std::bind(&GALONode::GnssYawCb, this, std::placeholders::_1));
  pure_state_sub_ = this->create_subscription<common_msgs::msg::PureState>(
      "/SC/pure_state", 1,
      std::bind(&GALONode::PureStateCb, this, std::placeholders::_1));
  deskew_cld_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
      "/GALO/deskewed_cloud", 1);
  colored_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
      "/GALO/colored_cloud", 1);
  ground_patches_pub_ =
      this->create_publisher<visualization_msgs::msg::MarkerArray>(
          "/GALO/ground_patch_normals", 10);
  translation_pub_ =
      this->create_publisher<geometry_msgs::msg::Point>("/GALO/translation", 1);
  gt_eulers_pub_ =
      this->create_publisher<geometry_msgs::msg::Point>("/GALO/gt_eulers", 1);
  est_eulers_pub_ =
      this->create_publisher<geometry_msgs::msg::Point>("/GALO/est_eulers", 1);
  pure_state_eulers_pub_ = this->create_publisher<geometry_msgs::msg::Point>(
      "/GALO/pure_state_eulers", 1);

  gnss_imu_pose_pub_ =
      this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
          "/GALO/true_pose", 1);
  lidar_pose_pub_ =
      this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
          "/GALO/estimate_pose", 1);
}

bool GALONode::GetExtrinsicTf(tf2_ros::Buffer& tf_buffer,
                              const std::string& imu_frame,
                              const std::string& lidar_frame) {
  if (has_imu_lidar_extrinsic_) return true;
  try {
    geometry_msgs::msg::TransformStamped tf_msg_il =
        tf_buffer.lookupTransform(imu_frame, lidar_frame, tf2::TimePointZero);
    geometry_msgs::msg::TransformStamped tf_msg_map =
        tf_buffer.lookupTransform(imu_frame, "map", tf2::TimePointZero);
    t_il = Eigen::Vector3d(tf_msg_il.transform.translation.x,
                           tf_msg_il.transform.translation.y,
                           tf_msg_il.transform.translation.z);

    q_il = Eigen::Quaterniond(
        tf_msg_il.transform.rotation.w, tf_msg_il.transform.rotation.x,
        tf_msg_il.transform.rotation.y, tf_msg_il.transform.rotation.z);
    q_il.normalize();

    q_i_map = Eigen::Quaterniond(
        tf_msg_map.transform.rotation.w, tf_msg_map.transform.rotation.x,
        tf_msg_map.transform.rotation.y, tf_msg_map.transform.rotation.z);
    q_i_map.normalize();

    geometry_msgs::msg::TransformStamped tf_pos_lidar =
        tf_buffer.lookupTransform(pos_antena_frame, lidar_frame,
                                  tf2::TimePointZero);

    t_pos_lidar = Eigen::Vector3d(tf_pos_lidar.transform.translation.x,
                                  tf_pos_lidar.transform.translation.y,
                                  tf_pos_lidar.transform.translation.z);

    q_pos_lidar = Eigen::Quaterniond(
        tf_pos_lidar.transform.rotation.w, tf_pos_lidar.transform.rotation.x,
        tf_pos_lidar.transform.rotation.y, tf_pos_lidar.transform.rotation.z);
    q_pos_lidar.normalize();
    has_imu_lidar_extrinsic_ = true;
    return true;
  } catch (const tf2::TransformException& ex) {
    RCLCPP_WARN(this->get_logger(), "Failed to get TF %s <- %s: %s",
                imu_frame.c_str(), lidar_frame.c_str(), ex.what());
    return false;
  }
}

bool GALONode::TryInitializeOdomFromGnss() {
  if (has_lidar_odom_initialized_from_gnss_) {
    return true;
  }

  if (!GetExtrinsicTf(*tf_buffer_, imu_frame, lidar_frame)) {
    return false;
  }

  Eigen::Vector3d gnss_local;
  Eigen::Matrix3d R_map_base = Eigen::Matrix3d::Identity();

  {
    std::lock_guard<std::mutex> lock(mut_);

    if (!gnss_data_.has_gnss_position_ || !gnss_data_.has_gnss_yaw ||
        !has_latest_R_map_base_) {
      return false;
    }

    gnss_local = gnss_data_.gnss_local_;
    R_map_base = latest_R_map_base_;
  }

  // Static TF:
  //
  //   pos_antenna <- lidar
  Eigen::Matrix3d R_pos_lidar = q_pos_lidar.toRotationMatrix();

  // Initial LiDAR pose:
  //
  //   map <- lidar = map <- pos_antenna * pos_antenna <- lidar
  R_map_lidar_ = R_map_base * R_pos_lidar;
  t_map_lidar_ = gnss_local + R_map_base * t_pos_lidar;

  has_lidar_odom_initialized_from_gnss_ = true;

  auto [base_roll, base_pitch, base_yaw] = EulersFromMatrixSimple(R_map_base);
  auto [lidar_roll, lidar_pitch, lidar_yaw] =
      EulersFromMatrixSimple(R_map_lidar_);

  RCLCPP_INFO(this->get_logger(),
              "Initialized LiDAR odometry from GNSS+IMU: "
              "t=[%.3f %.3f %.3f], "
              "base_rpy=[%.3f %.3f %.3f] deg, "
              "lidar_rpy=[%.3f %.3f %.3f] deg",
              t_map_lidar_.x(), t_map_lidar_.y(), t_map_lidar_.z(),
              base_roll * 180.0 / M_PI, base_pitch * 180.0 / M_PI,
              base_yaw * 180.0 / M_PI, lidar_roll * 180.0 / M_PI,
              lidar_pitch * 180.0 / M_PI, lidar_yaw * 180.0 / M_PI);

  return true;
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

void GALONode::GnssCb(const qarl_msgs::msg::NmeaGGA::SharedPtr msg) {
  // Only use high-quality GNSS fixes.
  // gps_qual >= 4 usually means RTK fixed / high-confidence solution.
  if (msg->gps_qual < 4) {
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
  if (!GetExtrinsicTf(*tf_buffer_, imu_frame, lidar_frame) ||
      imu_orientation_queue_.Size() < 2) {
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

    if (std::abs(yaw_time - pos_time) > 0.3) {
      RCLCPP_WARN(this->get_logger(),
                  "GNSS position/yaw timestamp mismatch: %.3f s",
                  std::abs(yaw_time - pos_time));
      return;
    }

    // Position of pos_antenna in the local GNSS/map frame.
    gnss_local = gnss_data_.gnss_local_;
  }

  // --------------------------------------------------------------------------
  // 1. Convert Trimble heading to ROS ENU yaw
  // --------------------------------------------------------------------------
  //
  // Trimble heading convention:
  //   heading = clockwise from North
  //
  // ROS ENU yaw convention:
  //   yaw = counter-clockwise from East
  double gnss_yaw = gnss_converter_.YawFromGnssHeading(msg->orientation.yaw);
  // RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
  //                      "GNSS yaw raw=%.3f deg, converted=%.3f rad %.3f deg",
  //                      msg->orientation.yaw, gnss_yaw, gnss_yaw * 180.0 /
  //                      M_PI);

  // --------------------------------------------------------------------------
  // 3. Get roll/pitch from IMU
  // --------------------------------------------------------------------------
  //
  // q_closest is the IMU orientation at the GNSS heading timestamp.
  //
  // q_i_map is your static correction from TF:
  //
  //   imu <- map
  //
  // This was already used in your earlier code to make the IMU orientation
  // compatible with your map convention.
  //
  // Result:
  //
  //   R_map_imu_like
  //
  // From this, we only keep roll and pitch.
  // GNSS yaw replaces the IMU yaw.
  Eigen::Matrix3d R_map_imu_like =
      q_i_map.toRotationMatrix() * q_closest->toRotationMatrix();

  auto [roll_imu, pitch_imu, yaw_imu_unused] =
      EulersFromMatrixSimple(R_map_imu_like);

  // auto [roll_imu, pitch_imu, yaw_imu_unused] =
  //     EulersFromMatrixSimple(q_closest->toRotationMatrix());

  (void)yaw_imu_unused;

  // --------------------------------------------------------------------------
  // 4. Build map <- base orientation
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
  Eigen::AngleAxisd roll_rot(roll_imu, Eigen::Vector3d::UnitX());
  Eigen::AngleAxisd pitch_rot(pitch_imu, Eigen::Vector3d::UnitY());
  Eigen::AngleAxisd yaw_rot(gnss_yaw, Eigen::Vector3d::UnitZ());

  Eigen::Matrix3d R_map_base = yaw_rot.toRotationMatrix() *
                               pitch_rot.toRotationMatrix() *
                               roll_rot.toRotationMatrix();

  // --------------------------------------------------------------------------
  // 5. Convert GNSS antenna pose to LiDAR pose
  // --------------------------------------------------------------------------
  //
  // Static TF was loaded as:
  //
  //   pos_antenna <- lidar
  //
  // Therefore:
  //
  //   p_pos = R_pos_lidar * p_lidar + t_pos_lidar
  //
  // For poses:
  //
  //   R_map_lidar = R_map_pos * R_pos_lidar
  //   t_map_lidar = t_map_pos + R_map_pos * t_pos_lidar
  //
  // Since pos_antenna has identity rotation relative to base_link in TF,
  // we can use:
  //
  //   R_map_pos = R_map_base
  //
  // and:
  //
  //   t_map_pos = gnss_local
  Eigen::Matrix3d R_pos_lidar = q_pos_lidar.toRotationMatrix();
  Eigen::Vector3d t_pos_lidar_local = t_pos_lidar;

  Eigen::Matrix3d R_map_lidar_gt = R_map_base * R_pos_lidar;
  Eigen::Vector3d t_map_lidar_gt = gnss_local + R_map_base * t_pos_lidar_local;

  // --------------------------------------------------------------------------
  // 5. Publish LiDAR GT pose
  // --------------------------------------------------------------------------
  Eigen::Vector<double, 6> sigmas = Eigen::Vector<double, 6>(
      node_params_.gt_cov_.x_precision, node_params_.gt_cov_.y_precision,
      node_params_.gt_cov_.z_precision,
      node_params_.gt_cov_.roll_precision * M_PI / 180.0,
      node_params_.gt_cov_.pitch_precision * M_PI / 180.0,
      node_params_.gt_cov_.yaw_precision * M_PI / 180.0);
  geometry_msgs::msg::PoseWithCovarianceStamped pose_msg =
      BuildPoseWithCovarianceMsg(msg->header.stamp, "gnss_map", R_map_lidar_gt,
                                 t_map_lidar_gt, sigmas);
  gnss_imu_pose_pub_->publish(pose_msg);

  auto [roll_gt, pitch_gt, yaw_gt] = EulersFromMatrixSimple(R_map_lidar_gt);
  geometry_msgs::msg::Point eulers_msg;
  eulers_msg.x = roll_gt;
  eulers_msg.y = pitch_gt;
  eulers_msg.z = NormalizeAngle0To2Pi(yaw_gt);
  gt_eulers_pub_->publish(eulers_msg);

  // --------------------------------------------------------------------------
  // 7. Store GNSS yaw for odometry initialization
  // --------------------------------------------------------------------------
  // TryInitializeOdomFromGnss() should use this yaw as map <- base/pos_antenna,
  // and then compose with pos_antenna <- lidar.
  {
    std::lock_guard<std::mutex> lock(mut_);

    gnss_data_.yaw = gnss_yaw;
    gnss_data_.has_gnss_yaw = true;

    latest_R_map_base_ = R_map_base;
    has_latest_R_map_base_ = true;
  }
}

void GALONode::LidarCb(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
  // Main LiDAR pipeline:
  //
  //   1. Deskew cloud
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

  std::optional<sensor_msgs::msg::PointCloud2> deskewed;

  {
    TimeMeasurments_t meas("deskew");
    std::lock_guard<std::mutex> lock(mut_);

    deskew_algo_.UpdateLidarQueue(msg);
    deskewed = deskew_algo_.ProcessCloudsQueue();

    meas.SetEnd();
    time_measurments.push_back(meas);
  }

  if (!deskewed) return;
  SegmentationResult segmentation_result;
  deskew_cld_pub_->publish(*deskewed);
  {
    TimeMeasurments_t meas("segmentaion");
    segmentation_result = segmentation_.Classify(*deskewed);
    meas.SetEnd();
    time_measurments.push_back(meas);
    if (node_params_.debug) {
      auto colored_cld = segmentation_.MakeColoredCloud(segmentation_result);
      colored_pub_->publish(colored_cld);
    }
  }
  std::vector<Eigen::Vector2d> cur_planar_points;
  {
    TimeMeasurments_t meas("objects_extration");
    cur_planar_points = planar_registration_.ExtractPoints(
        *deskewed, segmentation_result.labels);
    if (cur_planar_points.size()) {
      cur_planar_points = planar_registration_.Filter(cur_planar_points);
    }
    meas.SetEnd();
    time_measurments.push_back(meas);
  }
  std::vector<GroundPatch> cur_patches;
  {
    TimeMeasurments_t meas("ground_extration");
    cur_patches = ground_patches_extractor_.Extract(*deskewed,
                                                    segmentation_result.labels);
    meas.SetEnd();
    time_measurments.push_back(meas);
  }

  if (node_params_.debug) {
    auto patches_marker =
        ground_patches_extractor_.MakeGroundPatchMarkers(deskewed->header);
    ground_patches_pub_->publish(patches_marker);
  }
  if (!TryInitializeOdomFromGnss()) {
    RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 1000,
        "Waiting for GNSS position/yaw to initialize LiDAR odometry");
    return;
  }
  if (ground_map_.empty() || objects_map_.empty()) {
    RCLCPP_WARN(this->get_logger(),
                "Either objects or ground map is empty %zu %zu",
                ground_map_.size(), objects_map_.size());

    auto patches_in_map =
        TransformPatchesToMap(cur_patches, R_map_lidar_, t_map_lidar_);
    ground_map_frames_.push_back(patches_in_map);
    RebuildGroundMap();

    auto planar_points_in_map =
        TransformPointsToMap(cur_planar_points, R_map_lidar_, t_map_lidar_);
    objects_map_frames_.push_back(planar_points_in_map);
    RebuildObjectsMap();
    RCLCPP_WARN(
        this->get_logger(),
        "Initialized first map frame at pose t=[%.3f %.3f %.3f], yaw=%.3f",
        t_map_lidar_.x(), t_map_lidar_.y(), t_map_lidar_.z(),
        std::atan2(R_map_lidar_(1, 0), R_map_lidar_(0, 0)));
    return;
  }
  if (!GetExtrinsicTf(*tf_buffer_, imu_frame, lidar_frame) ||
      imu_orientation_queue_.Size() < 2) {
    return;
  }
  GnssData gnss_data_local;
  {
    std::lock_guard<std::mutex> lock(mut_);
    gnss_data_local = gnss_data_;
  }

  double lidar_time = rclcpp::Time(deskewed->header.stamp).seconds();
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
  // q_il comes from lookupTransform(imu_frame, lidar_frame), so R_ex maps:
  // lidar frame -> imu frame.
  Eigen::Matrix3d R_ex = q_il.toRotationMatrix();
  // Convert IMU-frame delta into LiDAR-frame delta.
  // Result maps current LiDAR frame -> previous LiDAR frame.
  Eigen::Matrix3d R_lidar_delta = R_ex.transpose() * R_imu_delta_lidar * R_ex;
  imu_q_lidar_prev_ = *q_lidar;

  // Predicted absolute map rotation from previous pose + IMU delta
  Eigen::Matrix3d R_imu_prior_map = R_map_lidar_ * R_lidar_delta;

  // Planar initial guess from current global pose
  double yaw_initial = std::atan2(R_imu_prior_map(1, 0), R_imu_prior_map(0, 0));
  double c = std::cos(yaw_initial);
  double s = std::sin(yaw_initial);

  Eigen::Matrix2d R_planar_initial;
  R_planar_initial << c, -s, s, c;

  Eigen::Vector3d t_pred = t_map_lidar_;

  if (has_prev_lidar_pose_for_prediction_) {
    double dt = lidar_time - prev_lidar_pose_time_;
    if (dt > 1e-3 && dt < 0.5) {
      t_pred = t_map_lidar_ + velocity_map_lidar_ * dt;
    }
  }

  Eigen::Vector2d t_planar_initial(t_pred.x(), t_pred.y());

  PlanarRegistrationResult planar_reg_res;
  {
    TimeMeasurments_t meas("planar_registration");
    planar_reg_res = planar_registration_.Align(
        objects_map_, cur_planar_points, R_planar_initial, t_planar_initial);
    meas.SetEnd();
    time_measurments.push_back(meas);
  }
  GroundRegistrationResult ground_reg_result;
  {
    TimeMeasurments_t meas("ground_registration");
    ground_reg_result = ground_registration_.Align(
        ground_map_, cur_patches, R_imu_prior_map, R_map_lidar_, t_pred);
    meas.SetEnd();
    time_measurments.push_back(meas);
  }

  if (!planar_reg_res.valid) {
    RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 1000,
        "Planar registration invalid, skipping odometry update");
    return;
  }
  bool is_ground_ok = CheckGroundRegistration(ground_reg_result);

  Eigen::Matrix3d R_abs;
  Eigen::Vector3d t_abs;
  if (is_ground_ok) {
    R_abs = MergeGroundAndPlanarRotation(ground_reg_result.R, R_map_lidar_,
                                         planar_reg_res.R);
    t_abs = MergeGroundAndPlanarTranslation(ground_reg_result.t, t_map_lidar_,
                                            planar_reg_res.t);
  } else {
    // Ground failed: trust planar x/y/yaw, keep previous z, take roll/pitch
    // from IMU prior.
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

    t_abs.x() = planar_reg_res.t.x();
    t_abs.y() = planar_reg_res.t.y();
    t_abs.z() = t_map_lidar_.z();
  }
  R_map_lidar_ = R_abs;
  t_map_lidar_ = t_abs;
  double dt_pose = lidar_time - prev_lidar_pose_time_;
  if (has_prev_lidar_pose_for_prediction_ && dt_pose > 1e-3 && dt_pose < 0.5) {
    velocity_map_lidar_ = (t_abs - prev_t_map_lidar_) / dt_pose;
  }

  prev_t_map_lidar_ = t_abs;
  prev_lidar_pose_time_ = lidar_time;
  has_prev_lidar_pose_for_prediction_ = true;
  auto planar_points_in_map =
      TransformPointsToMap(cur_planar_points, R_map_lidar_, t_map_lidar_);

  objects_map_frames_.push_back(planar_points_in_map);

  while (objects_map_frames_.size() > node_params_.max_planar_map_frames) {
    objects_map_frames_.pop_front();
  }
  RebuildObjectsMap();

  if (cur_patches.size() >= 6 && planar_reg_res.valid) {
    auto patches_in_map =
        TransformPatchesToMap(cur_patches, R_map_lidar_, t_map_lidar_);

    ground_map_frames_.push_back(patches_in_map);

    while (ground_map_frames_.size() > node_params_.max_ground_map_frames) {
      ground_map_frames_.pop_front();
    }
    RebuildGroundMap();
  }

  const auto& [sigma_xy, sigma_yaw] =
      node_params_.est_cov_.GetXYYawSigmas(planar_reg_res);

  Eigen::Vector<double, 6> sigmas = Eigen::Vector<double, 6>(
      sigma_xy, sigma_xy, node_params_.est_cov_.z, node_params_.est_cov_.roll,
      node_params_.est_cov_.pitch, sigma_yaw);
  geometry_msgs::msg::PoseWithCovarianceStamped pose_msg =
      BuildPoseWithCovarianceMsg(msg->header.stamp, "gnss_map", R_abs, t_abs,
                                 sigmas);
  lidar_pose_pub_->publish(pose_msg);

  auto [roll_abs, pitch_abs, yaw_abs] = EulersFromMatrixSimple(R_abs);
  geometry_msgs::msg::Point eulers_msg;
  eulers_msg.x = roll_abs;
  eulers_msg.y = pitch_abs;
  eulers_msg.z = yaw_abs;
  est_eulers_pub_->publish(eulers_msg);
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

    deskew_algo_.UpdateImuQueue(msg->angular_velocity.z, msg_time.seconds());
  }
}

void GALONode::PrintTimeMeasurments(
    const std::vector<TimeMeasurments_t>& measurments) {
  if (measurments.empty()) return;
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
  RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "%s",
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
    ground_map_.insert(ground_map_.end(), frame.begin(), frame.end());
  }
}

void GALONode::RebuildObjectsMap() {
  objects_map_.clear();

  for (const auto& frame : objects_map_frames_) {
    objects_map_.insert(objects_map_.end(), frame.begin(), frame.end());
  }
}

Eigen::Matrix3d GALONode::MergeGroundAndPlanarRotation(
    const Eigen::Matrix3d& R_ground, const Eigen::Matrix3d& R_prior,
    const Eigen::Matrix2d& R_planar, double alpha_rp) {
  double roll, pitch, dummy_yaw;
  auto [r_prior, p_prior, y_prior] = EulersFromMatrixSimple(R_prior);
  auto [r_ground, p_ground, y_ground] = EulersFromMatrixSimple(R_ground);

  double r_final = r_prior + alpha_rp * NormalizeAngle(r_ground - r_prior);
  double p_final = p_prior + alpha_rp * NormalizeAngle(p_ground - p_prior);

  double yaw = std::atan2(R_planar(1, 0), R_planar(0, 0));

  Eigen::AngleAxisd roll_rot(r_final, Eigen::Vector3d::UnitX());
  Eigen::AngleAxisd pitch_rot(p_final, Eigen::Vector3d::UnitY());
  Eigen::AngleAxisd yaw_rot(yaw, Eigen::Vector3d::UnitZ());

  return yaw_rot.toRotationMatrix() * pitch_rot.toRotationMatrix() *
         roll_rot.toRotationMatrix();
}

Eigen::Vector3d GALONode::MergeGroundAndPlanarTranslation(
    const Eigen::Vector3d& t_ground, const Eigen::Vector3d& t_prior,
    const Eigen::Vector2d& t_planar, double alpha_z) {
  Eigen::Vector3d t;
  t.x() = t_planar.x();
  t.y() = t_planar.y();

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
  double dz = std::abs(res.t.z() - t_map_lidar_.z());

  return droll < DEG2RAD(ground_reg_gate_params_.max_droll) &&
         dpitch < DEG2RAD(ground_reg_gate_params_.max_dpitch) &&
         dz < ground_reg_gate_params_.max_dz;
}

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<GALONode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}