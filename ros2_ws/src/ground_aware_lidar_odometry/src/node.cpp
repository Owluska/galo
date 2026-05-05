#include "ground_aware_lidar_odometry/node.hpp"

#include <functional>

GALONode::GALONode(const GALONodeParams& node_params)
    : Node("GALONode"),
      node_params_(node_params),
      segmentation_(segementation_params_),
      ground_patches_extractor_(ground_patch_params_),
      ground_registration_(ground_registration_params_, this->get_logger(),
                           *this->get_clock()),
      planar_registration_(planar_registration_params_, this->get_logger(),
                           *this->get_clock()),
      deskew_algo_(deskew_prms_, this->get_logger(), *this->get_clock()) {
  imu_orientation_queue_.Resize(2000);
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
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

  deskew_cld_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
      "/GALO/deskewed_cloud", 1);
  colored_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
      "/GALO/colored_cloud", 1);
  ground_patches_pub_ =
      this->create_publisher<visualization_msgs::msg::MarkerArray>(
          "/GALO/ground_patch_normals", 10);
  translation_pub_ =
      this->create_publisher<geometry_msgs::msg::Point>("/GALO/translation", 1);
  eulers_pub_ =
      this->create_publisher<geometry_msgs::msg::Point>("/GALO/eulers", 1);
  imu_eulers_pub_ =
      this->create_publisher<geometry_msgs::msg::Point>("/GALO/imu_eulers", 1);

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
    has_imu_lidar_extrinsic_ = true;
    return true;
  } catch (const tf2::TransformException& ex) {
    RCLCPP_WARN(this->get_logger(), "Failed to get TF %s <- %s: %s",
                imu_frame.c_str(), lidar_frame.c_str(), ex.what());
    return false;
  }
}

void GALONode::GnssCb(const qarl_msgs::msg::NmeaGGA::SharedPtr msg) {
  if (msg->gps_qual < 4) return;
  double northing = 0.0;
  double easting = 0.0;
  std::string zone;

  robot_localization::navsat_conversions::LLtoUTM(msg->lat, msg->lon, northing,
                                                  easting, zone);

  Eigen::Vector3d gnss_utm(easting, northing, msg->altitude);
  std::lock_guard<std::mutex> lock(mut_);

  if (!gnss_data_.has_gnss_origin_) {
    gnss_data_.gnss_origin_ = gnss_utm;
    gnss_data_.has_gnss_origin_ = true;
  }

  gnss_data_.gnss_local_ = gnss_utm - gnss_data_.gnss_origin_;
  gnss_data_.has_gnss_position_ = true;
}
void GALONode::GnssYawCb(
    const qarl_msgs::msg::OrientationStamped::SharedPtr msg) {
  if (!GetExtrinsicTf(*tf_buffer_, imu_frame, lidar_frame) ||
      imu_orientation_queue_.Size() < 2 || !gnss_data_.has_gnss_origin_) {
    return;
  }
  double gnss_time = rclcpp::Time(msg->header.stamp).seconds();

  std::optional<Eigen::Quaterniond> q_closest;
  {
    std::lock_guard<std::mutex> lock(mut_);
    q_closest = GetImuOrientationAt(gnss_time);
  }

  if (!q_closest) {
    PrintTimeMeasurments(time_measurments);
    return;
  }

  Eigen::Vector3d gnss_local;
  {
    std::lock_guard<std::mutex> lock(mut_);
    gnss_local = gnss_data_.gnss_local_;
  }
  auto R_map_imu_correction = q_i_map.toRotationMatrix();
  auto R_imu_map = R_map_imu_correction * q_closest->toRotationMatrix();
  auto [roll, pitch, yaw] = EulersFromMatrixSimple(R_imu_map);
  geometry_msgs::msg::PoseWithCovarianceStamped pose_msg;
  pose_msg.header.stamp = msg->header.stamp;
  pose_msg.header.frame_id = "gnss_map";
  pose_msg.pose.pose.position.x = gnss_local.x();
  pose_msg.pose.pose.position.y = gnss_local.y();
  pose_msg.pose.pose.position.z = gnss_local.z();

  double heading_deg = msg->orientation.yaw;
  double heading_rad = heading_deg * M_PI / 180.0;

  // Trimble: clockwise from North
  // ROS ENU: counter-clockwise from East
  double yaw_enu = M_PI / 2.0 - heading_rad;

  // normalize to [-pi, pi]
  yaw_enu = std::atan2(std::sin(yaw_enu), std::cos(yaw_enu));

  tf2::Quaternion q;
  q.setRPY(roll, pitch, yaw_enu);
  q.normalize();
  pose_msg.pose.pose.orientation = tf2::toMsg(q);
  auto& cov = pose_msg.pose.covariance;
  std::fill(cov.begin(), cov.end(), 0.0);

  cov[0] = node_params_.gt_cov_.x_precision * node_params_.gt_cov_.x_precision;
  cov[7] = node_params_.gt_cov_.y_precision * node_params_.gt_cov_.y_precision;
  cov[14] = node_params_.gt_cov_.z_precision * node_params_.gt_cov_.z_precision;

  cov[21] = (node_params_.gt_cov_.roll_precision * M_PI / 180.0) *
            (node_params_.gt_cov_.roll_precision * M_PI / 180.0);
  cov[28] = (node_params_.gt_cov_.pitch_precision * M_PI / 180.0) *
            (node_params_.gt_cov_.pitch_precision * M_PI / 180.0);
  cov[35] = (node_params_.gt_cov_.yaw_precision * M_PI / 180.0) *
            (node_params_.gt_cov_.yaw_precision * M_PI / 180.0);
  gnss_imu_pose_pub_->publish(pose_msg);
  std::lock_guard<std::mutex> lock(mut_);
  gnss_data_.yaw = yaw_enu;
  gnss_data_.has_gnss_yaw = true;
}

void GALONode::LidarCb(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
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

  deskew_cld_pub_->publish(*deskewed);

  TimeMeasurments_t seg_meas("segmentaion");
  auto segmentation_result = segmentation_.Classify(*deskewed);
  seg_meas.SetEnd();
  time_measurments.push_back(seg_meas);

  if (node_params_.debug) {
    auto colored_cld = segmentation_.MakeColoredCloud(segmentation_result);
    colored_pub_->publish(colored_cld);
  }
  TimeMeasurments_t objects_extraction_meas("objects_extration");
  auto cur_planar_points =
      planar_registration_.ExtractPoints(*deskewed, segmentation_result.labels);
  if (cur_planar_points.size()) {
    cur_planar_points = planar_registration_.Filter(cur_planar_points);
  }
  objects_extraction_meas.SetEnd();
  time_measurments.push_back(objects_extraction_meas);
  TimeMeasurments_t extraction_meas("ground_extration");
  std::vector<GroundPatch> cur_patches =
      ground_patches_extractor_.Extract(*deskewed, segmentation_result.labels);
  extraction_meas.SetEnd();
  time_measurments.push_back(extraction_meas);

  if (node_params_.debug) {
    auto patches_marker =
        ground_patches_extractor_.MakeGroundPatchMarkers(deskewed->header);
    ground_patches_pub_->publish(patches_marker);
  }
  if (ground_map_.empty() || objects_map_.empty()) {
    RCLCPP_WARN(this->get_logger(),
                "Either objects or ground map is empty %zu %zu",
                ground_map_.size(), objects_map_.size());
    auto patches_in_map = TransformPatchesToMap(
        cur_patches, Eigen::Matrix3d::Identity(), Eigen::Vector3d::Zero());
    ground_map_frames_.push_back(patches_in_map);
    RebuildGroundMap();

    auto planar_points_in_map =
        TransformPointsToMap(cur_planar_points, Eigen::Matrix3d::Identity(),
                             Eigen::Vector3d::Zero());
    objects_map_frames_.push_back(planar_points_in_map);
    RebuildObjectsMap();
    PrintTimeMeasurments(time_measurments);
    return;
  }
  if (!GetExtrinsicTf(*tf_buffer_, imu_frame, lidar_frame) ||
      imu_orientation_queue_.Size() >= 2) {
    PrintTimeMeasurments(time_measurments);
    return;
  }
  GnssData gnss_data_local;
  {
    std::lock_guard<std::mutex> lock(mut_);
    gnss_data_local = gnss_data_;
  }

  if (!gnss_data_local.has_gnss_position_ || !gnss_data_local.has_gnss_yaw) {
    PrintTimeMeasurments(time_measurments);
    return;
  }
  double lidar_time = rclcpp::Time(deskewed->header.stamp).seconds();
  std::optional<Eigen::Quaterniond> q_lidar;
  {
    std::lock_guard<std::mutex> lock(mut_);
    q_lidar = GetImuOrientationAt(lidar_time);
  }

  if (!q_lidar) {
    PrintTimeMeasurments(time_measurments);
    return;
  }

  if (!has_lidar_imu_prev_) {
    imu_q_lidar_prev_ = *q_lidar;
    has_lidar_imu_prev_ = true;
    PrintTimeMeasurments(time_measurments);
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
  double yaw_initial = std::atan2(R_map_lidar_(1, 0), R_map_lidar_(0, 0));
  double c = std::cos(yaw_initial);
  double s = std::sin(yaw_initial);

  Eigen::Matrix2d R_planar_initial;
  R_planar_initial << c, -s, s, c;

  Eigen::Vector2d t_planar_initial(t_map_lidar_.x(), t_map_lidar_.y());

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
        ground_map_, cur_patches, R_imu_prior_map, R_map_lidar_, t_map_lidar_);
    meas.SetEnd();
    time_measurments.push_back(meas);
  }

  if (ground_reg_result.valid && planar_reg_res.valid) {
    Eigen::Matrix3d R_map_lidar_prev = R_map_lidar_;
    Eigen::Vector3d t_map_lidar_prev = t_map_lidar_;

    Eigen::Matrix3d R_abs =
        MergeGroundAndPlanarRotation(ground_reg_result.R, planar_reg_res.R);

    Eigen::Vector3d t_abs =
        MergeGroundAndPlanarTranslation(ground_reg_result.t, planar_reg_res.t);

    geometry_msgs::msg::PoseWithCovarianceStamped pose_msg;
    pose_msg.header.frame_id = "lidar_odometry";
    pose_msg.header.stamp = msg->header.stamp;

    pose_msg.pose.pose.position.x = t_abs.x();
    pose_msg.pose.pose.position.y = t_abs.y();
    pose_msg.pose.pose.position.z = t_abs.z();
    auto [roll_abs, pitch_abs, yaw_abs] = EulersFromMatrixSimple(R_abs);
    tf2::Quaternion q;
    q.setRPY(roll_abs, pitch_abs, yaw_abs);
    q.normalize();
    pose_msg.pose.pose.orientation = tf2::toMsg(q);

    auto scale = 1;
    // worse if residual high
    scale *= std::clamp(planar_reg_res.mean_residual / 0.1, 1.0, 5.0);
    // worse if few matches
    scale *= std::clamp(100.0 / std::max(planar_reg_res.matches, 1), 1.0, 3.0);

    double sigma_xy = node_params_.est_cov_.base_xy * scale;
    double sigma_yaw = 2.0 * M_PI / 180.0 * scale;  // 2 deg base
    auto& cov = pose_msg.pose.covariance;
    cov[0] = sigma_xy * sigma_xy;
    cov[7] = sigma_xy * sigma_xy;
    cov[14] = node_params_.est_cov_.z *
              node_params_.est_cov_.z;  // z less reliable in LiDAR planar

    cov[21] = node_params_.est_cov_.roll;   // roll (weak)
    cov[28] = node_params_.est_cov_.pitch;  // pitch
    cov[35] = sigma_yaw * sigma_yaw;
    lidar_pose_pub_->publish(pose_msg);

    // if (node_params_.debug) {
    //
    //   auto [roll_ground, pitch_ground, yaw_ground] =
    //       EulersFromMatrixSimple(ground_reg_result.R);

    //   double planar_yaw =
    //       std::atan2(planar_reg_res.R(1, 0), planar_reg_res.R(0, 0));

    //   Eigen::Matrix3d R_debug_delta = R_abs *
    //   R_map_lidar_prev.transpose(); Eigen::Vector3d t_debug_delta =
    //       t_abs - R_debug_delta * t_map_lidar_prev;
    //   RCLCPP_INFO_THROTTLE(
    //       this->get_logger(), *this->get_clock(), 500,
    //       "GALO pose | t=[%.3f %.3f %.3f] | delta=[%.3f %.3f %.3f] | "
    //       "dist_xy=%.3f dz=%.3f",
    //       t_map_lidar_.x(), t_map_lidar_.y(), t_map_lidar_.z(),
    //       t_debug_delta.x(), t_debug_delta.y(), t_debug_delta.z(),
    //       std::hypot(t_map_lidar_.x(), t_map_lidar_.y()),
    //       t_map_lidar_.z());
    //   RCLCPP_INFO_THROTTLE(
    //       this->get_logger(), *this->get_clock(), 500,
    //       "Ground result | valid=%d matches=%d mean_abs_res=%.4f "
    //       "t_ground=[%.3f %.3f %.3f] rpy_ground=[%.5f %.5f %.5f]",
    //       ground_reg_result.valid, ground_reg_result.num_matches,
    //       ground_reg_result.mean_abs_residual, ground_reg_result.t.x(),
    //       ground_reg_result.t.y(), ground_reg_result.t.z(),
    //       roll_ground, pitch_ground, yaw_ground);
    //   geometry_msgs::msg::Point translation_msg;
    //   translation_msg.x = t_debug_delta.x();
    //   translation_msg.y = t_debug_delta.y();
    //   translation_msg.z = t_debug_delta.z();
    //   translation_pub_->publish(translation_msg);

    //   geometry_msgs::msg::Point eulers_msg;
    //   auto [roll, pitch, yaw] =
    //   EulersFromMatrixSimple(R_debug_delta); eulers_msg.x = roll;
    //   eulers_msg.y = pitch;
    //   eulers_msg.z = yaw;
    //   eulers_pub_->publish(eulers_msg);

    //   geometry_msgs::msg::Point eulers_imu_msg;
    //   auto [imu_r, imu_p, imu_y] =
    //   EulersFromMatrixSimple(R_lidar_delta);
    //   RCLCPP_INFO_THROTTLE(this->get_logger(),
    //   *this->get_clock(), 500,
    //                        "IMU delta | rpy=[%.5f %.5f %.5f]",
    //                        imu_r, imu_p, imu_y);
    //   eulers_imu_msg.x = imu_r;
    //   eulers_imu_msg.y = imu_p;
    //   eulers_imu_msg.z = imu_y;
    //   imu_eulers_pub_->publish(eulers_imu_msg);
    // }
    R_map_lidar_ = R_abs;
    t_map_lidar_ = t_abs;

    auto patches_in_map =
        TransformPatchesToMap(cur_patches, R_map_lidar_, t_map_lidar_);

    ground_map_frames_.push_back(patches_in_map);

    while (ground_map_frames_.size() > node_params_.max_ground_map_frames_) {
      ground_map_frames_.pop_front();
    }

    auto planar_points_in_map =
        TransformPointsToMap(cur_planar_points, R_map_lidar_, t_map_lidar_);

    objects_map_frames_.push_back(planar_points_in_map);

    while (objects_map_frames_.size() > node_params_.max_planar_map_frames_) {
      objects_map_frames_.pop_front();
    }

    RebuildGroundMap();
    RebuildObjectsMap();
  }
  PrintTimeMeasurments(time_measurments);
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

std::tuple<double, double, double> GALONode::EulersFromMatrixSimple(
    const Eigen::Matrix3d& R) {
  double roll = std::atan2(R(2, 1), R(2, 2));
  double pitch =
      std::atan2(-R(2, 0), std::sqrt(R(2, 1) * R(2, 1) + R(2, 2) * R(2, 2)));
  double yaw = std::atan2(R(1, 0), R(0, 0));
  return std::make_tuple(roll, pitch, yaw);
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
    const Eigen::Matrix3d& R_ground, const Eigen::Matrix2d& R_planar) {
  double roll, pitch, dummy_yaw;
  std::tie(roll, pitch, dummy_yaw) = EulersFromMatrixSimple(R_ground);

  double yaw = std::atan2(R_planar(1, 0), R_planar(0, 0));

  Eigen::AngleAxisd roll_rot(roll, Eigen::Vector3d::UnitX());
  Eigen::AngleAxisd pitch_rot(pitch, Eigen::Vector3d::UnitY());
  Eigen::AngleAxisd yaw_rot(yaw, Eigen::Vector3d::UnitZ());

  return yaw_rot.toRotationMatrix() * pitch_rot.toRotationMatrix() *
         roll_rot.toRotationMatrix();
}

Eigen::Vector3d GALONode::MergeGroundAndPlanarTranslation(
    const Eigen::Vector3d& t_ground, const Eigen::Vector2d& t_planar) {
  Eigen::Vector3d t;
  t.x() = t_planar.x();
  t.y() = t_planar.y();
  t.z() = t_ground.z();
  return t;
}

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  GALONodeParams prms;
  auto node = std::make_shared<GALONode>(prms);
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}