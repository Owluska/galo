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
}

bool GALONode::GetExtrinsicTf(tf2_ros::Buffer& tf_buffer,
                              const std::string& imu_frame,
                              const std::string& lidar_frame) {
  if (has_imu_lidar_extrinsic_) return true;
  try {
    geometry_msgs::msg::TransformStamped tf_msg =
        tf_buffer.lookupTransform(imu_frame, lidar_frame, tf2::TimePointZero);

    t_il = Eigen::Vector3d(tf_msg.transform.translation.x,
                           tf_msg.transform.translation.y,
                           tf_msg.transform.translation.z);

    q_il = Eigen::Quaterniond(
        tf_msg.transform.rotation.w, tf_msg.transform.rotation.x,
        tf_msg.transform.rotation.y, tf_msg.transform.rotation.z);
    q_il.normalize();
    has_imu_lidar_extrinsic_ = true;
    return true;
  } catch (const tf2::TransformException& ex) {
    RCLCPP_WARN(this->get_logger(), "Failed to get TF %s <- %s: %s",
                imu_frame.c_str(), lidar_frame.c_str(), ex.what());
    return false;
  }
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
  if (GetExtrinsicTf(*tf_buffer_, imu_frame, lidar_frame) &&
      imu_orientation_queue_.Size() >= 2) {
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

    Eigen::Matrix3d R_imu_delta_lidar =
        imu_q_lidar_prev_.toRotationMatrix().transpose() *
        q_lidar->toRotationMatrix();
    Eigen::Matrix3d R_ex = q_il.toRotationMatrix();
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
      ground_reg_result =
          ground_registration_.Align(ground_map_, cur_patches, R_imu_prior_map,
                                     R_map_lidar_, t_map_lidar_);
      meas.SetEnd();
      time_measurments.push_back(meas);
    }
    // RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 500,
    //                      "Reg debug | "
    //                      "ground: valid=%d matches=%d mean_abs_res=%.4f | "
    //                      "planar: valid=%d matches=%d mean_res=%.4f | "
    //                      "map sizes: ground=%zu objects=%zu | "
    //                      "cur sizes: ground=%zu objects=%zu",
    //                      ground_reg_result.valid,
    //                      ground_reg_result.num_matches,
    //                      ground_reg_result.mean_abs_residual,
    //                      planar_reg_res.valid, planar_reg_res.matches,
    //                      planar_reg_res.mean_residual, ground_map_.size(),
    //                      objects_map_.size(), cur_patches.size(),
    //                      cur_planar_points.size());
    if (ground_reg_result.valid && planar_reg_res.valid) {
      Eigen::Matrix3d R_map_lidar_prev = R_map_lidar_;
      Eigen::Vector3d t_map_lidar_prev = t_map_lidar_;

      Eigen::Matrix3d R_abs =
          MergeGroundAndPlanarRotation(ground_reg_result.R, planar_reg_res.R);

      Eigen::Vector3d t_abs = MergeGroundAndPlanarTranslation(
          ground_reg_result.t, planar_reg_res.t);
      auto [roll_abs, pitch_abs, yaw_abs] = EulersFromMatrixSimple(R_abs);
      auto [roll_ground, pitch_ground, yaw_ground] =
          EulersFromMatrixSimple(ground_reg_result.R);

      double planar_yaw =
          std::atan2(planar_reg_res.R(1, 0), planar_reg_res.R(0, 0));

      // RCLCPP_INFO_THROTTLE(
      //     this->get_logger(), *this->get_clock(), 500,
      //     "Pose debug | "
      //     "ground_rpy=[%.5f %.5f %.5f] planar_yaw=%.5f | "
      //     "abs_rpy=[%.5f %.5f %.5f] | "
      //     "ground_t=[%.3f %.3f %.3f] planar_t=[%.3f %.3f] abs_t=[%.3f %.3f "
      //     "%.3f]",
      //     roll_ground, pitch_ground, yaw_ground, planar_yaw, roll_abs,
      //     pitch_abs, yaw_abs, ground_reg_result.t.x(),
      //     ground_reg_result.t.y(), ground_reg_result.t.z(),
      //     planar_reg_res.t.x(), planar_reg_res.t.y(), t_abs.x(), t_abs.y(),
      //     t_abs.z());
      Eigen::Matrix3d R_debug_delta = R_abs * R_map_lidar_prev.transpose();
      Eigen::Vector3d t_debug_delta = t_abs - R_debug_delta * t_map_lidar_prev;
      if (node_params_.debug) {
        RCLCPP_INFO_THROTTLE(
            this->get_logger(), *this->get_clock(), 500,
            "GALO pose | t=[%.3f %.3f %.3f] | delta=[%.3f %.3f %.3f] | "
            "dist_xy=%.3f dz=%.3f",
            t_map_lidar_.x(), t_map_lidar_.y(), t_map_lidar_.z(),
            t_debug_delta.x(), t_debug_delta.y(), t_debug_delta.z(),
            std::hypot(t_map_lidar_.x(), t_map_lidar_.y()), t_map_lidar_.z());
        RCLCPP_INFO_THROTTLE(
            this->get_logger(), *this->get_clock(), 500,
            "Ground result | valid=%d matches=%d mean_abs_res=%.4f "
            "t_ground=[%.3f %.3f %.3f] rpy_ground=[%.5f %.5f %.5f]",
            ground_reg_result.valid, ground_reg_result.num_matches,
            ground_reg_result.mean_abs_residual, ground_reg_result.t.x(),
            ground_reg_result.t.y(), ground_reg_result.t.z(), roll_ground,
            pitch_ground, yaw_ground);
        geometry_msgs::msg::Point translation_msg;
        translation_msg.x = t_debug_delta.x();
        translation_msg.y = t_debug_delta.y();
        translation_msg.z = t_debug_delta.z();
        translation_pub_->publish(translation_msg);

        geometry_msgs::msg::Point eulers_msg;
        auto [roll, pitch, yaw] = EulersFromMatrixSimple(R_debug_delta);
        eulers_msg.x = roll;
        eulers_msg.y = pitch;
        eulers_msg.z = yaw;
        eulers_pub_->publish(eulers_msg);

        geometry_msgs::msg::Point eulers_imu_msg;
        auto [imu_r, imu_p, imu_y] = EulersFromMatrixSimple(R_lidar_delta);
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 500,
                             "IMU delta | rpy=[%.5f %.5f %.5f]", imu_r, imu_p,
                             imu_y);
        eulers_imu_msg.x = imu_r;
        eulers_imu_msg.y = imu_p;
        eulers_imu_msg.z = imu_y;
        imu_eulers_pub_->publish(eulers_imu_msg);
      }
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