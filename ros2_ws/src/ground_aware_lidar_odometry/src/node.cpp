#include "ground_aware_lidar_odometry/node.hpp"

#include <functional>

GALONode::GALONode(const GALONodeParams& node_params)
    : Node("GALONode"),
      node_params_(node_params),
      segmentation_(segementation_params_),
      ground_patches_extractor_(ground_patch_params_),
      ground_registration_(ground_registration_params_),
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
    RCLCPP_WARN(this->get_logger(), "Failed to get TF %s <- %s: %s", imu_frame,
                lidar_frame, ex.what());
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
  if (ground_map_.empty()) {
    auto patches_in_map = TransformPatchesToMap(
        cur_patches, Eigen::Matrix3d::Identity(), Eigen::Vector3d::Zero());
    ground_map_frames_.push_back(patches_in_map);
    RebuildGroundMap();
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
    TimeMeasurments_t registration_meas("ground_registration");
    auto reg_result =
        ground_registration_.Align(ground_map_, cur_patches, R_lidar_delta);
    registration_meas.SetEnd();
    time_measurments.push_back(registration_meas);
    if (reg_result.valid) {
      // compose global local-map pose
      R_map_lidar_ = reg_result.R * R_map_lidar_;
      t_map_lidar_ = reg_result.R * t_map_lidar_ + reg_result.t;
      auto patches_in_map =
          TransformPatchesToMap(cur_patches, R_map_lidar_, t_map_lidar_);

      ground_map_frames_.push_back(patches_in_map);

      while (ground_map_frames_.size() > node_params_.max_ground_map_frames_) {
        ground_map_frames_.pop_front();
      }

      RebuildGroundMap();
      if (node_params_.debug) {
        geometry_msgs::msg::Point translation_msg;
        translation_msg.x = reg_result.t.x();
        translation_msg.y = reg_result.t.y();
        translation_msg.z = reg_result.t.z();
        translation_pub_->publish(translation_msg);

        geometry_msgs::msg::Point eulers_msg;
        auto [roll, pitch, yaw] = EulersFromMatrixSimple(reg_result.R);
        eulers_msg.x = roll;
        eulers_msg.y = pitch;
        eulers_msg.z = yaw;
        eulers_pub_->publish(eulers_msg);

        geometry_msgs::msg::Point eulers_imu_msg;
        auto [imu_r, imu_p, imu_y] = EulersFromMatrixSimple(R_lidar_delta);
        eulers_imu_msg.x = imu_r;
        eulers_imu_msg.y = imu_p;
        eulers_imu_msg.z = imu_y;
        imu_eulers_pub_->publish(eulers_imu_msg);
        // std::cout << pitch << ":" << imu_p << std::endl;
      }
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

void GALONode::RebuildGroundMap() {
  ground_map_.clear();

  for (const auto& frame : ground_map_frames_) {
    ground_map_.insert(ground_map_.end(), frame.begin(), frame.end());
  }
}

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  GALONodeParams prms;
  auto node = std::make_shared<GALONode>(prms);
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}