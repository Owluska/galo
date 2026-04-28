#include "ground_aware_lidar_odometry/node.hpp"

#include <functional>

GALONode::GALONode(const GALONodeParams& node_params)
    : Node("GALONode"),
      node_params_(node_params),
      segmentation_(segementation_params_),
      ground_patches_extractor_(ground_patch_params_),
      ground_registration_(ground_registration_params_),
      deskew_algo_(deskew_prms_, this->get_logger(), *this->get_clock()) {
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
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                         "patches=%zu", cur_patches.size());
    auto patches_marker =
        ground_patches_extractor_.MakeGroundPatchMarkers(deskewed->header);
    ground_patches_pub_->publish(patches_marker);
  }
  if (!ground_map_.empty()) {
    TimeMeasurments_t registration_meas("ground_registration");
    auto reg_result = ground_registration_.Align(cur_patches, ground_map_);
    registration_meas.SetEnd();
    time_measurments.push_back(registration_meas);
    if (reg_result.valid) {
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
      {
        std::lock_guard<std::mutex> lock(mut_);
        geometry_msgs::msg::Point eulers_imu_msg;
        auto [imu_r, imu_p, imu_y] = EulersFromMatrixSimple(R_imu_delta);
        eulers_imu_msg.x = imu_r;
        eulers_imu_msg.y = imu_p;
        eulers_imu_msg.z = imu_y;
        imu_eulers_pub_->publish(eulers_imu_msg);
      }
    }
  }
  ground_map_ = cur_patches;
  PrintTimeMeasurments(time_measurments);
}

void GALONode::ImuCb(const sensor_msgs::msg::Imu::SharedPtr msg) {
  auto imu_q_curr_ = Eigen::Quaterniond(msg->orientation.w, msg->orientation.x,
                                        msg->orientation.y, msg->orientation.z);

  rclcpp::Time msg_time(msg->header.stamp);
  {
    std::lock_guard<std::mutex> lock(mut_);
    R_imu_delta = imu_q_prev_.toRotationMatrix().transpose() *
                  imu_q_curr_.toRotationMatrix();
    deskew_algo_.UpdateImuQueue(msg->angular_velocity.z, msg_time.seconds());
  }
  imu_q_prev_ = imu_q_curr_;
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

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  GALONodeParams prms;
  auto node = std::make_shared<GALONode>(prms);
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}