#include "ground_aware_lidar_odometry/node.hpp"

#include <functional>

GALONode::GALONode() : Node("GALONode") {
  deskew_algo_ = std::make_shared<DeskewAlgorithm>(
      deskew_prms_, this->get_logger(), *this->get_clock());
  lidar_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "/Sensor/lidar_front/rslidar_points", 10,
      std::bind(&GALONode::LidarCb, this, std::placeholders::_1));

  imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
      "/Sensor/imu_front/data", 1,
      std::bind(&GALONode::ImuCb, this, std::placeholders::_1));

  deskew_cld_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
      "/GALO/deskewed_cloud", 1);
}

void GALONode::LidarCb(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
  std::optional<sensor_msgs::msg::PointCloud2> deskewed;
  {
    std::lock_guard<std::mutex> lock(mut_);
    deskew_algo_->UpdateLidarQueue(msg);
    deskewed = deskew_algo_->ProcessCloudsQueue();
  }
  if (deskewed) {
    deskew_cld_pub_->publish(*deskewed);
  }
}

void GALONode::ImuCb(const sensor_msgs::msg::Imu::SharedPtr msg) {
  rclcpp::Time msg_time(msg->header.stamp);
  {
    std::lock_guard<std::mutex> lock(mut_);
    deskew_algo_->UpdateImuQueue(msg->angular_velocity.z, msg_time.seconds());
  }
}

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<GALONode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}