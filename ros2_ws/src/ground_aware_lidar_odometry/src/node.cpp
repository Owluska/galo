#include "ground_aware_lidar_odometry/node.hpp"

#include <functional>

GALONode::GALONode() : Node("GALONode") {
  lidar_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "/Sensor/lidar_front/rslidar_points", 10,
      std::bind(&GALONode::LidarCb, this, std::placeholders::_1));

  imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
      "/Sensor/imu_front/data", 1,
      std::bind(&GALONode::ImuCb, this, std::placeholders::_1));

  pure_state_sub_ = this->create_subscription<user_msgs::msg::PureState>(
      "/SC/pure_state", 1,
      std::bind(&GALONode::PureStateCb, this, std::placeholders::_1));
}

void GALONode::LidarCb(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
  auto deskewd = deskew_algo_->ProcessCloud(msg);
}

void GALONode::ImuCb(const sensor_msgs::msg::Imu::SharedPtr msg) {
  if (!deskew_algo_) {
    deskew_algo_ =
        std::make_shared<DeskewAlgorithm>(deskew_prms_, this->get_logger());
  }
  rclcpp::Time msg_time(msg->header.stamp);
  deskew_algo_->UpdateQueue(last_speed_, msg->angular_velocity.z,
                            msg_time.seconds());
}

void GALONode::PureStateCb(const user_msgs::msg::PureState::SharedPtr msg) {
  last_speed_ = std::hypot(msg->velocity.linear.x, msg->velocity.linear.y,
                           msg->velocity.linear.z);
}

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<GALONode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}