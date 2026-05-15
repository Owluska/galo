#include "ground_aware_lidar_odometry/node.hpp"

#include "rclcpp/executors/multi_threaded_executor.hpp"

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<GaloOdometryComponent>(rclcpp::NodeOptions());
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
