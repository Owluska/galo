#pragma once

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <memory>
#include <mutex>
#include <string>

#include "common_msgs/msg/wheel_speed.hpp"
#include "ground_aware_lidar_odometry/deskew.hpp"
#include "ground_aware_lidar_odometry/prediction.hpp"
#include "qarl_msgs/msg/w_angle_feedback.hpp"
#include "rclcpp/callback_group.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

namespace ground_aware_lidar_odometry {

class GaloDeskewComponent : public rclcpp::Node {
 public:
  explicit GaloDeskewComponent(const rclcpp::NodeOptions& options);

 private:
  struct Params {
    std::string lidar_frame = "rslidar";
    std::string body_frame = "base_link";
    std::string lidar_topic = "/Sensor/lidar_front/rslidar_points";
    std::string imu_topic = "/Sensor/imu_front/data";
    std::string wheel_speed_topic = "/FB/wheel_speed_feedback";
    std::string wheel_angle_topic = "/FB/wangle_feedback";
    std::string deskewed_cloud_topic = "/GALO/deskewed_cloud";
  };

  Params params_;
  DeskewParams deskew_params_;
  PredictionParams prediction_params_;

  std::mutex mutex_;
  bool has_lidar_body_tf_ = false;
  double last_wheel_angle_ = 0.0;
  WheelSpeedAngleData wheel_data_;
  Eigen::Quaterniond q_lidar_body_ = Eigen::Quaterniond::Identity();
  Eigen::Vector3d t_lidar_body_ = Eigen::Vector3d::Zero();

  rclcpp::CallbackGroup::SharedPtr lidar_callback_group_;
  rclcpp::CallbackGroup::SharedPtr sensor_callback_group_;

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr lidar_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<common_msgs::msg::WheelSpeed>::SharedPtr
      wheel_speed_sub_;
  rclcpp::Subscription<qarl_msgs::msg::WAngleFeedback>::SharedPtr
      wheel_angle_sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr deskew_pub_;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  PositionPredictor position_predictor_;
  DeskewAlgorithm deskew_algorithm_;

  void LidarCb(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
  void ImuCb(const sensor_msgs::msg::Imu::SharedPtr msg);
  void WheelSpeedCb(const common_msgs::msg::WheelSpeed::SharedPtr msg);
  void WheelAngleCb(const qarl_msgs::msg::WAngleFeedback::SharedPtr msg);

  bool EnsureLidarBodyTf();

  static Params LoadParams(rclcpp::Node& node);
  static DeskewParams LoadDeskewParams(rclcpp::Node& node);
  static PredictionParams LoadPredictionParams(rclcpp::Node& node);
};

}  // namespace ground_aware_lidar_odometry
