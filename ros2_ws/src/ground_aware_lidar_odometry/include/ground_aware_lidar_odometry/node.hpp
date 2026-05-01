#pragma once
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>

#include <deque>
#include <memory>
#include <optional>
#include <thread>
#include <tuple>

#include "geometry_msgs/msg/point.hpp"
#include "ground_aware_lidar_odometry/deskew.hpp"
#include "ground_aware_lidar_odometry/segmentation.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

struct GALONodeParams {
  int debug = 1;
  int max_ground_map_frames_ = 10;
  int max_planar_map_frames_ = 10;
};

struct ImuOrientationStamped {
  double time = 0.0;
  Eigen::Quaterniond q = Eigen::Quaterniond::Identity();
};
struct GroundPatchFrame {
  std::vector<GroundPatch> patches;
  Eigen::Matrix3d R_map_lidar;
  Eigen::Vector3d t_map_lidar;
};

class GALONode : public rclcpp::Node {
 public:
  GALONode(const GALONodeParams& node_params);

 private:
  std::string imu_frame = "imu";
  std::string lidar_frame = "rslidar";
  FiniteDeque<ImuOrientationStamped> imu_orientation_queue_;
  bool has_imu_lidar_extrinsic_ = false;
  bool has_imu_prev_ = false;
  bool has_lidar_imu_prev_ = false;
  GALONodeParams node_params_;
  DeskewParams deskew_prms_;
  GroundSegmentationParams segementation_params_;
  GroundPatchParams ground_patch_params_;
  GroundRegistrationParams ground_registration_params_;
  PlanarRegistrationParams planar_registration_params_;
  std::mutex mut_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr lidar_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr deskew_cld_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr colored_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr
      ground_patches_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr translation_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr eulers_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr imu_eulers_pub_;
  Eigen::Matrix3d R_map_lidar_ = Eigen::Matrix3d::Identity();
  Eigen::Vector3d t_map_lidar_ = Eigen::Vector3d::Zero();
  Eigen::Quaterniond imu_q_prev_, q_il, imu_q_lidar_prev_;
  Eigen::Matrix3d R_imu_delta;
  Eigen::Vector3d t_il;
  DeskewAlgorithm deskew_algo_;
  Segmentation segmentation_;
  GroundPatchExtractor ground_patches_extractor_;
  GroundRegistration ground_registration_;
  PlanarRegistration planar_registration_;
  std::vector<TimeMeasurments_t> time_measurments;
  std::deque<std::vector<GroundPatch>> ground_map_frames_;
  std::vector<GroundPatch> ground_map_;
  std::deque<std::vector<Eigen::Vector2d>> objects_map_frames_;
  std::vector<Eigen::Vector2d> objects_map_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  void LidarCb(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
  void ImuCb(const sensor_msgs::msg::Imu::SharedPtr msg);
  void PrintTimeMeasurments(const std::vector<TimeMeasurments_t>& measurments);
  std::tuple<double, double, double> EulersFromMatrixSimple(
      const Eigen::Matrix3d& R);
  bool GetExtrinsicTf(tf2_ros::Buffer& tf_buffer, const std::string& imu_frame,
                      const std::string& lidar_frame);
  std::optional<Eigen::Quaterniond> GetImuOrientationAt(
      double query_time) const;
  std::vector<GroundPatch> TransformPatchesToMap(
      const std::vector<GroundPatch>& patches, const Eigen::Matrix3d& R,
      const Eigen::Vector3d& t);
  std::vector<Eigen::Vector2d> TransformPointsToMap(
      const std::vector<Eigen::Vector2d>& points, const Eigen::Matrix3d& R,
      const Eigen::Vector3d& t);
  void RebuildGroundMap();
  void RebuildObjectsMap();
  Eigen::Vector3d MergeGroundAndPlanarTranslation(
      const Eigen::Vector3d& t_ground, const Eigen::Vector2d& t_planar);

  Eigen::Matrix3d MergeGroundAndPlanarRotation(const Eigen::Matrix3d& R_ground,
                                               const Eigen::Matrix2d& R_planar);
};