#pragma once
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>

#include <deque>
#include <functional>
#include <memory>
#include <optional>
#include <robot_localization/navsat_conversions.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <thread>
#include <tuple>

#include "common_msgs/msg/pure_state.hpp"
#include "common_msgs/msg/wheel_speed.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "ground_aware_lidar_odometry/deskew.hpp"
#include "ground_aware_lidar_odometry/prediction.hpp"
#include "ground_aware_lidar_odometry/segmentation.hpp"
#include "ground_aware_lidar_odometry/simple_gnss_converter.hpp"
#include "qarl_msgs/msg/nmea_gga.hpp"
#include "qarl_msgs/msg/orientation_stamped.hpp"
#include "qarl_msgs/msg/w_angle_feedback.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "std_msgs/msg/float32.hpp"

namespace {

template <typename T>
T DeclareAndGet(rclcpp::Node& node, const std::string& name,
                const T& default_value) {
  return node.declare_parameter<T>(name, default_value);
}

}  // namespace

struct GnssCovariance {
  double x_precision = 0.03;
  double y_precision = 0.03;
  double z_precision = 0.03;
  double roll_precision = 5;   // deg
  double pitch_precision = 5;  // deg
  double yaw_precision = 5;    // deg
};

struct LidarCovariance {
  double base_xy = 0.1;  // 10 cm baseline
  double z = 0.2;        // m
  double scale = 1.0;
  double roll = 0.5;   // rad^2
  double pitch = 0.5;  // rad^2

  std::tuple<double, double> GetXYYawSigmas(
      const PlanarRegistrationResult& res) {
    auto scale = 1;
    // worse if residual high
    scale *= std::clamp(res.mean_residual / 0.1, 1.0, 5.0);
    // worse if few matches
    scale *= std::clamp(100.0 / std::max(res.matches, 1), 1.0, 3.0);

    double sigma_xy = base_xy * scale;
    double sigma_yaw = 2.0 * M_PI / 180.0 * scale;  // 2 deg base
    return std::make_tuple(sigma_xy, sigma_yaw);
  }
};

struct GALONodeParams {
  int debug = 1;
  int max_ground_map_frames = 10;
  int max_planar_map_frames = 10;
  GnssCovariance gt_cov_;
  LidarCovariance est_cov_;
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

struct GnssData {
  Eigen::Vector3d gnss_local_;
  double yaw;
  bool has_gnss_position_ = false;
  bool has_gnss_yaw = false;
  double nmea_time = 0;
};

struct GroundRegistrationGatePrms {
  int min_matches = 35;
  double max_residual = 0.35;
  double max_droll = 5.0;   // deg
  double max_dpitch = 5.0;  // deg
  double max_dz = 1.0;      // m
};

class GALONode : public rclcpp::Node {
 public:
  GALONode();

 private:
  std::string imu_frame = "imu";
  std::string lidar_frame = "rslidar";
  std::string pos_antena_frame = "pos_antenna";
  bool has_imu_lidar_extrinsic_ = false;
  bool has_imu_prev_ = false;
  bool has_lidar_imu_prev_ = false;
  bool has_lidar_odom_initialized_from_gnss_ = false;
  bool has_latest_R_map_base_ = false;
  bool has_prev_lidar_pose_for_prediction_ = false;
  double prev_lidar_pose_time_ = 0.0;
  double last_wa_ = 0;
  std::mutex mut_;

  GALONodeParams node_params_;
  DeskewParams deskew_prms_;
  GroundSegmentationParams segementation_params_;
  GroundPatchParams ground_patch_params_;
  GroundRegistrationParams ground_registration_params_;
  PlanarRegistrationParams planar_registration_params_;
  GroundRegistrationGatePrms ground_reg_gate_params_;
  GnssLocalizationParams gnss_loc_params_;
  PredictionParams prediction_params_;

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr lidar_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<qarl_msgs::msg::NmeaGGA>::SharedPtr gnss_sub_;
  rclcpp::Subscription<qarl_msgs::msg::WAngleFeedback>::SharedPtr wa_sub_;
  rclcpp::Subscription<common_msgs::msg::WheelSpeed>::SharedPtr
      wheel_speed_sub_;
  rclcpp::Subscription<qarl_msgs::msg::OrientationStamped>::SharedPtr
      gnss_orientation_sub_;

  rclcpp::Subscription<common_msgs::msg::PureState>::SharedPtr pure_state_sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr deskew_cld_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr colored_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr
      ground_patches_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr translation_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr gt_eulers_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr
      pure_state_eulers_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr est_eulers_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr
      gnss_imu_pose_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr
      lidar_pose_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr speed_pub_;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  Eigen::Vector3d t_map_lidar_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d t_il, t_pos_lidar;
  Eigen::Vector3d prev_t_map_lidar_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d velocity_map_lidar_ = Eigen::Vector3d::Zero();

  Eigen::Quaterniond imu_q_prev_, q_il, q_i_map, imu_q_lidar_prev_, q_pos_lidar;
  Eigen::Matrix3d R_imu_delta;
  Eigen::Matrix3d latest_R_map_base_ = Eigen::Matrix3d::Identity();
  Eigen::Matrix3d R_map_lidar_ = Eigen::Matrix3d::Identity();

  FiniteDeque<ImuOrientationStamped> imu_orientation_queue_;
  std::vector<TimeMeasurments_t> time_measurments;
  std::vector<GroundPatch> ground_map_;
  std::vector<Eigen::Vector2d> objects_map_;
  std::deque<std::vector<GroundPatch>> ground_map_frames_;
  std::deque<std::vector<Eigen::Vector2d>> objects_map_frames_;

  GnssData gnss_data_;
  WheelSpeedAngleData wheel_data;
  GnssLocalConverter gnss_converter_;
  DeskewAlgorithm deskew_algo_;
  Segmentation segmentation_;
  GroundPatchExtractor ground_patches_extractor_;
  GroundRegistration ground_registration_;
  PlanarRegistration planar_registration_;
  PositionPredictor position_predictor_;

  void LidarCb(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
  void ImuCb(const sensor_msgs::msg::Imu::SharedPtr msg);
  void GnssCb(const qarl_msgs::msg::NmeaGGA::SharedPtr msg);
  void GnssYawCb(const qarl_msgs::msg::OrientationStamped::SharedPtr msg);
  void PureStateCb(const common_msgs::msg::PureState::SharedPtr msg);
  void WheelSpeedCb(const common_msgs::msg::WheelSpeed::SharedPtr msg);
  void WheelAngleCb(const qarl_msgs::msg::WAngleFeedback::SharedPtr msg);

  void PrintTimeMeasurments(const std::vector<TimeMeasurments_t>& measurments);

  void ProcessCloud(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
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
  geometry_msgs::msg::PoseWithCovarianceStamped BuildPoseWithCovarianceMsg(
      const builtin_interfaces::msg::Time& time, const std::string& frame,
      const Eigen::Matrix3d& R, const Eigen::Vector3d& t,
      const Eigen::Vector<double, 6>& sigmas);
  Eigen::Vector3d MergeGroundAndPlanarTranslation(
      const Eigen::Vector3d& t_ground, const Eigen::Vector3d& t_prior,
      const Eigen::Vector2d& t_planar, double alpha_xy = 0.3,
      double alpha_z = 0.3);

  Eigen::Matrix3d MergeGroundAndPlanarRotation(const Eigen::Matrix3d& R_ground,
                                               const Eigen::Matrix3d& R_prior,
                                               const Eigen::Matrix2d& R_planar,
                                               double alpha_rp = 0.2);
  bool TryInitializeOdomFromGnss();
  bool CheckGroundRegistration(const GroundRegistrationResult& res);

  static GALONodeParams LoadNodeParams(rclcpp::Node& node);
  static DeskewParams LoadDeskewParams(rclcpp::Node& node);
  static GroundSegmentationParams LoadGroundSegmentationParams(
      rclcpp::Node& node);
  static GroundPatchParams LoadGroundPatchParams(rclcpp::Node& node);
  static GroundRegistrationParams LoadGroundRegistrationParams(
      rclcpp::Node& node);
  static PlanarRegistrationParams LoadPlanarRegistrationParams(
      rclcpp::Node& node);
  static GnssLocalizationParams LoadGnssParams(rclcpp::Node& node);
};