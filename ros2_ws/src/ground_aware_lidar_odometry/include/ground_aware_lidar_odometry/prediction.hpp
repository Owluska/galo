#pragma once

#include "ground_aware_lidar_odometry/utils.hpp"

struct PredictedPose {
  Eigen::Vector3d t;
  double yaw;
  double speed;
};

struct WheelSpeedAngleData {
  double right_speed = 0;
  double left_speed = 0;
  double wheel_angle = 0;
  double wheel_time = -1;
  bool has_wheel_data = false;
};

struct RearWheelSpeedResult {
  double speed;
  bool valid;
  bool used_left;
  bool used_right;
  double residual;
};

struct PredictionParams {
  double rear_track_ = 5.16;
  double wheelbase_ = 5.3;
  double max_diff_residual = 0.5;  // m/s
  double max_jump = 2.0;           // m/s
  double min_speed_for_turn_check = 0.2;
  double tau = 1.0;
  double min_prediction_dt = 1e-3;
  double max_prediction_dt = 0.5;
  double max_wheel_data_age = 0.3;
  double min_valid_speed = 0.05;
  double max_steering_correction = 0.5;
  bool use_pitch_for_z = false;
  double pitch_z_gain = 0.2;
  double max_vertical_speed = 0.5;
};

class PositionPredictor {
 public:
  PositionPredictor(const PredictionParams& params,
                    const rclcpp::Logger& logger, const rclcpp::Clock& clock)
      : logger_(logger), clock_(clock), params_(params) {}
  PredictedPose PredictFromWheelModel(const WheelSpeedAngleData& speed_data,
                                      const Eigen::Matrix3d& R_current,
                                      const Eigen::Vector3d& t_current,
                                      double prev_time, double curr_time);

  PredictedPose PredictFromWheelQueue(
      const FiniteDeque<WheelSpeedAngleData>& wheel_queue,
      const Eigen::Matrix3d& R_current, const Eigen::Vector3d& t_current,
      double prev_time, double curr_time);

  RearWheelSpeedResult EstimateRearAxleSpeed(
      const WheelSpeedAngleData& speed_data);

 private:
  PredictedPose PredictFromWheelModelImpl(const WheelSpeedAngleData& speed_data,
                                          const Eigen::Matrix3d& R_current,
                                          const Eigen::Vector3d& t_current,
                                          double prev_time, double curr_time,
                                          bool enforce_max_dt, double forward_z);

  double prev_speed_ = 0;
  bool has_prev_speed_ = false;
  rclcpp::Logger logger_;
  rclcpp::Clock clock_;
  PredictionParams params_;
};
