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
};

class PositionPredictor {
 public:
  PositionPredictor(const PredictionParams& params) : params_(params) {}
  PredictedPose PredictFromWheelModel(const WheelSpeedAngleData& speed_data,
                                      const Eigen::Matrix3d& R_current,
                                      const Eigen::Vector3d& t_current,
                                      double prev_time, double curr_time);

  RearWheelSpeedResult EstimateRearAxleSpeed(
      const WheelSpeedAngleData& speed_data);

 private:
  double prev_speed_ = 0;
  bool has_prev_speed_ = false;
  PredictionParams params_;
};
