#include "ground_aware_lidar_odometry/prediction.hpp"

namespace {
Eigen::Matrix3d RotationFromYaw(double yaw) {
  const double c = std::cos(yaw);
  const double s = std::sin(yaw);
  Eigen::Matrix3d R = Eigen::Matrix3d::Identity();
  R(0, 0) = c;
  R(0, 1) = -s;
  R(1, 0) = s;
  R(1, 1) = c;
  return R;
}
}  // namespace

PredictedPose PositionPredictor::PredictFromWheelModel(
    const WheelSpeedAngleData& speed_data, const Eigen::Matrix3d& R_current,
    const Eigen::Vector3d& t_current, double prev_time, double curr_time) {
  return PredictFromWheelModelImpl(speed_data, R_current, t_current, prev_time,
                                   curr_time, true);
}

PredictedPose PositionPredictor::PredictFromWheelQueue(
    const FiniteDeque<WheelSpeedAngleData>& wheel_queue,
    const Eigen::Matrix3d& R_current, const Eigen::Vector3d& t_current,
    double prev_time, double curr_time) {
  PredictedPose out;
  out.t = t_current;
  out.yaw = std::atan2(R_current(1, 0), R_current(0, 0));
  out.speed = has_prev_speed_ ? prev_speed_ : 0.0;

  if (wheel_queue.Size() == 0) {
    return out;
  }

  auto active = FindClosestByTime(
      wheel_queue, prev_time,
      [](const WheelSpeedAngleData& item) { return item.wheel_time; });
  if (!active) {
    return out;
  }

  double segment_start = prev_time;
  for (const auto& sample : wheel_queue) {
    if (!sample.has_wheel_data || sample.wheel_time <= prev_time) {
      continue;
    }
    if (sample.wheel_time >= curr_time) {
      break;
    }

    out = PredictFromWheelModelImpl(*active, RotationFromYaw(out.yaw), out.t,
                                    segment_start, sample.wheel_time, true);
    segment_start = sample.wheel_time;
    active = sample;
  }

  out = PredictFromWheelModelImpl(*active, RotationFromYaw(out.yaw), out.t,
                                  segment_start, curr_time, true);
  return out;
}

PredictedPose PositionPredictor::PredictFromWheelModelImpl(
    const WheelSpeedAngleData& speed_data, const Eigen::Matrix3d& R_current,
    const Eigen::Vector3d& t_current, double prev_time, double curr_time,
    bool enforce_max_dt) {
  PredictedPose out;
  out.t = t_current;
  out.yaw = std::atan2(R_current(1, 0), R_current(0, 0));
  out.speed = has_prev_speed_
                  ? prev_speed_
                  : 0.5 * (speed_data.left_speed + speed_data.right_speed);

  if (!speed_data.has_wheel_data) {
    return out;
  }

  const double dt = curr_time - prev_time;
  if (dt <= params_.min_prediction_dt) {
    // that's okay - we don't need to update because last prediction is basicaly
    // the same
    return out;
  }
  if (enforce_max_dt && dt > params_.max_prediction_dt) {
    RCLCPP_WARN(logger_,
                "pose prediction failed because of wrong dt value %.4f", dt);
    return out;
  }

  if (std::abs(prev_time - speed_data.wheel_time) >
      params_.max_wheel_data_age) {
    RCLCPP_WARN(logger_,
                "pose prediction failed because wheel data is stalled %.1f",
                std::abs(prev_time - speed_data.wheel_time));
    return out;
  }

  RearWheelSpeedResult res = EstimateRearAxleSpeed(speed_data);
  if (!res.valid || std::abs(res.speed) < params_.min_valid_speed) {
    RCLCPP_WARN(logger_,
                "pose prediction failed because speed estimation is invalid");
    return out;
  }

  double speed = res.speed;
  if (has_prev_speed_) {
    const double alpha = std::clamp(params_.tau, 0.0, 1.0);
    speed = prev_speed_ + alpha * (speed - prev_speed_);
  }

  // Make sure wheel_angle is radians.
  const double steer = speed_data.wheel_angle;

  const double yaw_rate = speed / params_.wheelbase_ * std::tan(steer);

  const double dyaw = yaw_rate * dt;
  const double yaw_mid = out.yaw + 0.5 * dyaw;
  out.speed = speed;
  out.t.x() += speed * dt * std::cos(yaw_mid);
  out.t.y() += speed * dt * std::sin(yaw_mid);
  out.yaw = NormalizeAngle(out.yaw + dyaw);

  prev_speed_ = speed;
  has_prev_speed_ = true;
  return out;
}

RearWheelSpeedResult PositionPredictor::EstimateRearAxleSpeed(
    const WheelSpeedAngleData& speed_data) {
  RearWheelSpeedResult out;
  out.speed = prev_speed_;
  out.valid = false;
  out.used_left = false;
  out.used_right = false;
  out.residual = 0.0;
  double v_rr = speed_data.right_speed;
  double v_rl = speed_data.left_speed;
  if (!std::isfinite(v_rl) || !std::isfinite(v_rr) ||
      !std::isfinite(speed_data.wheel_angle)) {
    return out;
  }

  const double k = params_.rear_track_ / (2.0 * params_.wheelbase_) *
                   std::tan(speed_data.wheel_angle);

  // Avoid singular/crazy correction at extreme steering or bad calibration.
  if (std::abs(k) > params_.max_steering_correction) {
    return out;
  }

  const double v_avg = 0.5 * (v_rl + v_rr);

  const double measured_diff = v_rr - v_rl;
  const double expected_diff = 2.0 * k * v_avg;
  const double residual = measured_diff - expected_diff;

  out.residual = residual;

  const double abs_residual = std::abs(residual);

  // Case 1: both wheels are mutually consistent.
  if (std::abs(v_avg) < params_.min_speed_for_turn_check ||
      abs_residual < params_.max_diff_residual) {
    out.speed = v_avg;
    out.valid = true;
    out.used_left = true;
    out.used_right = true;
    return out;
  }

  // Case 2: try "right wheel good, left wheel bad".
  const double v_from_right = v_rr / (1.0 + k);

  // Case 3: try "left wheel good, right wheel bad".
  const double v_from_left = v_rl / (1.0 - k);

  const double jump_right = std::abs(v_from_right - prev_speed_);
  const double jump_left = std::abs(v_from_left - prev_speed_);

  if (jump_right < jump_left && jump_right < params_.max_jump) {
    out.speed = v_from_right;
    out.valid = true;
    out.used_right = true;
    out.used_left = false;
    return out;
  }

  if (jump_left < jump_right && jump_left < params_.max_jump) {
    out.speed = v_from_left;
    out.valid = true;
    out.used_left = true;
    out.used_right = false;
    return out;
  }

  return out;
}
