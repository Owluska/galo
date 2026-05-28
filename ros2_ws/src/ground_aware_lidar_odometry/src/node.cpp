#include "ground_aware_lidar_odometry/node.hpp"

#include <algorithm>
#include <exception>
#include <iomanip>

#include "ground_aware_lidar_odometry/feature_conversions.hpp"
#include "rclcpp_components/register_node_macro.hpp"

namespace {
bool ShouldLogSteady(std::chrono::steady_clock::time_point& last_log_time,
                     int throttle_ms) {
  const auto now = std::chrono::steady_clock::now();
  const auto throttle = std::chrono::milliseconds(throttle_ms);
  if (last_log_time == std::chrono::steady_clock::time_point{} ||
      now - last_log_time >= throttle) {
    last_log_time = now;
    return true;
  }
  return false;
}

Pose6D MakePose6D(const Eigen::Matrix3d& R, const Eigen::Vector3d& t) {
  auto [roll, pitch, yaw] = EulersFromMatrixSimple(R);
  Pose6D pose;
  pose.x = t.x();
  pose.y = t.y();
  pose.z = t.z();
  pose.roll = roll;
  pose.pitch = pitch;
  pose.yaw = yaw;
  return pose;
}

std::array<double, 6> PoseToArray(const Pose6D& pose) {
  return {pose.x, pose.y, pose.z, pose.roll, pose.pitch, pose.yaw};
}

Pose6D InterpolatePose(const TimedPose6D& a, const TimedPose6D& b,
                       double query_time) {
  const double dt = b.time - a.time;
  const double alpha = dt > 1e-9 ? (query_time - a.time) / dt : 0.0;

  Pose6D pose;
  pose.x = a.pose.x + alpha * (b.pose.x - a.pose.x);
  pose.y = a.pose.y + alpha * (b.pose.y - a.pose.y);
  pose.z = a.pose.z + alpha * (b.pose.z - a.pose.z);
  pose.roll = a.pose.roll + alpha * NormalizeAngle(b.pose.roll - a.pose.roll);
  pose.pitch =
      a.pose.pitch + alpha * NormalizeAngle(b.pose.pitch - a.pose.pitch);
  pose.yaw = a.pose.yaw + alpha * NormalizeAngle(b.pose.yaw - a.pose.yaw);
  pose.roll = NormalizeAngle(pose.roll);
  pose.pitch = NormalizeAngle(pose.pitch);
  pose.yaw = NormalizeAngle(pose.yaw);
  return pose;
}

std::optional<Pose6D> InterpolatePoseHistory(
    const std::deque<TimedPose6D>& history, double query_time) {
  if (history.empty()) return {};
  if (query_time < history.front().time || query_time > history.back().time) {
    return {};
  }

  auto upper = std::lower_bound(
      history.begin(), history.end(), query_time,
      [](const TimedPose6D& pose, double time) { return pose.time < time; });

  if (upper == history.begin()) {
    return upper->pose;
  }
  if (upper == history.end()) {
    return history.back().pose;
  }
  if (std::abs(upper->time - query_time) < 1e-9) {
    return upper->pose;
  }

  const auto prev = std::prev(upper);
  return InterpolatePose(*prev, *upper, query_time);
}
}  // namespace

GaloOdometryComponent::GaloOdometryComponent(const rclcpp::NodeOptions& options)
    : Node("galo", options),
      node_params_(LoadNodeParams(*this)),
      ground_registration_params_(LoadGroundRegistrationParams(*this)),
      planar_registration_params_(LoadPlanarRegistrationParams(*this)),
      ground_reg_gate_params_(LoadGroundGateParams(*this)),
      planar_reg_gate_params_(LoadPlanarGateParams(*this)),
      gnss_loc_params_(LoadGnssParams(*this)),
      prediction_params_(LoadPredictionParams(*this)),
      gnss_converter_(gnss_loc_params_),
      ground_registration_(ground_registration_params_, this->get_logger(),
                           *this->get_clock()),
      planar_registration_(planar_registration_params_, this->get_logger(),
                           *this->get_clock()),
      position_predictor_(prediction_params_, this->get_logger(),
                          *this->get_clock()) {
  imu_frame = node_params_.imu_frame;
  lidar_frame = node_params_.lidar_frame;
  pos_antena_frame = node_params_.pos_antenna_frame;
  orientation_antenna_frame = node_params_.orientation_antenna_frame;
  map_frame = node_params_.map_frame;
  body_frame = node_params_.body_frame;
  imu_orientation_queue_.Resize(node_params_.imu_orientation_queue_size);
  wheel_data_queue_.Resize(node_params_.wheel_data_queue_size);
  prediction_queue_.Resize(node_params_.prediction_queue_size);

  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  lidar_callback_group_ =
      this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  other_callback_group_ =
      this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  rclcpp::SubscriptionOptions lidar_sub_options;
  lidar_sub_options.callback_group = lidar_callback_group_;

  rclcpp::SubscriptionOptions other_sub_options;
  other_sub_options.callback_group = other_callback_group_;

  RCLCPP_INFO(this->get_logger(), "Loaded GALO parameters from ROS params");
  features_sub_ = this->create_subscription<
      ground_aware_lidar_odometry::msg::FrameFeatures>(
      node_params_.frame_features_topic, 10,
      std::bind(&GaloOdometryComponent::FrameFeaturesCb, this,
                std::placeholders::_1),
      lidar_sub_options);

  imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
      node_params_.imu_topic, 500,
      std::bind(&GaloOdometryComponent::ImuCb, this, std::placeholders::_1),
      other_sub_options);
  gnss_sub_ = this->create_subscription<qarl_msgs::msg::NmeaGGA>(
      node_params_.gnss_topic, 100,
      std::bind(&GaloOdometryComponent::GnssCb, this, std::placeholders::_1),
      other_sub_options);
  gnss_orientation_sub_ =
      this->create_subscription<qarl_msgs::msg::OrientationStamped>(
          node_params_.gnss_orientation_topic, 100,
          std::bind(&GaloOdometryComponent::GnssYawCb, this,
                    std::placeholders::_1),
          other_sub_options);
  pure_state_sub_ = this->create_subscription<common_msgs::msg::PureState>(
      node_params_.pure_state_topic, 100,
      std::bind(&GaloOdometryComponent::PureStateCb, this,
                std::placeholders::_1),
      other_sub_options);
  wheel_speed_sub_ = this->create_subscription<common_msgs::msg::WheelSpeed>(
      node_params_.wheel_speed_topic, 100,
      std::bind(&GaloOdometryComponent::WheelSpeedCb, this,
                std::placeholders::_1),
      other_sub_options);
  wa_sub_ = this->create_subscription<qarl_msgs::msg::WAngleFeedback>(
      node_params_.wheel_angle_topic, 100,
      std::bind(&GaloOdometryComponent::WheelAngleCb, this,
                std::placeholders::_1),
      other_sub_options);
  gt_eulers_pub_ = this->create_publisher<geometry_msgs::msg::Point>(
      node_params_.gt_eulers_topic, 1);
  est_eulers_pub_ = this->create_publisher<geometry_msgs::msg::Point>(
      node_params_.est_eulers_topic, 1);
  pure_state_eulers_pub_ = this->create_publisher<geometry_msgs::msg::Point>(
      node_params_.pure_state_eulers_topic, 1);
  gnss_imu_pose_pub_ =
      this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
          node_params_.true_pose_topic, 1);
  lidar_pose_pub_ =
      this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
          node_params_.estimate_pose_topic, 1);
  speed_pub_ = this->create_publisher<std_msgs::msg::Float32>(
      node_params_.speed_topic, 1);
  planar_lines_pub_ =
      this->create_publisher<visualization_msgs::msg::MarkerArray>(
          node_params_.planar_lines_topic, 1);

  if (node_params_.gnss_correction_period_sec > 0.0) {
    auto correction_period =
        std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::duration<double>(
                node_params_.gnss_correction_period_sec));
    gnss_correction_timer_ = this->create_wall_timer(
        correction_period,
        std::bind(&GaloOdometryComponent::GnssCorrectionTimerCb, this),
        lidar_callback_group_);
    RCLCPP_INFO(this->get_logger(),
                "GNSS LiDAR odometry correction timer period: %.3f s",
                node_params_.gnss_correction_period_sec);
  }
}

void GaloOdometryComponent::WheelAngleCb(
    const qarl_msgs::msg::WAngleFeedback::SharedPtr msg) {
  bool should_publish_speed = false;
  float speed_output = 0.0f;

  {
    std::lock_guard<std::mutex> lock(mut_);
    last_wa_ = msg->wangle;
    if (wheel_data.has_wheel_data) {
      wheel_data.wheel_angle = last_wa_;
      const RearWheelSpeedResult speed_result =
          position_predictor_.EstimateRearAxleSpeed(wheel_data);
      if (speed_result.valid && std::isfinite(speed_result.speed)) {
        last_speed_output_ = speed_result.speed;
        has_last_speed_output_ = true;
      }
    }

    if (has_last_speed_output_) {
      speed_output = static_cast<float>(last_speed_output_);
      should_publish_speed = true;
    }
  }

  if (should_publish_speed) {
    std_msgs::msg::Float32 speed_msg;
    speed_msg.data = speed_output;
    speed_pub_->publish(speed_msg);
  }
}

void GaloOdometryComponent::WheelSpeedCb(
    const common_msgs::msg::WheelSpeed::SharedPtr msg) {
  std::lock_guard<std::mutex> lock(mut_);
  wheel_data.wheel_angle = last_wa_;
  wheel_data.left_speed = msg->rear_left;
  wheel_data.right_speed = msg->rear_right;
  wheel_data.wheel_time = rclcpp::Time(msg->header.stamp).seconds();
  wheel_data.has_wheel_data = true;
  wheel_data_queue_.UpdateSorted(wheel_data, [](const WheelSpeedAngleData& a,
                                                const WheelSpeedAngleData& b) {
    return a.wheel_time < b.wheel_time;
  });

  if (!has_prev_lidar_pose_for_prediction_ ||
      wheel_data.wheel_time <=
          prev_lidar_pose_time_ + node_params_.pose_dt_min) {
    return;
  }

  Eigen::Matrix3d R_prediction_base = R_map_lidar_;
  Eigen::Vector3d t_prediction_base = t_map_lidar;
  double prediction_prev_time = prev_lidar_pose_time_;

  auto reset_base_from_prediction = [&](const PredictedPose& pose) {
    const double c = std::cos(pose.yaw);
    const double s = std::sin(pose.yaw);
    R_prediction_base = Eigen::Matrix3d::Identity();
    R_prediction_base(0, 0) = c;
    R_prediction_base(0, 1) = -s;
    R_prediction_base(1, 0) = s;
    R_prediction_base(1, 1) = c;
    t_prediction_base = pose.t;
  };

  auto append_prediction = [&](const WheelSpeedAngleData& sample) {
    if (!sample.has_wheel_data ||
        sample.wheel_time <= prediction_prev_time + node_params_.pose_dt_min) {
      return true;
    }

    const double dt = sample.wheel_time - prediction_prev_time;
    if (!std::isfinite(dt) || dt > prediction_params_.max_prediction_dt) {
      return false;
    }

    TimedPredictedPose timed_pred;
    timed_pred.time = sample.wheel_time;
    timed_pred.pose = position_predictor_.PredictFromWheelModel(
        sample, R_prediction_base, t_prediction_base, prediction_prev_time,
        sample.wheel_time);
    prediction_queue_.UpdateSorted(timed_pred, [](const TimedPredictedPose& a,
                                                  const TimedPredictedPose& b) {
      return a.time < b.time;
    });

    reset_base_from_prediction(timed_pred.pose);
    prediction_prev_time = timed_pred.time;
    return true;
  };

  if (prediction_queue_.Size() > 0) {
    const TimedPredictedPose last_pred = prediction_queue_.PeerBack();
    if (wheel_data.wheel_time <= last_pred.time + node_params_.pose_dt_min) {
      return;
    }
    reset_base_from_prediction(last_pred.pose);
    prediction_prev_time = last_pred.time;
    append_prediction(wheel_data);
    return;
  }

  for (const auto& sample : wheel_data_queue_) {
    if (sample.wheel_time <= prev_lidar_pose_time_ + node_params_.pose_dt_min) {
      continue;
    }
    if (sample.wheel_time > wheel_data.wheel_time) {
      break;
    }
    if (!append_prediction(sample)) {
      return;
    }
  }
}

void GaloOdometryComponent::PureStateCb(
    const common_msgs::msg::PureState::SharedPtr msg) {
  Eigen::Quaterniond q =
      Eigen::Quaterniond(msg->pose.orientation.w, msg->pose.orientation.x,
                         msg->pose.orientation.y, msg->pose.orientation.z);
  q.normalize();
  auto [roll, pitch, yaw] = EulersFromMatrixSimple(q.toRotationMatrix());

  geometry_msgs::msg::Point eulers_msg;
  eulers_msg.x = roll;
  eulers_msg.y = pitch;
  eulers_msg.z = yaw;
  pure_state_eulers_pub_->publish(eulers_msg);
}

void GaloOdometryComponent::GnssCorrectionTimerCb() {
  try {
    while (rclcpp::ok() && !TryInitializeOdomFromGnss(true)) {
      if (ShouldLogSteady(last_gnss_correction_warn_time_,
                          node_params_.initialization_log_throttle)) {
        RCLCPP_WARN(this->get_logger(),
                    "Waiting for GNSS position/yaw to correct LiDAR odometry");
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
  } catch (const std::exception& ex) {
    RCLCPP_ERROR(this->get_logger(), "GNSS correction timer failed: %s",
                 ex.what());
  }
}

void GaloOdometryComponent::GnssCb(
    const qarl_msgs::msg::NmeaGGA::SharedPtr msg) {
  // Only use high-quality GNSS fixes.
  // gps_qual >= 4 usually means RTK fixed / high-confidence solution.
  if (msg->gps_qual < node_params_.min_gnss_quality) {
    return;
  }

  Eigen::Vector3d gnss_local =
      gnss_converter_.ToLocal(msg->lon, msg->lat, msg->altitude);

  std::lock_guard<std::mutex> lock(mut_);
  gnss_data_.nmea_time = rclcpp::Time(msg->header.stamp).seconds();
  gnss_data_.gnss_local_ = gnss_local;
  gnss_data_.has_gnss_position_ = true;
}

void GaloOdometryComponent::GnssYawCb(
    const qarl_msgs::msg::OrientationStamped::SharedPtr msg) {
  // We need:
  //
  // 1. Static TFs:
  //      imu <- lidar
  //      imu <- map
  //      pos_antenna <- lidar
  //
  // 2. IMU orientation history, because roll/pitch come from IMU.
  //
  // 3. GNSS origin, because GNSS local coordinates are only meaningful
  //    after the first valid GNSS position has initialized the origin.
  bool has_imu_history = false;
  {
    std::lock_guard<std::mutex> lock(mut_);
    has_imu_history = imu_orientation_queue_.Size() >= 2;
  }
  if (!GetExtrinsicTf(*tf_buffer_, imu_frame, lidar_frame) ||
      !has_imu_history) {
    return;
  }

  const double yaw_time = rclcpp::Time(msg->header.stamp).seconds();

  // Find IMU orientation at the GNSS heading timestamp.
  //
  // This gives us roll/pitch at approximately the same time as the GNSS yaw.
  std::optional<Eigen::Quaterniond> q_closest;
  {
    std::lock_guard<std::mutex> lock(mut_);
    q_closest = GetImuOrientationAt(yaw_time);
  }

  if (!q_closest) {
    return;
  }

  Eigen::Vector3d gnss_local;
  {
    std::lock_guard<std::mutex> lock(mut_);

    if (!gnss_data_.has_gnss_position_) {
      return;
    }
    double pos_time = gnss_data_.nmea_time;

    if (std::abs(yaw_time - pos_time) > node_params_.gnss_yaw_position_max_dt) {
      RCLCPP_WARN(this->get_logger(),
                  "GNSS position/yaw timestamp mismatch: %.3f s",
                  std::abs(yaw_time - pos_time));
      return;
    }

    // Position of pos_antenna in the local GNSS/map frame.
    gnss_local = gnss_data_.gnss_local_;
  }

  // --------------------------------------------------------------------------
  // Convert Trimble heading to ROS ENU yaw
  // --------------------------------------------------------------------------
  //
  // Trimble heading convention:
  //   heading = clockwise from North
  //
  // ROS ENU yaw convention:
  //   yaw = counter-clockwise from East
  double gnss_yaw = gnss_converter_.YawFromGnssHeading(msg->orientation.yaw);
  Eigen::Matrix3d R_map_base =
      q_closest->toRotationMatrix() * q_body_imu.toRotationMatrix().transpose();

  auto [roll_imu, pitch_imu, yaw_imu_unused] =
      EulersFromMatrixSimple(R_map_base);

  (void)yaw_imu_unused;

  // --------------------------------------------------------------------------
  // Build map <- base orientation
  // --------------------------------------------------------------------------
  //
  // Final vehicle/base attitude:
  //
  //   roll  = IMU roll
  //   pitch = IMU pitch
  //   yaw   = GNSS dual-antenna yaw
  //
  // Rotation convention:
  //
  //   R = Rz(yaw) * Ry(pitch) * Rx(roll)
  Eigen::AngleAxisd yaw_rot(gnss_yaw, Eigen::Vector3d::UnitZ());
  Eigen::AngleAxisd roll_rot(roll_imu, Eigen::Vector3d::UnitX());
  Eigen::AngleAxisd pitch_rot(pitch_imu, Eigen::Vector3d::UnitY());

  Eigen::Matrix3d R_gt = yaw_rot.toRotationMatrix() *
                         pitch_rot.toRotationMatrix() *
                         roll_rot.toRotationMatrix();
  Eigen::Vector3d t_body_antenna_mid = 0.5 * (t_body_pos + t_body_orientation);
  Eigen::Vector3d t_gt = gnss_local - R_gt * t_body_antenna_mid;

  // --------------------------------------------------------------------------
  // Publish GT pose
  // --------------------------------------------------------------------------
  Eigen::Vector<double, 6> sigmas = Eigen::Vector<double, 6>(
      node_params_.gt_cov_.x_precision, node_params_.gt_cov_.y_precision,
      node_params_.gt_cov_.z_precision,
      node_params_.gt_cov_.roll_precision * M_PI / 180.0,
      node_params_.gt_cov_.pitch_precision * M_PI / 180.0,
      node_params_.gt_cov_.yaw_precision * M_PI / 180.0);
  geometry_msgs::msg::PoseWithCovarianceStamped pose_msg =
      BuildPoseWithCovarianceMsg(msg->header.stamp, node_params_.gnss_map_frame,
                                 R_gt, t_gt, sigmas);
  gnss_imu_pose_pub_->publish(pose_msg);

  auto [roll_gt, pitch_gt, yaw_gt] = EulersFromMatrixSimple(R_gt);
  geometry_msgs::msg::Point eulers_msg;
  eulers_msg.x = roll_gt;
  eulers_msg.y = pitch_gt;
  eulers_msg.z = yaw_gt;
  gt_eulers_pub_->publish(eulers_msg);

  // --------------------------------------------------------------------------
  // Store GNSS yaw for odometry initialization
  // --------------------------------------------------------------------------
  // TryInitializeOdomFromGnss() should use this yaw as map <- base/pos_antenna,
  // and then compose with pos_antenna <- lidar.
  {
    std::lock_guard<std::mutex> lock(mut_);

    gnss_data_.yaw = gnss_yaw;
    gnss_data_.has_gnss_yaw = true;
    gnss_data_.yaw_time = yaw_time;
    latest_R_base_ = R_gt;
    has_latest_R_base_ = true;
    TimedPose6D gt_sample;
    gt_sample.time = yaw_time;
    gt_sample.pose = MakePose6D(R_gt, t_gt);
    auto insert_pos = std::lower_bound(
        gt_pose_history_.begin(), gt_pose_history_.end(), gt_sample.time,
        [](const TimedPose6D& pose, double time) { return pose.time < time; });
    gt_pose_history_.insert(insert_pos, gt_sample);
    constexpr size_t kMaxGtPoseHistory = 2000;
    while (gt_pose_history_.size() > kMaxGtPoseHistory) {
      gt_pose_history_.pop_front();
    }
  }
}

void GaloOdometryComponent::ImuCb(const sensor_msgs::msg::Imu::SharedPtr msg) {
  Eigen::Quaterniond imu_q_curr(msg->orientation.w, msg->orientation.x,
                                msg->orientation.y, msg->orientation.z);
  imu_q_curr.normalize();

  {
    std::lock_guard<std::mutex> lock(mut_);
    ImuOrientationStamped item;
    item.time = rclcpp::Time(msg->header.stamp).seconds();
    item.q = imu_q_curr;
    imu_orientation_queue_.Update(item);
  }
}

void GaloOdometryComponent::FrameFeaturesCb(
    const ground_aware_lidar_odometry::msg::FrameFeatures::SharedPtr msg) {
  // Main LiDAR pipeline:
  //
  // Main odometry backend pipeline:
  //
  //   1. Receive deskewed cloud
  //   2. Segment ground/non-ground
  //   3. Extract planar/non-ground lines
  //   4. Extract ground patches
  //   5. Initialize from GNSS if needed
  //   6. Run planar line registration for x/y/yaw
  //   7. Run ground registration for z/roll/pitch
  //   8. Merge results
  //   9. Publish odometry pose
  //   10. Add current frame into local map
  ProcessFrameFeatures(msg);

  // Print throttled timing summary.
  PrintTimeMeasurments(time_measurments);
}

void GaloOdometryComponent::ProcessFrameFeatures(
    const ground_aware_lidar_odometry::msg::FrameFeatures::SharedPtr msg) {
  time_measurments.clear();
  EstimatePoseFromFeatures(ground_aware_lidar_odometry::FromMsg(*msg));
}

void GaloOdometryComponent::EstimatePoseFromFeatures(
    const FrameFeatures& features) {
  const double lidar_time = features.lidar_time;
  if (!TryInitializeOdomFromGnss()) {
    if (ShouldLogSteady(last_initialization_warn_time_,
                        node_params_.initialization_log_throttle)) {
      GnssData gnss_snapshot;
      bool has_base_snapshot = false;
      size_t imu_queue_size = 0;
      {
        std::lock_guard<std::mutex> lock(mut_);
        gnss_snapshot = gnss_data_;
        has_base_snapshot = has_latest_R_base_;
        imu_queue_size = imu_orientation_queue_.Size();
      }
      RCLCPP_WARN(this->get_logger(),
                  "Odometry gate: waiting for GNSS init. "
                  "pos=%d yaw=%d base=%d imu_queue=%zu lidar_time=%.6f "
                  "gnss_pos_time=%.6f gnss_yaw_time=%.6f",
                  gnss_snapshot.has_gnss_position_, gnss_snapshot.has_gnss_yaw,
                  has_base_snapshot, imu_queue_size, lidar_time,
                  gnss_snapshot.nmea_time, gnss_snapshot.yaw_time);
    }
    return;
  }
  if (has_prev_lidar_pose_for_prediction_ &&
      lidar_time <= prev_lidar_pose_time_ + node_params_.pose_dt_min) {
    RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(),
        node_params_.registration_log_throttle,
        "Odometry gate: dropping non-monotonic feature frame lidar_time=%.6f "
        "prev=%.6f",
        lidar_time, prev_lidar_pose_time_);
    return;
  }
  if (has_prev_lidar_pose_for_prediction_ && node_params_.pose_dt_max > 0.0) {
    const double lidar_dt = lidar_time - prev_lidar_pose_time_;
    if (lidar_dt > node_params_.pose_dt_max) {
      RCLCPP_WARN(this->get_logger(),
                  "Odometry gate: large LiDAR dt %.3f s exceeds %.3f s. "
                  "Trying GNSS correction and resetting local maps.",
                  lidar_dt, node_params_.pose_dt_max);

      const bool corrected_from_gnss = TryInitializeOdomFromGnss(true);
      if (!corrected_from_gnss) {
        ground_map_frames_.clear();
        objects_map_frames_.clear();
        ground_map_.clear();
        objects_map_.clear();
        has_lidar_imu_prev_ = false;
        {
          std::lock_guard<std::mutex> lock(mut_);
          has_prev_lidar_pose_for_prediction_ = false;
          prediction_queue_.Clear();
        }
        RCLCPP_WARN(this->get_logger(),
                    "Odometry gate: GNSS correction unavailable after large "
                    "LiDAR dt; reseeding local map at the previous pose.");
      }
    }
  }

  PruneStaleMapFrames(lidar_time);
  RebuildGroundMap();
  RebuildObjectsMap();

  if (ground_map_.empty() || objects_map_.empty()) {
    RCLCPP_WARN(this->get_logger(),
                "Either objects or ground map is empty %zu %zu",
                ground_map_.size(), objects_map_.size());

    auto patches_in_map = TransformPatchesToMap(features.ground_patches,
                                                R_map_lidar_, t_map_lidar);
    ground_map_frames_.push_back(GroundPatchFrame{patches_in_map, lidar_time});

    auto planar_lines_in_map =
        TransformLinesToMap(features.planar_lines, R_map_lidar_, t_map_lidar);
    objects_map_frames_.push_back(
        PlanarMapFrame{planar_lines_in_map, lidar_time});
    PruneStaleMapFrames(lidar_time);
    RebuildGroundMap();
    RebuildObjectsMap();
    {
      std::lock_guard<std::mutex> lock(mut_);
      prev_t_map_lidar_ = t_map_lidar;
      prev_lidar_pose_time_ = lidar_time;
      has_prev_lidar_pose_for_prediction_ = false;
      prediction_queue_.Clear();
    }

    RCLCPP_WARN(this->get_logger(),
                "Odometry gate: initialized first map frame, next frame should "
                "publish. "
                "pose t=[%.3f %.3f %.3f], yaw=%.3f planar_lines=%zu "
                "ground_patches=%zu",
                t_map_lidar.x(), t_map_lidar.y(), t_map_lidar.z(),
                std::atan2(R_map_lidar_(1, 0), R_map_lidar_(0, 0)),
                features.planar_lines.size(), features.ground_patches.size());
    return;
  }
  bool has_imu_history = false;
  size_t imu_queue_size = 0;
  {
    std::lock_guard<std::mutex> lock(mut_);
    imu_queue_size = imu_orientation_queue_.Size();
    has_imu_history = imu_queue_size >= 2;
  }
  const bool has_extrinsic =
      GetExtrinsicTf(*tf_buffer_, imu_frame, lidar_frame);
  if (!has_extrinsic || !has_imu_history) {
    if (ShouldLogSteady(last_odometry_gate_warn_time_,
                        node_params_.registration_log_throttle)) {
      RCLCPP_WARN(this->get_logger(),
                  "Odometry gate: missing %s%s. lidar_time=%.6f imu_queue=%zu",
                  has_extrinsic ? "" : "TF/extrinsic",
                  (!has_extrinsic && !has_imu_history)
                      ? "+IMU history"
                      : (has_imu_history ? "" : "IMU history"),
                  lidar_time, imu_queue_size);
    }
    return;
  }
  GnssData gnss_data_local;
  {
    std::lock_guard<std::mutex> lock(mut_);
    gnss_data_local = gnss_data_;
  }

  std::optional<Eigen::Quaterniond> q_lidar;
  {
    std::lock_guard<std::mutex> lock(mut_);
    q_lidar = GetImuOrientationAt(lidar_time);
  }

  if (!q_lidar) {
    if (ShouldLogSteady(last_odometry_gate_warn_time_,
                        node_params_.registration_log_throttle)) {
      double imu_first = 0.0;
      double imu_last = 0.0;
      size_t imu_queue_size_local = 0;
      {
        std::lock_guard<std::mutex> lock(mut_);
        imu_queue_size_local = imu_orientation_queue_.Size();
        if (imu_queue_size_local > 0) {
          imu_first = imu_orientation_queue_.PeerFront().time;
          imu_last = imu_orientation_queue_.PeerBack().time;
        }
      }
      RCLCPP_WARN(this->get_logger(),
                  "Odometry gate: no IMU orientation at lidar_time=%.6f "
                  "imu_queue=%zu imu_range=[%.6f, %.6f]",
                  lidar_time, imu_queue_size_local, imu_first, imu_last);
    }
    return;
  }

  if (!has_lidar_imu_prev_) {
    imu_q_lidar_prev_ = *q_lidar;
    has_lidar_imu_prev_ = true;
    if (ShouldLogSteady(last_odometry_gate_warn_time_,
                        node_params_.registration_log_throttle)) {
      RCLCPP_INFO(this->get_logger(),
                  "Odometry gate: primed first lidar/IMU orientation at %.6f",
                  lidar_time);
    }
    return;
  }
  // IMU relative rotation between previous and current LiDAR scan timestamps.
  // With q = R_world_imu / R_map_imu style body-to-world orientation,
  // R_prev.transpose() * R_curr maps vectors from current IMU frame to
  // previous IMU frame.
  Eigen::Matrix3d R_imu_delta_lidar =
      imu_q_lidar_prev_.toRotationMatrix().transpose() *
      q_lidar->toRotationMatrix();
  // Static extrinsic rotation between LiDAR and IMU frames.
  // q_imu_lidar comes from lookupTransform(imu_frame, lidar_frame), so R_ex
  // maps: lidar frame -> imu frame.
  Eigen::Matrix3d R_ex = q_imu_lidar.toRotationMatrix();
  // Convert IMU-frame delta into LiDAR-frame delta.
  // Result maps current LiDAR frame -> previous LiDAR frame.
  Eigen::Matrix3d R_lidar_delta = R_ex.transpose() * R_imu_delta_lidar * R_ex;
  imu_q_lidar_prev_ = *q_lidar;

  // Predicted absolute map rotation from previous pose + IMU delta
  Eigen::Matrix3d R_imu_prior_map = R_map_lidar_ * R_lidar_delta;

  // Planar initial guess from the current pose or wheel prediction.
  // The IMU path is only used for roll/pitch and short-term attitude delta; it
  // is not an absolute yaw source.
  PredictedPose pred;
  pred.t = t_map_lidar;
  pred.yaw = std::atan2(R_map_lidar_(1, 0), R_map_lidar_(0, 0));
  pred.speed = 0.0;

  FiniteDeque<TimedPredictedPose> prediction_queue_loc;
  bool has_last_speed_output_loc = false;
  double last_speed_output_loc = 0.0;
  {
    std::lock_guard<std::mutex> lock(mut_);
    prediction_queue_loc = prediction_queue_;
    has_last_speed_output_loc = has_last_speed_output_;
    last_speed_output_loc = last_speed_output_;
  }

  const auto queued_pred = FindClosestByTime(
      prediction_queue_loc, lidar_time,
      [](const TimedPredictedPose& item) { return item.time; });
  const bool has_queued_pred = queued_pred &&
                               std::abs(lidar_time - queued_pred->time) <=
                                   prediction_params_.max_wheel_data_age &&
                               std::isfinite(queued_pred->pose.speed);

  if (has_queued_pred) {
    pred = queued_pred->pose;
  } else if (has_last_speed_output_loc) {
    pred.speed = last_speed_output_loc;
  }

  PlanarRegistrationResult planar_reg_res;
  {
    Eigen::Vector2d t_planar_initial(pred.t.x(), pred.t.y());
    const double c = std::cos(pred.yaw);
    const double s = std::sin(pred.yaw);
    Eigen::Matrix2d R_planar_initial;
    R_planar_initial << c, -s, s, c;
    TimeMeasurments_t meas("planar_registration");

    planar_reg_res =
        planar_registration_.AlignLines(objects_map_, features.planar_lines,
                                        R_planar_initial, t_planar_initial);

    meas.SetEnd();
    time_measurments.push_back(meas);
  }

  if (node_params_.debug && planar_lines_pub_) {
    planar_lines_pub_->publish(
        planar_registration_.MakeLineMarkers(features.header, map_frame));
  }

  const double dt_registration_gate = lidar_time - prev_lidar_pose_time_;
  bool is_planar_ok =
      CheckPlanarRegistration(planar_reg_res, pred, dt_registration_gate);

  if (is_planar_ok) {
    const double max_dx =
        std::min(planar_reg_gate_params_.max_abs_dx,
                 planar_reg_gate_params_.max_dx * dt_registration_gate);
    const double max_dy =
        std::min(planar_reg_gate_params_.max_abs_dy,
                 planar_reg_gate_params_.max_dy * dt_registration_gate);
    const double max_dyaw = std::min(
        DegToRad(planar_reg_gate_params_.max_abs_dyaw),
        DegToRad(planar_reg_gate_params_.max_dyaw) * dt_registration_gate);

    const Eigen::Vector2d raw_t = planar_reg_res.t;
    const double raw_yaw =
        std::atan2(planar_reg_res.R(1, 0), planar_reg_res.R(0, 0));

    const double clamped_dx =
        std::clamp(raw_t.x() - pred.t.x(), -max_dx, max_dx);
    const double clamped_dy =
        std::clamp(raw_t.y() - pred.t.y(), -max_dy, max_dy);
    const double clamped_dyaw =
        std::clamp(NormalizeAngle(raw_yaw - pred.yaw), -max_dyaw, max_dyaw);

    const bool was_clamped =
        std::abs(clamped_dx - (raw_t.x() - pred.t.x())) > 1e-9 ||
        std::abs(clamped_dy - (raw_t.y() - pred.t.y())) > 1e-9 ||
        std::abs(clamped_dyaw - NormalizeAngle(raw_yaw - pred.yaw)) > 1e-9;

    if (was_clamped) {
      planar_reg_res.t.x() = pred.t.x() + clamped_dx;
      planar_reg_res.t.y() = pred.t.y() + clamped_dy;

      const double yaw = pred.yaw + clamped_dyaw;
      const double c = std::cos(yaw);
      const double ss = std::sin(yaw);
      planar_reg_res.R << c, -ss, ss, c;

      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(),
                           node_params_.registration_log_throttle,
                           "planar gate soft clamped: matches=%d residual=%.3f "
                           "dt=%.3f dx=%.3f->%.3f m dy=%.3f->%.3f m "
                           "dyaw=%.2f->%.2f deg",
                           planar_reg_res.matches, planar_reg_res.mean_residual,
                           dt_registration_gate, raw_t.x() - pred.t.x(),
                           clamped_dx, raw_t.y() - pred.t.y(), clamped_dy,
                           NormalizeAngle(raw_yaw - pred.yaw) * 180.0 / M_PI,
                           clamped_dyaw * 180.0 / M_PI);
    }
  }

  if (!is_planar_ok) {
    const double yaw_planar =
        std::atan2(planar_reg_res.R(1, 0), planar_reg_res.R(0, 0));
    const double dx_rate =
        std::isfinite(dt_registration_gate) && dt_registration_gate > 0.0
            ? std::abs(planar_reg_res.t.x() - pred.t.x()) / dt_registration_gate
            : std::numeric_limits<double>::infinity();
    const double dy_rate =
        std::isfinite(dt_registration_gate) && dt_registration_gate > 0.0
            ? std::abs(planar_reg_res.t.y() - pred.t.y()) / dt_registration_gate
            : std::numeric_limits<double>::infinity();
    const double dyaw_rate_deg =
        std::isfinite(dt_registration_gate) && dt_registration_gate > 0.0
            ? std::abs(NormalizeAngle(yaw_planar - pred.yaw)) /
                  dt_registration_gate * 180.0 / M_PI
            : std::numeric_limits<double>::infinity();
    const double abs_dx = std::abs(planar_reg_res.t.x() - pred.t.x());
    const double abs_dy = std::abs(planar_reg_res.t.y() - pred.t.y());
    const double abs_dyaw_deg =
        std::abs(NormalizeAngle(yaw_planar - pred.yaw)) * 180.0 / M_PI;
    RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(),
        node_params_.registration_log_throttle,
        "planar gate rejected: valid=%d matches=%d/%d residual=%.3f/%.3f "
        "dt=%.3f dx=%.3f/%.3f m/s dy=%.3f/%.3f m/s "
        "dyaw=%.2f/%.2f deg/s abs_dx=%.3f/%.3f m "
        "abs_dy=%.3f/%.3f m abs_dyaw=%.2f/%.2f deg "
        "planar_lines=%zu map_lines=%zu",
        planar_reg_res.valid, planar_reg_res.matches,
        planar_reg_gate_params_.min_matches, planar_reg_res.mean_residual,
        planar_reg_gate_params_.max_residual, dt_registration_gate, dx_rate,
        planar_reg_gate_params_.max_dx, dy_rate, planar_reg_gate_params_.max_dy,
        dyaw_rate_deg, planar_reg_gate_params_.max_dyaw, abs_dx,
        planar_reg_gate_params_.max_abs_dx, abs_dy,
        planar_reg_gate_params_.max_abs_dy, abs_dyaw_deg,
        planar_reg_gate_params_.max_abs_dyaw, features.planar_lines.size(),
        objects_map_.size());
  }

  GroundRegistrationResult ground_reg_result;
  {
    Eigen::Vector3d t_ground_initial;
    t_ground_initial.x() = is_planar_ok ? planar_reg_res.t.x() : pred.t.x();
    t_ground_initial.y() = is_planar_ok ? planar_reg_res.t.y() : pred.t.y();
    t_ground_initial.z() = pred.t.z();

    double yaw_planar = is_planar_ok ? std::atan2(planar_reg_res.R(1, 0),
                                                  planar_reg_res.R(0, 0))
                                     : pred.yaw;

    auto [roll_imu, pitch_imu, yaw_unused] =
        EulersFromMatrixSimple(R_imu_prior_map);
    (void)yaw_unused;

    Eigen::Matrix3d R_ground_initial =
        Eigen::AngleAxisd(yaw_planar, Eigen::Vector3d::UnitZ())
            .toRotationMatrix() *
        Eigen::AngleAxisd(pitch_imu, Eigen::Vector3d::UnitY())
            .toRotationMatrix() *
        Eigen::AngleAxisd(roll_imu, Eigen::Vector3d::UnitX())
            .toRotationMatrix();
    TimeMeasurments_t meas("ground_registration");
    ground_reg_result = ground_registration_.Align(
        ground_map_, features.ground_patches, R_imu_prior_map, R_ground_initial,
        t_ground_initial);
    meas.SetEnd();
    time_measurments.push_back(meas);
  }
  const double dt_ground_gate = dt_registration_gate;
  bool is_ground_ok =
      CheckGroundRegistration(ground_reg_result, dt_ground_gate);

  Eigen::Matrix3d R_abs;
  Eigen::Vector3d t_abs;
  if (is_ground_ok && is_planar_ok) {
    R_abs = MergeGroundAndPlanarRotation(ground_reg_result.R, R_map_lidar_,
                                         planar_reg_res.R,
                                         node_params_.pose_smoothing.alpha_rp,
                                         node_params_.pose_smoothing.alpha_yaw);
    t_abs = MergeGroundAndPlanarTranslation(
        ground_reg_result.t, pred.t, planar_reg_res.t,
        node_params_.pose_smoothing.alpha_xy,
        node_params_.pose_smoothing.alpha_z);
  } else {
    // At least one registration gate failed. Blend available accepted updates
    // with prediction, and take roll/pitch from the IMU prior in fallback.
    if (!is_ground_ok) {
      if (!ground_reg_result.valid) {
        RCLCPP_WARN_THROTTLE(
            this->get_logger(), *this->get_clock(),
            node_params_.registration_log_throttle,
            "ground gate rejected: invalid registration matches=%d/%d "
            "residual=%.3f/%.3f planar_gate_ok=%d planar_matches=%d "
            "planar_lines=%zu ground_patches=%zu",
            ground_reg_result.num_matches, ground_reg_gate_params_.min_matches,
            ground_reg_result.mean_abs_residual,
            ground_reg_gate_params_.max_residual, is_planar_ok,
            planar_reg_res.matches, features.planar_lines.size(),
            features.ground_patches.size());
      } else if (std::isfinite(dt_ground_gate) && dt_ground_gate > 0.0) {
        auto [roll_prev, pitch_prev, yaw_prev] =
            EulersFromMatrixSimple(R_map_lidar_);
        auto [roll_g, pitch_g, yaw_g] =
            EulersFromMatrixSimple(ground_reg_result.R);
        (void)yaw_prev;
        (void)yaw_g;

        const double droll = std::abs(std::atan2(std::sin(roll_g - roll_prev),
                                                 std::cos(roll_g - roll_prev)));
        const double dpitch = std::abs(std::atan2(
            std::sin(pitch_g - pitch_prev), std::cos(pitch_g - pitch_prev)));
        const double droll_rate_deg = droll / dt_ground_gate * 180.0 / M_PI;
        const double dpitch_rate_deg = dpitch / dt_ground_gate * 180.0 / M_PI;
        const double dz_rate =
            std::abs(ground_reg_result.t.z() - t_map_lidar.z()) /
            dt_ground_gate;
        RCLCPP_WARN_THROTTLE(
            this->get_logger(), *this->get_clock(),
            node_params_.registration_log_throttle,
            "ground gate rejected: matches=%d/%d residual=%.3f/%.3f "
            "dt=%.3f droll=%.2f/%.2f deg/s dpitch=%.2f/%.2f deg/s "
            "dz=%.3f/%.3f m/s",
            ground_reg_result.num_matches, ground_reg_gate_params_.min_matches,
            ground_reg_result.mean_abs_residual,
            ground_reg_gate_params_.max_residual, dt_ground_gate,
            droll_rate_deg, ground_reg_gate_params_.max_droll, dpitch_rate_deg,
            ground_reg_gate_params_.max_dpitch, dz_rate,
            ground_reg_gate_params_.max_dz);
      }
    } else if (!is_planar_ok) {
      RCLCPP_WARN_THROTTLE(
          this->get_logger(), *this->get_clock(),
          node_params_.registration_log_throttle,
          "pose merge fallback: ground gate accepted but planar gate rejected; "
          "using prediction for xy/yaw and ground for z. "
          "ground_matches=%d residual=%.3f planar_valid=%d "
          "planar_matches=%d/%d planar_residual=%.3f/%.3f",
          ground_reg_result.num_matches, ground_reg_result.mean_abs_residual,
          planar_reg_res.valid, planar_reg_res.matches,
          planar_reg_gate_params_.min_matches, planar_reg_res.mean_residual,
          planar_reg_gate_params_.max_residual);
    }
    const double yaw_planar =
        std::atan2(planar_reg_res.R(1, 0), planar_reg_res.R(0, 0));
    const double alpha_yaw =
        is_planar_ok ? node_params_.fallback_merge.alpha_yaw : 0.0;
    const double yaw =
        pred.yaw + alpha_yaw * NormalizeAngle(yaw_planar - pred.yaw);
    // const double yaw = yaw_planar;
    auto [roll_prior, pitch_prior, yaw_prior_unused] =
        EulersFromMatrixSimple(R_imu_prior_map);
    (void)yaw_prior_unused;

    R_abs =
        Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix() *
        Eigen::AngleAxisd(pitch_prior, Eigen::Vector3d::UnitY())
            .toRotationMatrix() *
        Eigen::AngleAxisd(roll_prior, Eigen::Vector3d::UnitX())
            .toRotationMatrix();

    const double alpha_xy =
        is_planar_ok ? node_params_.fallback_merge.alpha_xy : 0.0;
    const double alpha_z =
        ground_reg_result.valid
            ? node_params_.fallback_merge.alpha_z_valid_ground
            : 0.0;
    t_abs.x() = pred.t.x() + alpha_xy * (planar_reg_res.t.x() - pred.t.x());
    t_abs.y() = pred.t.y() + alpha_xy * (planar_reg_res.t.y() - pred.t.y());
    t_abs.z() = pred.t.z() + alpha_z * (ground_reg_result.t.z() - pred.t.z());
  }
  {
    auto [roll_candidate, pitch_candidate, yaw_candidate] =
        EulersFromMatrixSimple(R_abs);
    auto [roll_pred, pitch_pred, yaw_pred_unused] =
        EulersFromMatrixSimple(R_imu_prior_map);
    (void)yaw_pred_unused;

    const double max_xy_jump =
        std::max(0.1, planar_reg_gate_params_.max_abs_dx);
    const double max_z_jump = std::max(0.1, ground_reg_gate_params_.max_dz);
    const double max_yaw_jump = DegToRad(planar_reg_gate_params_.max_abs_dyaw);

    const double dx_final = t_abs.x() - pred.t.x();
    const double dy_final = t_abs.y() - pred.t.y();
    const double dz_final = t_abs.z() - pred.t.z();
    const double dyaw_final = NormalizeAngle(yaw_candidate - pred.yaw);

    bool clamped_final = false;
    if (std::abs(dx_final) > max_xy_jump) {
      t_abs.x() = pred.t.x() + std::clamp(dx_final, -max_xy_jump, max_xy_jump);
      clamped_final = true;
    }
    if (std::abs(dy_final) > max_xy_jump) {
      t_abs.y() = pred.t.y() + std::clamp(dy_final, -max_xy_jump, max_xy_jump);
      clamped_final = true;
    }
    if (std::abs(dz_final) > max_z_jump) {
      t_abs.z() = pred.t.z() + std::clamp(dz_final, -max_z_jump, max_z_jump);
      clamped_final = true;
    }
    if (std::abs(dyaw_final) > max_yaw_jump) {
      yaw_candidate =
          pred.yaw + std::clamp(dyaw_final, -max_yaw_jump, max_yaw_jump);
      R_abs = Eigen::AngleAxisd(yaw_candidate, Eigen::Vector3d::UnitZ())
                  .toRotationMatrix() *
              Eigen::AngleAxisd(pitch_candidate, Eigen::Vector3d::UnitY())
                  .toRotationMatrix() *
              Eigen::AngleAxisd(roll_candidate, Eigen::Vector3d::UnitX())
                  .toRotationMatrix();
      clamped_final = true;
    }

    if (clamped_final) {
      RCLCPP_WARN_THROTTLE(
          this->get_logger(), *this->get_clock(),
          node_params_.registration_log_throttle,
          "final pose correction clamped: d=[%.3f %.3f %.3f] yaw=%.2f deg "
          "limits=[%.3f %.3f %.2f deg]",
          dx_final, dy_final, dz_final, dyaw_final * 180.0 / M_PI, max_xy_jump,
          max_z_jump, max_yaw_jump * 180.0 / M_PI);
    }
    (void)roll_pred;
    (void)pitch_pred;
  }

  {
    std::lock_guard<std::mutex> lock(mut_);
    R_map_lidar_ = R_abs;
    t_map_lidar = t_abs;
    prev_t_map_lidar_ = t_abs;
    prev_lidar_pose_time_ = lidar_time;
    has_prev_lidar_pose_for_prediction_ = true;
    // prediction_queue_.Clear(); // to avoid error accumulation because of IMU
    // drift
  }

  auto planar_lines_in_map =
      TransformLinesToMap(features.planar_lines, R_map_lidar_, t_map_lidar);
  objects_map_frames_.push_back(
      PlanarMapFrame{planar_lines_in_map, lidar_time});

  while (objects_map_frames_.size() >
         static_cast<size_t>(node_params_.max_planar_map_frames)) {
    objects_map_frames_.pop_front();
  }

  if (features.ground_patches.size() >=
      static_cast<size_t>(node_params_.min_ground_patches_for_map_update)) {
    auto patches_in_map = TransformPatchesToMap(features.ground_patches,
                                                R_map_lidar_, t_map_lidar);

    ground_map_frames_.push_back(GroundPatchFrame{patches_in_map, lidar_time});

    while (ground_map_frames_.size() >
           static_cast<size_t>(node_params_.max_ground_map_frames)) {
      ground_map_frames_.pop_front();
    }
  }

  PruneStaleMapFrames(lidar_time);
  RebuildGroundMap();
  RebuildObjectsMap();
  // T_map_base_est = T_map_lidar_est * T_lidar_base
  Eigen::Matrix3d R_lidar_body = q_lidar_body.toRotationMatrix();

  Eigen::Matrix3d R_map_base_est = R_abs * R_lidar_body;
  Eigen::Vector3d t_map_base_est = t_abs + R_abs * t_lidar_body;

  const auto& [sigma_xy, sigma_yaw] =
      node_params_.est_cov_.GetXYYawSigmas(planar_reg_res);

  Eigen::Vector<double, 6> sigmas = Eigen::Vector<double, 6>(
      sigma_xy, sigma_xy, node_params_.est_cov_.z, node_params_.est_cov_.roll,
      node_params_.est_cov_.pitch, sigma_yaw);
  geometry_msgs::msg::PoseWithCovarianceStamped pose_msg =
      BuildPoseWithCovarianceMsg(features.header.stamp,
                                 node_params_.gnss_map_frame, R_map_base_est,
                                 t_map_base_est, sigmas);
  lidar_pose_pub_->publish(pose_msg);

  auto [roll_abs, pitch_abs, yaw_abs] = EulersFromMatrixSimple(R_map_base_est);
  geometry_msgs::msg::Point eulers_msg;
  eulers_msg.x = roll_abs;
  eulers_msg.y = pitch_abs;
  eulers_msg.z = yaw_abs;
  est_eulers_pub_->publish(eulers_msg);

  const double yaw_planar_debug =
      std::atan2(planar_reg_res.R(1, 0), planar_reg_res.R(0, 0));
  Eigen::Matrix3d R_pred_lidar =
      Eigen::AngleAxisd(pred.yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  Eigen::Vector3d pred_base_t = pred.t + R_pred_lidar * t_lidar_body;
  auto [pred_base_roll_unused, pred_base_pitch_unused, pred_base_yaw] =
      EulersFromMatrixSimple(R_pred_lidar * R_lidar_body);
  (void)pred_base_roll_unused;
  (void)pred_base_pitch_unused;

  Eigen::Matrix3d R_planar_lidar =
      Eigen::AngleAxisd(yaw_planar_debug, Eigen::Vector3d::UnitZ())
          .toRotationMatrix();
  Eigen::Vector3d planar_lidar_t(planar_reg_res.t.x(), planar_reg_res.t.y(),
                                 pred.t.z());
  Eigen::Vector3d planar_base_t =
      planar_lidar_t + R_planar_lidar * t_lidar_body;
  auto [planar_base_roll_unused, planar_base_pitch_unused, planar_base_yaw] =
      EulersFromMatrixSimple(R_planar_lidar * R_lidar_body);
  (void)planar_base_roll_unused;
  (void)planar_base_pitch_unused;

  DumpOdometryErrorCsvRow(features.header.stamp, R_map_base_est, t_map_base_est,
                          planar_reg_res, pred_base_t, pred_base_yaw,
                          planar_base_t, planar_base_yaw, is_planar_ok);
}

bool GaloOdometryComponent::OpenOdometryErrorCsvIfNeeded() {
  if (!node_params_.odom_error_csv_enabled) {
    return false;
  }
  if (odom_error_csv_.is_open()) {
    return true;
  }
  if (odom_error_csv_open_failed_) {
    return false;
  }

  odom_error_csv_.open(node_params_.odom_error_csv_path, std::ios::out);
  if (!odom_error_csv_) {
    odom_error_csv_open_failed_ = true;
    RCLCPP_ERROR(this->get_logger(), "Failed to open odometry error CSV: %s",
                 node_params_.odom_error_csv_path.c_str());
    return false;
  }

  odom_error_csv_
      << "odom_time,gt_time"
      << ",gt_x,gt_y,gt_z,gt_roll,gt_pitch,gt_yaw"
      << ",odom_x,odom_y,odom_z,odom_roll,odom_pitch,odom_yaw"
      << ",error_x,error_y,error_z,error_roll,error_pitch,error_yaw"
      << ",min_error_x,min_error_y,min_error_z,min_error_roll,min_"
         "error_pitch,min_error_yaw"
      << ",max_error_x,max_error_y,max_error_z,max_error_roll,max_"
         "error_pitch,max_error_yaw"
      << ",pred_yaw,planar_yaw,final_yaw,planar_yaw_delta"
      << ",pred_x,pred_y,planar_x,planar_y,final_x,final_y"
      << ",planar_dx,planar_dy,final_dx,final_dy"
      << ",planar_valid,planar_gate_ok,planar_matches,planar_mean_residual"
      << ",gnss_corrected\n";
  odom_error_csv_ << std::fixed << std::setprecision(9);
  RCLCPP_INFO(this->get_logger(), "Writing odometry error CSV: %s",
              node_params_.odom_error_csv_path.c_str());
  return true;
}

void GaloOdometryComponent::DumpOdometryErrorCsvRow(
    const builtin_interfaces::msg::Time& stamp, const Eigen::Matrix3d& R_odom,
    const Eigen::Vector3d& t_odom, const PlanarRegistrationResult& planar_res,
    const Eigen::Vector3d& pred_base_t, double pred_base_yaw,
    const Eigen::Vector3d& planar_base_t, double planar_base_yaw,
    bool planar_gate_ok) {
  if (!node_params_.odom_error_csv_enabled) {
    return;
  }

  const double odom_time = rclcpp::Time(stamp).seconds();
  std::optional<Pose6D> gt_pose;
  {
    std::lock_guard<std::mutex> lock(mut_);
    gt_pose = InterpolatePoseHistory(gt_pose_history_, odom_time);
  }
  if (!gt_pose) {
    return;
  }

  const bool gnss_corrected = gnss_corrected_since_last_report_;
  gnss_corrected_since_last_report_ = false;

  if (!OpenOdometryErrorCsvIfNeeded()) {
    return;
  }

  const Pose6D odom_pose = MakePose6D(R_odom, t_odom);
  const double final_yaw = odom_pose.yaw;
  const double planar_yaw_delta =
      NormalizeAngle(planar_base_yaw - pred_base_yaw);
  const double planar_dx = planar_base_t.x() - pred_base_t.x();
  const double planar_dy = planar_base_t.y() - pred_base_t.y();
  const double final_dx = t_odom.x() - pred_base_t.x();
  const double final_dy = t_odom.y() - pred_base_t.y();
  const auto gt = PoseToArray(*gt_pose);
  const auto odom = PoseToArray(odom_pose);
  std::array<double, 6> error{};
  for (size_t i = 0; i < error.size(); ++i) {
    error[i] = odom[i] - gt[i];
  }
  error[3] = NormalizeAngle(error[3]);
  error[4] = NormalizeAngle(error[4]);
  error[5] = NormalizeAngle(error[5]);

  if (!odom_error_stats_.initialized) {
    odom_error_stats_.min = error;
    odom_error_stats_.max = error;
    odom_error_stats_.initialized = true;
  } else {
    for (size_t i = 0; i < error.size(); ++i) {
      odom_error_stats_.min[i] = std::min(odom_error_stats_.min[i], error[i]);
      odom_error_stats_.max[i] = std::max(odom_error_stats_.max[i], error[i]);
    }
  }

  odom_error_csv_ << odom_time << ',' << odom_time;
  for (const double v : gt) odom_error_csv_ << ',' << v;
  for (const double v : odom) odom_error_csv_ << ',' << v;
  for (const double v : error) odom_error_csv_ << ',' << v;
  for (const double v : odom_error_stats_.min) odom_error_csv_ << ',' << v;
  for (const double v : odom_error_stats_.max) odom_error_csv_ << ',' << v;
  odom_error_csv_ << ',' << pred_base_yaw << ',' << planar_base_yaw << ','
                  << final_yaw << ',' << planar_yaw_delta << ','
                  << pred_base_t.x() << ',' << pred_base_t.y() << ','
                  << planar_base_t.x() << ',' << planar_base_t.y() << ','
                  << t_odom.x() << ',' << t_odom.y() << ',' << planar_dx << ','
                  << planar_dy << ',' << final_dx << ',' << final_dy << ','
                  << planar_res.valid << ',' << planar_gate_ok << ','
                  << planar_res.matches << ',' << planar_res.mean_residual
                  << ',' << gnss_corrected;
  odom_error_csv_ << '\n';
  odom_error_csv_.flush();
}

bool GaloOdometryComponent::GetTransformation(tf2_ros::Buffer& tf_buffer,
                                              const std::string& target_frame,
                                              const std::string& source_frame,
                                              Eigen::Quaterniond& q,
                                              Eigen::Vector3d& t) {
  try {
    geometry_msgs::msg::TransformStamped tf_msg = tf_buffer.lookupTransform(
        target_frame, source_frame, tf2::TimePointZero);
    q = Eigen::Quaterniond(
        tf_msg.transform.rotation.w, tf_msg.transform.rotation.x,
        tf_msg.transform.rotation.y, tf_msg.transform.rotation.z);
    q.normalize();

    t = Eigen::Vector3d(tf_msg.transform.translation.x,
                        tf_msg.transform.translation.y,
                        tf_msg.transform.translation.z);
    return true;
  } catch (const tf2::TransformException& ex) {
    RCLCPP_WARN(this->get_logger(), "Failed to get TF %s <- %s: %s",
                target_frame.c_str(), source_frame.c_str(), ex.what());
    return false;
  }
}

bool GaloOdometryComponent::GetOrientation(tf2_ros::Buffer& tf_buffer,
                                           const std::string& target_frame,
                                           const std::string& source_frame,
                                           Eigen::Quaterniond& q) {
  try {
    geometry_msgs::msg::TransformStamped tf_msg = tf_buffer.lookupTransform(
        target_frame, source_frame, tf2::TimePointZero);
    q = Eigen::Quaterniond(
        tf_msg.transform.rotation.w, tf_msg.transform.rotation.x,
        tf_msg.transform.rotation.y, tf_msg.transform.rotation.z);
    q.normalize();
    return true;
  } catch (const tf2::TransformException& ex) {
    RCLCPP_WARN(this->get_logger(), "Failed to get TF %s <- %s: %s",
                target_frame.c_str(), source_frame.c_str(), ex.what());
    return false;
  }
}

bool GaloOdometryComponent::GetTranslation(tf2_ros::Buffer& tf_buffer,
                                           const std::string& target_frame,
                                           const std::string& source_frame,
                                           Eigen::Vector3d& t) {
  try {
    geometry_msgs::msg::TransformStamped tf_msg = tf_buffer.lookupTransform(
        target_frame, source_frame, tf2::TimePointZero);
    t = Eigen::Vector3d(tf_msg.transform.translation.x,
                        tf_msg.transform.translation.y,
                        tf_msg.transform.translation.z);
    return true;
  } catch (const tf2::TransformException& ex) {
    RCLCPP_WARN(this->get_logger(), "Failed to get TF %s <- %s: %s",
                target_frame.c_str(), source_frame.c_str(), ex.what());
    return false;
  }
}

bool GaloOdometryComponent::GetExtrinsicTf(tf2_ros::Buffer& tf_buffer,
                                           const std::string& imu_frame,
                                           const std::string& lidar_frame) {
  std::lock_guard<std::mutex> lock(mut_);
  if (has_imu_lidar_extrinsic_) return has_imu_lidar_extrinsic_;

  bool res =
      GetOrientation(tf_buffer, imu_frame, lidar_frame, q_imu_lidar) &&
      GetTranslation(tf_buffer, map_frame, lidar_frame, t_map_lidar) &&
      GetTransformation(tf_buffer, lidar_frame, body_frame, q_lidar_body,
                        t_lidar_body) &&
      GetOrientation(tf_buffer, body_frame, imu_frame, q_body_imu) &&
      GetTranslation(tf_buffer, body_frame, pos_antena_frame, t_body_pos) &&
      GetTranslation(tf_buffer, body_frame, orientation_antenna_frame,
                     t_body_orientation);
  has_imu_lidar_extrinsic_ = res;
  return res;
}

bool GaloOdometryComponent::TryInitializeOdomFromGnss(bool force_correction) {
  constexpr double kMaxGnssDataAgeSec = 0.2;

  if (has_lidar_odom_initialized_from_gnss_ && !force_correction) {
    return true;
  }

  if (!GetExtrinsicTf(*tf_buffer_, imu_frame, lidar_frame)) {
    return false;
  }

  Eigen::Vector3d gnss_local;
  Eigen::Matrix3d R_base = Eigen::Matrix3d::Identity();

  {
    std::lock_guard<std::mutex> lock(mut_);

    if (!gnss_data_.has_gnss_position_ || !gnss_data_.has_gnss_yaw ||
        !has_latest_R_base_) {
      if (ShouldLogSteady(last_gnss_stale_warn_time_,
                          node_params_.initialization_log_throttle)) {
        RCLCPP_WARN(this->get_logger(),
                    "GNSS init unavailable: pos=%d yaw=%d base=%d",
                    gnss_data_.has_gnss_position_, gnss_data_.has_gnss_yaw,
                    has_latest_R_base_);
      }
      return false;
    }

    const double now = this->get_clock()->now().seconds();
    const double position_age = now - gnss_data_.nmea_time;
    const double yaw_age = now - gnss_data_.yaw_time;
    if (position_age < 0.0 || position_age > kMaxGnssDataAgeSec ||
        yaw_age < 0.0 || yaw_age > kMaxGnssDataAgeSec) {
      if (ShouldLogSteady(last_gnss_stale_warn_time_,
                          node_params_.initialization_log_throttle)) {
        RCLCPP_WARN(this->get_logger(),
                    "GNSS data is stale for LiDAR odometry correction: "
                    "position_age=%.3f s yaw_age=%.3f s",
                    position_age, yaw_age);
      }
      return false;
    }

    gnss_local = gnss_data_.gnss_local_;
    R_base = latest_R_base_;
  }

  Eigen::Vector3d t_body_antenna_mid = 0.5 * (t_body_pos + t_body_orientation);
  Eigen::Vector3d t_map_base = gnss_local - R_base * t_body_antenna_mid;

  Eigen::Matrix3d R_lidar_body = q_lidar_body.toRotationMatrix();
  Eigen::Matrix3d R_body_lidar = R_lidar_body.transpose();
  Eigen::Vector3d t_body_lidar = -R_body_lidar * t_lidar_body;

  {
    std::lock_guard<std::mutex> lock(mut_);
    R_map_lidar_ = R_base * R_body_lidar;
    t_map_lidar = t_map_base + R_base * t_body_lidar;
    prev_t_map_lidar_ = t_map_lidar;
    has_prev_lidar_pose_for_prediction_ = false;
    prediction_queue_.Clear();
  }

  has_lidar_odom_initialized_from_gnss_ = true;

  if (force_correction) {
    ground_map_frames_.clear();
    objects_map_frames_.clear();
    ground_map_.clear();
    objects_map_.clear();
    has_lidar_imu_prev_ = false;
    gnss_corrected_since_last_report_ = true;
    RCLCPP_INFO(this->get_logger(),
                "GNSS correction reset local maps; next frame will seed a new "
                "local map");
  }

  auto [base_roll, base_pitch, base_yaw] = EulersFromMatrixSimple(R_base);
  auto [lidar_roll, lidar_pitch, lidar_yaw] =
      EulersFromMatrixSimple(R_map_lidar_);

  RCLCPP_INFO(this->get_logger(),
              "%s LiDAR odometry from GNSS+IMU: "
              "t=[%.3f %.3f %.3f], "
              "base_rpy=[%.3f %.3f %.3f] deg, "
              "lidar_rpy=[%.3f %.3f %.3f] deg",
              force_correction ? "Corrected" : "Initialized", t_map_lidar.x(),
              t_map_lidar.y(), t_map_lidar.z(), base_roll * 180.0 / M_PI,
              base_pitch * 180.0 / M_PI, base_yaw * 180.0 / M_PI,
              lidar_roll * 180.0 / M_PI, lidar_pitch * 180.0 / M_PI,
              lidar_yaw * 180.0 / M_PI);

  return true;
}

void GaloOdometryComponent::PrintTimeMeasurments(
    const std::vector<TimeMeasurments_t>& measurments) {
  if (measurments.empty()) return;
  bool has_big_meas = false;
  for (const auto& m : measurments) {
    if (m.label.find("total") != std::string::npos) {
      continue;
    }
    if (GetDelayMs(m.start, m.end) >= node_params_.elapsed_time_thresh) {
      has_big_meas = true;
      break;
    }
  }
  if (!has_big_meas) return;
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
  if (ShouldLogSteady(last_timing_info_time_,
                      node_params_.registration_log_throttle)) {
    RCLCPP_INFO(this->get_logger(), "%s", ss.str().c_str());
  }
}

std::optional<Eigen::Quaterniond> GaloOdometryComponent::GetImuOrientationAt(
    double query_time) const {
  if (imu_orientation_queue_.Size() < 2) {
    return {};
  }

  if (query_time < imu_orientation_queue_.PeerFront().time ||
      query_time > imu_orientation_queue_.PeerBack().time) {
    return {};
  }

  size_t idx = 0;
  while (idx + 1 < imu_orientation_queue_.Size() &&
         imu_orientation_queue_[idx + 1].time < query_time) {
    ++idx;
  }

  const auto& a = imu_orientation_queue_[idx];
  const auto& b = imu_orientation_queue_[idx + 1];

  const double dt = b.time - a.time;
  if (dt <= 1e-9) {
    return a.q;
  }

  const double alpha = (query_time - a.time) / dt;
  return a.q.slerp(alpha, b.q);
}

std::vector<GroundPatch> GaloOdometryComponent::TransformPatchesToMap(
    const std::vector<GroundPatch>& patches, const Eigen::Matrix3d& R,
    const Eigen::Vector3d& t) {
  std::vector<GroundPatch> out;
  out.reserve(patches.size());

  for (auto p : patches) {
    p.centroid = R * p.centroid + t;
    p.normal = R * p.normal;
    p.normal.normalize();
    out.push_back(p);
  }
  return out;
}

std::vector<PlanarLine> GaloOdometryComponent::TransformLinesToMap(
    const std::vector<PlanarLine>& lines, const Eigen::Matrix3d& R,
    const Eigen::Vector3d& t) {
  std::vector<PlanarLine> out;
  out.reserve(lines.size());
  const double yaw = std::atan2(R(1, 0), R(0, 0));

  const double c = std::cos(yaw);
  const double s = std::sin(yaw);

  Eigen::Matrix2d R_2d;
  R_2d << c, -s, s, c;

  Eigen::Vector2d t_2d(t.x(), t.y());
  for (auto line : lines) {
    line.center = R_2d * line.center + t_2d;
    line.z = t.z() + line.z;
    line.direction = R_2d * line.direction;
    if (line.direction.squaredNorm() > 1e-12) line.direction.normalize();
    line.normal = R_2d * line.normal;
    if (line.normal.squaredNorm() > 1e-12) line.normal.normalize();
    out.push_back(line);
  }
  return out;
}

void GaloOdometryComponent::RebuildGroundMap() {
  ground_map_.clear();

  for (const auto& frame : ground_map_frames_) {
    ground_map_.insert(ground_map_.end(), frame.patches.begin(),
                       frame.patches.end());
  }
}

void GaloOdometryComponent::RebuildObjectsMap() {
  objects_map_.clear();

  for (const auto& frame : objects_map_frames_) {
    objects_map_.insert(objects_map_.end(), frame.lines.begin(),
                        frame.lines.end());
  }
}

void GaloOdometryComponent::PruneStaleMapFrames(double current_time) {
  const double max_age_sec = node_params_.map_stale_threshold_ms * 1e-3;
  if (max_age_sec <= 0.0) return;

  size_t pruned_ground_frames = 0;
  while (!ground_map_frames_.empty() &&
         current_time - ground_map_frames_.front().time > max_age_sec) {
    ground_map_frames_.pop_front();
    ++pruned_ground_frames;
  }

  size_t pruned_object_frames = 0;
  while (!objects_map_frames_.empty() &&
         current_time - objects_map_frames_.front().time > max_age_sec) {
    objects_map_frames_.pop_front();
    ++pruned_object_frames;
  }

  if (pruned_ground_frames > 0 || pruned_object_frames > 0) {
    if (ShouldLogSteady(last_prune_info_time_,
                        node_params_.registration_log_throttle)) {
      RCLCPP_INFO(
          this->get_logger(),
          "Pruned stale map frames: ground=%zu objects=%zu "
          "threshold=%.1f ms remaining_ground=%zu remaining_objects=%zu",
          pruned_ground_frames, pruned_object_frames,
          node_params_.map_stale_threshold_ms, ground_map_frames_.size(),
          objects_map_frames_.size());
    }
  }
}

Eigen::Matrix3d GaloOdometryComponent::MergeGroundAndPlanarRotation(
    const Eigen::Matrix3d& R_ground, const Eigen::Matrix3d& R_prior,
    const Eigen::Matrix2d& R_planar, double alpha_rp, double alpha_y) {
  auto [r_prior, p_prior, y_prior] = EulersFromMatrixSimple(R_prior);
  auto [r_ground, p_ground, y_ground] = EulersFromMatrixSimple(R_ground);
  (void)y_ground;

  double r_final = r_prior + alpha_rp * NormalizeAngle(r_ground - r_prior);
  double p_final = p_prior + alpha_rp * NormalizeAngle(p_ground - p_prior);

  double y_planar = std::atan2(R_planar(1, 0), R_planar(0, 0));
  double y_final = y_prior + alpha_y * NormalizeAngle(y_planar - y_prior);

  Eigen::AngleAxisd roll_rot(r_final, Eigen::Vector3d::UnitX());
  Eigen::AngleAxisd pitch_rot(p_final, Eigen::Vector3d::UnitY());
  Eigen::AngleAxisd yaw_rot(y_final, Eigen::Vector3d::UnitZ());

  return yaw_rot.toRotationMatrix() * pitch_rot.toRotationMatrix() *
         roll_rot.toRotationMatrix();
}

Eigen::Vector3d GaloOdometryComponent::MergeGroundAndPlanarTranslation(
    const Eigen::Vector3d& t_ground, const Eigen::Vector3d& t_prior,
    const Eigen::Vector2d& t_planar, double alpha_xy, double alpha_z) {
  Eigen::Vector3d t;
  t.x() = t_prior.x() + alpha_xy * (t_planar.x() - t_prior.x());
  t.y() = t_prior.y() + alpha_xy * (t_planar.y() - t_prior.y());

  double z_final = t_prior.z() + alpha_z * (t_ground.z() - t_prior.z());
  t.z() = z_final;
  return t;
}

geometry_msgs::msg::PoseWithCovarianceStamped
GaloOdometryComponent::BuildPoseWithCovarianceMsg(
    const builtin_interfaces::msg::Time& time, const std::string& frame,
    const Eigen::Matrix3d& R, const Eigen::Vector3d& t,
    const Eigen::Vector<double, 6>& sigmas) {
  geometry_msgs::msg::PoseWithCovarianceStamped pose_msg;
  pose_msg.header.stamp = time;
  pose_msg.header.frame_id = frame;

  pose_msg.pose.pose.position.x = t.x();
  pose_msg.pose.pose.position.y = t.y();
  pose_msg.pose.pose.position.z = t.z();

  auto [roll, pitch, yaw] = EulersFromMatrixSimple(R);

  tf2::Quaternion q;
  q.setRPY(roll, pitch, yaw);
  q.normalize();

  pose_msg.pose.pose.orientation = tf2::toMsg(q);

  auto& cov = pose_msg.pose.covariance;
  std::fill(cov.begin(), cov.end(), 0.0);

  cov[0] = std::pow(sigmas(0), 2);
  cov[7] = std::pow(sigmas(1), 2);
  cov[14] = std::pow(sigmas(2), 2);

  cov[21] = std::pow(sigmas(3), 2);
  cov[28] = std::pow(sigmas(4), 2);
  cov[35] = std::pow(sigmas(5), 2);
  return pose_msg;
}

bool GaloOdometryComponent::CheckPlanarRegistration(
    const PlanarRegistrationResult& res, const PredictedPose& pred, double dt) {
  if (!std::isfinite(dt) || dt <= 0.0) {
    return false;
  }

  if (!res.valid || res.matches < planar_reg_gate_params_.min_matches ||
      res.mean_residual > planar_reg_gate_params_.max_residual) {
    return false;
  }

  const double yaw_planar = std::atan2(res.R(1, 0), res.R(0, 0));
  const double dx = std::abs(res.t.x() - pred.t.x());
  const double dy = std::abs(res.t.y() - pred.t.y());
  const double dyaw = std::abs(NormalizeAngle(yaw_planar - pred.yaw));

  const double max_dx = std::min(planar_reg_gate_params_.max_abs_dx,
                                 planar_reg_gate_params_.max_dx * dt);
  const double max_dy = std::min(planar_reg_gate_params_.max_abs_dy,
                                 planar_reg_gate_params_.max_dy * dt);
  const double max_dyaw =
      std::min(DegToRad(planar_reg_gate_params_.max_abs_dyaw),
               DegToRad(planar_reg_gate_params_.max_dyaw) * dt);

  return dx <= max_dx && dy <= max_dy && dyaw <= max_dyaw;
}

bool GaloOdometryComponent::CheckGroundRegistration(
    const GroundRegistrationResult& res, double dt) {
  if (!std::isfinite(dt) || dt <= 0.0) {
    return false;
  }

  if (!res.valid || res.num_matches < ground_reg_gate_params_.min_matches ||
      res.mean_abs_residual > ground_reg_gate_params_.max_residual) {
    return false;
  }
  auto [roll_prev, pitch_prev, yaw_prev] = EulersFromMatrixSimple(R_map_lidar_);
  auto [roll_g, pitch_g, yaw_g] = EulersFromMatrixSimple(res.R);
  (void)yaw_prev;
  (void)yaw_g;

  const double droll = std::abs(std::atan2(std::sin(roll_g - roll_prev),
                                           std::cos(roll_g - roll_prev))) /
                       dt;
  const double dpitch = std::abs(std::atan2(std::sin(pitch_g - pitch_prev),
                                            std::cos(pitch_g - pitch_prev))) /
                        dt;
  const double dz = std::abs(res.t.z() - t_map_lidar.z()) / dt;

  return droll < DegToRad(ground_reg_gate_params_.max_droll) &&
         dpitch < DegToRad(ground_reg_gate_params_.max_dpitch) &&
         dz < ground_reg_gate_params_.max_dz;
}

RCLCPP_COMPONENTS_REGISTER_NODE(GaloOdometryComponent)
