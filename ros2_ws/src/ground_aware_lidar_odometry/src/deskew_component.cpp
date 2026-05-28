#include "ground_aware_lidar_odometry/deskew_component.hpp"

#include <exception>
#include <sstream>

#include "rclcpp_components/register_node_macro.hpp"

namespace ground_aware_lidar_odometry {
namespace {

template <typename T>
T DeclareAndGet(rclcpp::Node& node, const std::string& name,
                const T& default_value) {
  return node.declare_parameter<T>(name, default_value);
}

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

}  // namespace

GaloDeskewComponent::GaloDeskewComponent(const rclcpp::NodeOptions& options)
    : Node("galo_deskew", options),
      params_(LoadParams(*this)),
      deskew_params_(LoadDeskewParams(*this)),
      prediction_params_(LoadPredictionParams(*this)),
      position_predictor_(prediction_params_, this->get_logger(),
                          *this->get_clock()),
      deskew_algorithm_(deskew_params_, this->get_logger(),
                        *this->get_clock()) {
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  lidar_callback_group_ =
      this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  sensor_callback_group_ =
      this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  rclcpp::SubscriptionOptions lidar_options;
  lidar_options.callback_group = lidar_callback_group_;

  rclcpp::SubscriptionOptions sensor_options;
  sensor_options.callback_group = sensor_callback_group_;

  lidar_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      params_.lidar_topic, rclcpp::SensorDataQoS().keep_last(10),
      std::bind(&GaloDeskewComponent::LidarCb, this, std::placeholders::_1),
      lidar_options);
  imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
      params_.imu_topic, 500,
      std::bind(&GaloDeskewComponent::ImuCb, this, std::placeholders::_1),
      sensor_options);
  wheel_speed_sub_ = this->create_subscription<common_msgs::msg::WheelSpeed>(
      params_.wheel_speed_topic, 100,
      std::bind(&GaloDeskewComponent::WheelSpeedCb, this,
                std::placeholders::_1),
      sensor_options);
  wheel_angle_sub_ = this->create_subscription<qarl_msgs::msg::WAngleFeedback>(
      params_.wheel_angle_topic, 100,
      std::bind(&GaloDeskewComponent::WheelAngleCb, this,
                std::placeholders::_1),
      sensor_options);
  deskew_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
      params_.deskewed_cloud_topic, 10);
  deskew_heartbeat_pub_ = this->create_publisher<std_msgs::msg::Header>(
      params_.deskewed_heartbeat_topic, 10);

  RCLCPP_INFO(this->get_logger(), "GALO deskew component: %s -> %s",
              params_.lidar_topic.c_str(),
              params_.deskewed_cloud_topic.c_str());
}

void GaloDeskewComponent::LidarCb(
    const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
  try {
    const auto callback_start = std::chrono::steady_clock::now();
    std::vector<TimeMeasurments_t> measurements;

    {
      TimeMeasurments_t meas("deskew_tf_lookup");
      const bool has_tf = EnsureLidarBodyTf();
      meas.SetEnd();
      measurements.push_back(meas);
      if (!has_tf) {
        PrintTimeMeasurements(measurements);
        return;
      }
    }

    std::optional<DeskewInput> input;
    Eigen::Matrix3d R_lidar_body = Eigen::Matrix3d::Identity();
    {
      TimeMeasurments_t meas("deskew_queue_update");
      std::lock_guard<std::mutex> lock(mutex_);
      deskew_algorithm_.UpdateLidarQueue(msg);
      input = deskew_algorithm_.TakeReadyCloud();
      R_lidar_body = q_lidar_body_.toRotationMatrix();
      meas.SetEnd();
      measurements.push_back(meas);
    }

    if (!input) {
      TimeMeasurments_t callback_meas("deskew_callback_total");
      callback_meas.start = callback_start;
      callback_meas.SetEnd();
      measurements.push_back(callback_meas);
      PrintTimeMeasurements(measurements);
      return;
    }

    std::optional<sensor_msgs::msg::PointCloud2> deskewed;
    {
      TimeMeasurments_t meas("deskew_cloud");
      deskewed = deskew_algorithm_.DeskewCloud(*input, R_lidar_body);
      meas.SetEnd();
      measurements.push_back(meas);
    }

    if (deskewed) {
      TimeMeasurments_t meas("deskew_publish");
      deskew_pub_->publish(*deskewed);
      deskew_heartbeat_pub_->publish(deskewed->header);
      meas.SetEnd();
      measurements.push_back(meas);
    }

    TimeMeasurments_t callback_meas("deskew_callback_total");
    callback_meas.start = callback_start;
    callback_meas.SetEnd();
    measurements.push_back(callback_meas);
    PrintTimeMeasurements(measurements);
  } catch (const std::exception& ex) {
    RCLCPP_ERROR(this->get_logger(), "Dropping lidar message: %s", ex.what());
  }
}

void GaloDeskewComponent::ImuCb(const sensor_msgs::msg::Imu::SharedPtr msg) {
  try {
    std::lock_guard<std::mutex> lock(mutex_);
    deskew_algorithm_.UpdateImuQueue(msg->angular_velocity.z,
                                     rclcpp::Time(msg->header.stamp).seconds());
  } catch (const std::exception& ex) {
    RCLCPP_ERROR(this->get_logger(), "Dropping IMU message: %s", ex.what());
  }
}

void GaloDeskewComponent::WheelSpeedCb(
    const common_msgs::msg::WheelSpeed::SharedPtr msg) {
  try {
    std::lock_guard<std::mutex> lock(mutex_);
    wheel_data_.wheel_angle = last_wheel_angle_;
    wheel_data_.left_speed = msg->rear_left;
    wheel_data_.right_speed = msg->rear_right;
    wheel_data_.wheel_time = rclcpp::Time(msg->header.stamp).seconds();
    wheel_data_.has_wheel_data = true;

    const RearWheelSpeedResult speed_result =
        position_predictor_.EstimateRearAxleSpeed(wheel_data_);
    if (speed_result.valid) {
      deskew_algorithm_.UpdateSpeedQueue(speed_result.speed,
                                         wheel_data_.wheel_time);
    }
  } catch (const std::exception& ex) {
    RCLCPP_ERROR(this->get_logger(), "Dropping wheel speed message: %s",
                 ex.what());
  }
}

void GaloDeskewComponent::WheelAngleCb(
    const qarl_msgs::msg::WAngleFeedback::SharedPtr msg) {
  std::lock_guard<std::mutex> lock(mutex_);
  last_wheel_angle_ = msg->wangle;
}

void GaloDeskewComponent::PrintTimeMeasurements(
    const std::vector<TimeMeasurments_t>& measurements) {
  if (measurements.empty()) return;

  bool has_big_measurement = false;
  for (const auto& m : measurements) {
    if (m.label.find("total") != std::string::npos) {
      continue;
    }
    if (GetDelayMs(m.start, m.end) >= params_.elapsed_time_thresh) {
      has_big_measurement = true;
      break;
    }
  }
  if (!has_big_measurement) return;

  std::stringstream ss;
  ss << "Deskew time measurements (ms): ";
  bool first = true;
  for (const auto& measurement : measurements) {
    if (first)
      first = false;
    else
      ss << ", ";
    ss << measurement.label << " - "
       << GetDelayMs(measurement.start, measurement.end);
  }

  if (ShouldLogSteady(last_timing_info_time_, deskew_params_.log_throttle)) {
    RCLCPP_INFO(this->get_logger(), "%s", ss.str().c_str());
  }
}

bool GaloDeskewComponent::EnsureLidarBodyTf() {
  std::lock_guard<std::mutex> lock(mutex_);
  if (has_lidar_body_tf_) {
    return true;
  }

  try {
    const geometry_msgs::msg::TransformStamped tf_msg =
        tf_buffer_->lookupTransform(params_.lidar_frame, params_.body_frame,
                                    tf2::TimePointZero);
    q_lidar_body_ = Eigen::Quaterniond(
        tf_msg.transform.rotation.w, tf_msg.transform.rotation.x,
        tf_msg.transform.rotation.y, tf_msg.transform.rotation.z);
    q_lidar_body_.normalize();
    t_lidar_body_ = Eigen::Vector3d(tf_msg.transform.translation.x,
                                    tf_msg.transform.translation.y,
                                    tf_msg.transform.translation.z);
    has_lidar_body_tf_ = true;
  } catch (const tf2::TransformException& ex) {
    const auto now = std::chrono::steady_clock::now();
    const auto throttle =
        std::chrono::milliseconds(deskew_params_.log_throttle);
    if (last_tf_warn_time_ == std::chrono::steady_clock::time_point{} ||
        now - last_tf_warn_time_ >= throttle) {
      last_tf_warn_time_ = now;
      RCLCPP_WARN(this->get_logger(), "Failed to get TF %s <- %s: %s",
                  params_.lidar_frame.c_str(), params_.body_frame.c_str(),
                  ex.what());
    }
    return false;
  }

  return true;
}

GaloDeskewComponent::Params GaloDeskewComponent::LoadParams(
    rclcpp::Node& node) {
  Params p;
  p.lidar_frame =
      DeclareAndGet<std::string>(node, "frames.lidar_frame", p.lidar_frame);
  p.body_frame =
      DeclareAndGet<std::string>(node, "frames.body_frame", p.body_frame);
  p.lidar_topic =
      DeclareAndGet<std::string>(node, "topics.lidar", p.lidar_topic);
  p.imu_topic = DeclareAndGet<std::string>(node, "topics.imu", p.imu_topic);
  p.wheel_speed_topic = DeclareAndGet<std::string>(node, "topics.wheel_speed",
                                                   p.wheel_speed_topic);
  p.wheel_angle_topic = DeclareAndGet<std::string>(node, "topics.wheel_angle",
                                                   p.wheel_angle_topic);
  p.deskewed_cloud_topic = DeclareAndGet<std::string>(
      node, "topics.deskewed_cloud", p.deskewed_cloud_topic);
  p.deskewed_heartbeat_topic = DeclareAndGet<std::string>(
      node, "topics.deskewed_heartbeat", p.deskewed_heartbeat_topic);
  p.elapsed_time_thresh = DeclareAndGet<double>(
      node, "node.elapsed_time_thresh", p.elapsed_time_thresh);
  return p;
}

DeskewParams GaloDeskewComponent::LoadDeskewParams(rclcpp::Node& node) {
  DeskewParams p;
  p.imu_queue_size =
      DeclareAndGet<int>(node, "deskew.imu_queue_size", p.imu_queue_size);
  p.lidar_queue_size =
      DeclareAndGet<int>(node, "deskew.lidar_queue_size", p.lidar_queue_size);
  p.speed_queue_size =
      DeclareAndGet<int>(node, "deskew.speed_queue_size", p.speed_queue_size);
  p.num_threads = DeclareAndGet<int>(node, "deskew.num_threads", p.num_threads);
  p.scan_period_ =
      DeclareAndGet<double>(node, "deskew.scan_period", p.scan_period_);
  p.stamp_is_scan_end_ = DeclareAndGet<bool>(node, "deskew.stamp_is_scan_end",
                                             p.stamp_is_scan_end_);
  p.log_throttle =
      DeclareAndGet<int>(node, "deskew.log_throttle", p.log_throttle);
  p.debug = DeclareAndGet<int>(node, "deskew.debug", p.debug);
  p.min_time_epsilon = DeclareAndGet<double>(node, "deskew.min_time_epsilon",
                                             p.min_time_epsilon);
  p.relative_time_tolerance = DeclareAndGet<double>(
      node, "deskew.relative_time_tolerance", p.relative_time_tolerance);
  p.azimuth_range_epsilon = DeclareAndGet<double>(
      node, "deskew.azimuth_range_epsilon", p.azimuth_range_epsilon);
  p.max_speed_age =
      DeclareAndGet<double>(node, "deskew.max_speed_age", p.max_speed_age);
  return p;
}

PredictionParams GaloDeskewComponent::LoadPredictionParams(rclcpp::Node& node) {
  PredictionParams p;
  p.rear_track_ =
      DeclareAndGet<double>(node, "prediction.rear_track", p.rear_track_);
  p.wheelbase_ =
      DeclareAndGet<double>(node, "prediction.wheelbase", p.wheelbase_);
  p.max_diff_residual = DeclareAndGet<double>(
      node, "prediction.max_diff_residual", p.max_diff_residual);
  p.max_jump = DeclareAndGet<double>(node, "prediction.max_jump", p.max_jump);
  p.min_speed_for_turn_check = DeclareAndGet<double>(
      node, "prediction.min_speed_for_turn_check", p.min_speed_for_turn_check);
  p.tau = DeclareAndGet<double>(node, "prediction.tau", p.tau);
  p.min_prediction_dt = DeclareAndGet<double>(
      node, "prediction.min_prediction_dt", p.min_prediction_dt);
  p.max_prediction_dt = DeclareAndGet<double>(
      node, "prediction.max_prediction_dt", p.max_prediction_dt);
  p.max_wheel_data_age = DeclareAndGet<double>(
      node, "prediction.max_wheel_data_age", p.max_wheel_data_age);
  p.min_valid_speed = DeclareAndGet<double>(node, "prediction.min_valid_speed",
                                            p.min_valid_speed);
  p.max_steering_correction = DeclareAndGet<double>(
      node, "prediction.max_steering_correction", p.max_steering_correction);
  return p;
}

}  // namespace ground_aware_lidar_odometry

RCLCPP_COMPONENTS_REGISTER_NODE(
    ground_aware_lidar_odometry::GaloDeskewComponent)
