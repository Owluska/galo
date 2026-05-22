#include "ground_aware_lidar_odometry/frontend_component.hpp"

#include <exception>
#include <sstream>

#include "ground_aware_lidar_odometry/feature_conversions.hpp"
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

GaloFrontendComponent::GaloFrontendComponent(const rclcpp::NodeOptions& options)
    : Node("galo_frontend", options),
      params_(LoadParams(*this)),
      segmentation_params_(LoadGroundSegmentationParams(*this)),
      ground_patch_params_(LoadGroundPatchParams(*this)),
      planar_registration_params_(LoadPlanarRegistrationParams(*this)),
      frontend_(segmentation_params_, ground_patch_params_,
                planar_registration_params_, this->get_logger(),
                *this->get_clock()) {
  callback_group_ =
      this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  rclcpp::SubscriptionOptions options_in;
  options_in.callback_group = callback_group_;

  cloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      params_.deskewed_cloud_topic, rclcpp::SensorDataQoS().keep_last(1),
      std::bind(&GaloFrontendComponent::CloudCb, this, std::placeholders::_1),
      options_in);
  features_pub_ =
      this->create_publisher<ground_aware_lidar_odometry::msg::FrameFeatures>(
          params_.frame_features_topic, 1);
  colored_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
      params_.colored_cloud_topic, 1);
  ground_patches_pub_ =
      this->create_publisher<visualization_msgs::msg::MarkerArray>(
          params_.ground_patches_topic, 10);

  RCLCPP_INFO(this->get_logger(), "GALO frontend component: %s -> %s",
              params_.deskewed_cloud_topic.c_str(),
              params_.frame_features_topic.c_str());
}

void GaloFrontendComponent::CloudCb(
    const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
  try {
    const auto callback_start = std::chrono::steady_clock::now();
    FrontendResult result = frontend_.Extract(*msg, params_.debug);

    TimeMeasurments_t conversion_meas("feature_msg_conversion");
    auto features_msg = ToMsg(result.features);
    conversion_meas.SetEnd();
    result.measurements.push_back(conversion_meas);

    if (result.colored_cloud) {
      colored_pub_->publish(*result.colored_cloud);
    }
    if (result.ground_markers) {
      ground_patches_pub_->publish(*result.ground_markers);
    }

    TimeMeasurments_t publish_meas("feature_publish");
    features_pub_->publish(features_msg);
    publish_meas.SetEnd();
    result.measurements.push_back(publish_meas);

    TimeMeasurments_t callback_meas("frontend_callback_total");
    callback_meas.start = callback_start;
    callback_meas.SetEnd();
    result.measurements.push_back(callback_meas);

    PrintTimeMeasurements(result.measurements);
  } catch (const std::exception& ex) {
    RCLCPP_ERROR(this->get_logger(), "Dropping frontend cloud: %s", ex.what());
  }
}

void GaloFrontendComponent::PrintTimeMeasurements(
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
  ss << "Frontend time measurements (ms): ";
  bool first = true;
  for (const auto& measurement : measurements) {
    if (first)
      first = false;
    else
      ss << ", ";
    ss << measurement.label << " - "
       << GetDelayMs(measurement.start, measurement.end);
  }

  if (ShouldLogSteady(last_timing_info_time_, params_.log_throttle)) {
    RCLCPP_INFO(this->get_logger(), "%s", ss.str().c_str());
  }
}

GaloFrontendComponent::Params GaloFrontendComponent::LoadParams(
    rclcpp::Node& node) {
  Params p;
  p.debug = DeclareAndGet<int>(node, "node.debug", p.debug);
  p.elapsed_time_thresh = DeclareAndGet<double>(
      node, "node.elapsed_time_thresh", p.elapsed_time_thresh);
  p.log_throttle = DeclareAndGet<int>(node, "node.registration_log_throttle",
                                      p.log_throttle);
  p.deskewed_cloud_topic = DeclareAndGet<std::string>(
      node, "topics.deskewed_cloud", p.deskewed_cloud_topic);
  p.frame_features_topic = DeclareAndGet<std::string>(
      node, "topics.frame_features", p.frame_features_topic);
  p.colored_cloud_topic = DeclareAndGet<std::string>(
      node, "topics.colored_cloud", p.colored_cloud_topic);
  p.ground_patches_topic = DeclareAndGet<std::string>(
      node, "topics.ground_patches", p.ground_patches_topic);
  return p;
}

GroundSegmentationParams GaloFrontendComponent::LoadGroundSegmentationParams(
    rclcpp::Node& node) {
  GroundSegmentationParams p;
  p.cell_size =
      DeclareAndGet<double>(node, "ground_segmentation.cell_size", p.cell_size);
  p.min_range =
      DeclareAndGet<double>(node, "ground_segmentation.min_range", p.min_range);
  p.max_range =
      DeclareAndGet<double>(node, "ground_segmentation.max_range", p.max_range);
  p.ground_height_threshold =
      DeclareAndGet<double>(node, "ground_segmentation.ground_height_threshold",
                            p.ground_height_threshold);
  p.min_points_per_cell = DeclareAndGet<int>(
      node, "ground_segmentation.min_points_per_cell", p.min_points_per_cell);
  p.neighbor_radius = DeclareAndGet<int>(
      node, "ground_segmentation.neighbor_radius", p.neighbor_radius);
  p.min_neighbor_cells = DeclareAndGet<int>(
      node, "ground_segmentation.min_neighbor_cells", p.min_neighbor_cells);
  p.ground_z_quantile = DeclareAndGet<double>(
      node, "ground_segmentation.ground_z_quantile", p.ground_z_quantile);
  p.grid_reserve = DeclareAndGet<int>(node, "ground_segmentation.grid_reserve",
                                      p.grid_reserve);
  p.smoothed_grid_reserve =
      DeclareAndGet<int>(node, "ground_segmentation.smoothed_grid_reserve",
                         p.smoothed_grid_reserve);
  return p;
}

GroundPatchParams GaloFrontendComponent::LoadGroundPatchParams(
    rclcpp::Node& node) {
  GroundPatchParams p;
  p.cell_size =
      DeclareAndGet<double>(node, "ground_patch.cell_size", p.cell_size);
  p.min_points =
      DeclareAndGet<int>(node, "ground_patch.min_points", p.min_points);
  p.max_thickness = DeclareAndGet<double>(node, "ground_patch.max_thickness",
                                          p.max_thickness);
  p.min_normal_z =
      DeclareAndGet<double>(node, "ground_patch.min_normal_z", p.min_normal_z);
  p.max_surface_variation = DeclareAndGet<double>(
      node, "ground_patch.max_surface_variation", p.max_surface_variation);
  p.patch_reserve =
      DeclareAndGet<int>(node, "ground_patch.patch_reserve", p.patch_reserve);
  p.valid_patch_reserve = DeclareAndGet<int>(
      node, "ground_patch.valid_patch_reserve", p.valid_patch_reserve);
  p.marker_normal_scale = DeclareAndGet<double>(
      node, "ground_patch.marker_normal_scale", p.marker_normal_scale);
  p.marker_shaft_diameter = DeclareAndGet<double>(
      node, "ground_patch.marker_shaft_diameter", p.marker_shaft_diameter);
  p.marker_head_diameter = DeclareAndGet<double>(
      node, "ground_patch.marker_head_diameter", p.marker_head_diameter);
  p.marker_head_length = DeclareAndGet<double>(
      node, "ground_patch.marker_head_length", p.marker_head_length);
  p.marker_lifetime = DeclareAndGet<double>(
      node, "ground_patch.marker_lifetime", p.marker_lifetime);
  p.cell_marker_z_offset = DeclareAndGet<double>(
      node, "ground_patch.cell_marker_z_offset", p.cell_marker_z_offset);
  p.cell_marker_height = DeclareAndGet<double>(
      node, "ground_patch.cell_marker_height", p.cell_marker_height);
  p.cell_marker_alpha = DeclareAndGet<double>(
      node, "ground_patch.cell_marker_alpha", p.cell_marker_alpha);
  return p;
}

PlanarRegistrationParams GaloFrontendComponent::LoadPlanarRegistrationParams(
    rclcpp::Node& node) {
  PlanarRegistrationParams p;
  p.voxel_size = DeclareAndGet<double>(node, "planar_registration.voxel_size",
                                       p.voxel_size);
  p.min_points_per_voxel = DeclareAndGet<int>(
      node, "planar_registration.min_points_per_voxel", p.min_points_per_voxel);
  p.grid_reserve = DeclareAndGet<int>(node, "planar_registration.grid_reserve",
                                      p.grid_reserve);
  return p;
}

}  // namespace ground_aware_lidar_odometry

RCLCPP_COMPONENTS_REGISTER_NODE(
    ground_aware_lidar_odometry::GaloFrontendComponent)
