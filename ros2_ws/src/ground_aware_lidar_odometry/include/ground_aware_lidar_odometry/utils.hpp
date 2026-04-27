#pragma once
#include <chrono>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"

using CloudMsg = sensor_msgs::msg::PointCloud2;
using time_pt = std::chrono::time_point<std::chrono::steady_clock>;

struct TimeMeasurments_t {
  std::string label;
  time_pt start;
  time_pt end;

  TimeMeasurments_t(const std::string& l)
      : label(l), start(std::chrono::steady_clock::now()) {}

  void SetEnd() { end = std::chrono::steady_clock::now(); }
};

bool IsFinitePoint(float x, float y, float z = .0f);

float GetDelayMs(const time_pt& start, const time_pt& end);
