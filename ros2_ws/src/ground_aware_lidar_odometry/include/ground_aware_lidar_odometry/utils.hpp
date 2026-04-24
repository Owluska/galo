#pragma once
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"

using CloudMsg = sensor_msgs::msg::PointCloud2;

bool IsFinitePoint(float x, float y, float z = .0f);