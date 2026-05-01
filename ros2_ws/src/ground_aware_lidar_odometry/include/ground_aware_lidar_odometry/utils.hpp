#pragma once
#include <Eigen/Dense>
#include <chrono>
#include <cmath>
#include <deque>

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

template <typename T>
class FiniteDeque {
 public:
  FiniteDeque() : size_(0) {}
  void Resize(int s) { size_ = static_cast<size_t>(s); }
  void Update(const T& item) {
    queue_.push_back(item);
    while (queue_.size() > size_) {
      queue_.pop_front();
    }
  }
  size_t Size() const { return queue_.size(); }

  T PeerFront() const { return queue_.front(); }
  T PeerBack() const { return queue_.back(); }

  void PopFront() { queue_.pop_front(); }

  const T& operator[](size_t index) const { return queue_[index]; }
  T& operator[](size_t index) { return queue_[index]; }

 private:
  size_t size_;
  std::deque<T> queue_;
};

std::string VectorToString(const Eigen::VectorXd& vec);
