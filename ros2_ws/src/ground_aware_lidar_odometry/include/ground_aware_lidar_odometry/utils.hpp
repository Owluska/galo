#pragma once
#include <Eigen/Dense>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <deque>
#include <iterator>
#include <optional>

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
  using const_iterator = typename std::deque<T>::const_iterator;
  FiniteDeque() : size_(0) {}
  void Resize(int s) { size_ = static_cast<size_t>(s); }
  void Update(const T& item) {
    queue_.push_back(item);
    while (queue_.size() > size_) {
      queue_.pop_front();
    }
  }

  template <typename Compare>
  void UpdateSorted(const T& item, Compare compare) {
    auto pos = std::upper_bound(queue_.begin(), queue_.end(), item, compare);
    queue_.insert(pos, item);
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

  const_iterator begin() const { return queue_.begin(); }
  const_iterator end() const { return queue_.end(); }

 private:
  size_t size_;
  std::deque<T> queue_;
};

template <typename T, typename TimeGetter>
std::optional<T> FindClosestByTime(const FiniteDeque<T>& queue,
                                   double query_time,
                                   TimeGetter get_time) {
  if (queue.Size() == 0) {
    return std::nullopt;
  }

  auto after_or_equal = std::lower_bound(
      queue.begin(), queue.end(), query_time,
      [get_time](const T& item, double time) { return get_time(item) < time; });

  if (after_or_equal == queue.begin()) {
    return *after_or_equal;
  }
  if (after_or_equal == queue.end()) {
    return queue.PeerBack();
  }

  const auto before = std::prev(after_or_equal);
  if (std::abs(get_time(*before) - query_time) <=
      std::abs(get_time(*after_or_equal) - query_time)) {
    return *before;
  }

  return *after_or_equal;
}

std::string VectorToString(const Eigen::VectorXd& vec);

double NormalizeAngle0To2Pi(double a);

double NormalizeAngle(double a);

std::tuple<double, double, double> EulersFromMatrixSimple(
    const Eigen::Matrix3d& R);

double DegToRad(double deg);