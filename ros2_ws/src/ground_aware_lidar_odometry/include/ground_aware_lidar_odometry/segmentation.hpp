#pragma once
#include <math.h>

#include <algorithm>
#include <limits>
#include <unordered_map>
#include <utility>

#include "ground_aware_lidar_odometry/utils.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"

struct GroundSegmentationParams {
  double cell_size = 1.0;
  double min_range = 2.0;
  double max_range = 80.0;
  double ground_height_threshold = 0.15;
  int min_points_per_cell = 5;
};
struct CellKey {
  int x;
  int y;
  bool operator==(const CellKey& other) const {
    return x == other.x && y == other.y;
  }
};
struct GridCell {
  int count = 0;
  double min_z = std::numeric_limits<double>::infinity();
};

struct CellKeyHash {
  std::size_t operator()(const CellKey& k) const {
    // Combine hashes of members using XOR and bit-shifting
    return std::hash<int>()(k.x) ^ (std::hash<int>()(k.y) << 1);
  }
};

enum PointLabels { GROUND, NON_GROUND, UNKNOWN };

struct SegmentationResult {
  std::vector<PointLabels> labels;
  CloudMsg msg;
};

class Segmentation {
 public:
  Segmentation(const GroundSegmentationParams& params) : params_(params) {
    grid_.reserve(50000);  // tweak later
    smoothed_ground_z_.reserve(50000);
  }

  SegmentationResult Classify(const CloudMsg& msg);

  sensor_msgs::msg::PointCloud2 MakeColoredCloud(
      const SegmentationResult& result) const;

 private:
  GroundSegmentationParams params_;
  std::unordered_map<CellKey, GridCell, CellKeyHash> grid_;
  std::unordered_map<CellKey, double, CellKeyHash> smoothed_ground_z_;

  std::pair<int, int> GetIndexes(float x, float y) const;

  void FillGrid(const CloudMsg& msg);

  void FillSmoothedGrid();

  double GetNeighborGroundZ(const CellKey& key) const;
};