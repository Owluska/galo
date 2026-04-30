#pragma once
#include <math.h>

#include <Eigen/Dense>
#include <algorithm>
#include <limits>
#include <unordered_map>
#include <utility>

#include "ground_aware_lidar_odometry/utils.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
enum PointLabels { GROUND, NON_GROUND, UNKNOWN };

struct CellKey {
  int x;
  int y;
  bool operator==(const CellKey& other) const {
    return x == other.x && y == other.y;
  }
};
struct CellKeyHash {
  std::size_t operator()(const CellKey& k) const {
    // Combine hashes of members using XOR and bit-shifting
    return std::hash<int>()(k.x) ^ (std::hash<int>()(k.y) << 1);
  }
};

struct GroundSegmentationParams {
  double cell_size = 1.0;
  double min_range = 2.0;
  double max_range = 80.0;
  double ground_height_threshold = 0.20;
  int min_points_per_cell = 5;
};
struct GroundPatchParams {
  double cell_size = 2.5;
  int min_points = 15;
  double max_thickness = 0.20;
  double min_normal_z = 0.85;  // ~30° slope
  double min_planarity = 0.05;
};
struct GroundRegistrationParams {
  double max_match_distance = 5.0;
  double min_normal_dot = 0.85;
  int max_iterations = 5;
  int min_matches = 2;
  double max_dz = 0.1;     // meters
  double max_roll = 0.02;  // ~1°
  double max_pitch = 0.02;
  double damping_z = 1e-3;
  double damping_roll = 1e-2;
  double damping_pitch = 1e-2;
  double min_x_span_for_pitch = 8.0;
  double min_y_span_for_roll = 6.0;
  double imu_roll_weight = 100.0;
  double imu_pitch_weight = 100.0;
  bool use_imu_prior = true;
  int log_throttle = 2000;  // ms
};

struct PlanarRegistrationParams {
  double voxel_size = 0.5;  // m
  double max_match_distance = 1.0;
  int min_points_per_voxel = 3;
  int max_iterations = 10;
  int min_matches = 20;
  double damping = 1e-4;

  double max_dx = 1.0;
  double max_dy = 1.0;
  double max_dyaw = 0.2;

  double max_dx_step = 0.2;
  double max_dy_step = 0.2;
  double max_dyaw_step = 0.05;

  double convergence_eps = 1e-5;
};

struct GridCell {
  int count = 0;
  double min_z = std::numeric_limits<double>::infinity();
};

struct PatchCell {
  int count = 0;
  Eigen::Vector3d sum = Eigen::Vector3d::Zero();
  Eigen::Matrix3d sum_outer = Eigen::Matrix3d::Zero();

  void AddPoint(const Eigen::Vector3d& p) {
    ++count;
    sum += p;
    sum_outer += p * p.transpose();
  }
};

struct SegmentationResult {
  std::vector<PointLabels> labels;
  CloudMsg msg;
};

struct GroundRegistrationResult {
  Eigen::Matrix3d R = Eigen::Matrix3d::Identity();
  Eigen::Vector3d t = Eigen::Vector3d::Zero();
  int num_matches = 0;
  double mean_abs_residual = 0.0;
  bool valid = false;
};

struct PlanarRegistrationResult {
  Eigen::Matrix2d R = Eigen::Matrix2d::Identity();
  Eigen::Vector2d t = Eigen::Vector2d::Zero();
  int matches = 0;
  double mean_residual = 0.0;
  bool valid = false;
  ;
};

struct GroundPatch {
  Eigen::Vector3d centroid;
  Eigen::Vector3d normal;
  Eigen::Matrix3d covariance;
  double planarity = 0.0;
  double weight = 1.0;
  int support = 0;
  CellKey key;
};

struct Voxel2D {
  int count = 0;
  Eigen::Vector2d sum = Eigen::Vector2d::Zero();
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

class GroundPatchExtractor {
 public:
  explicit GroundPatchExtractor(const GroundPatchParams& params);

  std::vector<GroundPatch> Extract(const CloudMsg& cloud,
                                   const std::vector<PointLabels>& labels);
  visualization_msgs::msg::MarkerArray MakeGroundPatchMarkers(
      const std_msgs::msg::Header& header, double normal_scale = 0.7) const;

 private:
  GroundPatchParams params_;
  std::unordered_map<CellKey, PatchCell, CellKeyHash> patches_;
  std::vector<GroundPatch> valid_patches_;

  std::pair<int, int> GetIndexes(float x, float y) const;
};

class GroundRegistration {
 public:
  explicit GroundRegistration(const GroundRegistrationParams& params,
                              const rclcpp::Logger& logger,
                              const rclcpp::Clock& clock)
      : params_(params), logger_(logger), clock_(clock) {}
  GroundRegistrationResult Align(const std::vector<GroundPatch>& map,
                                 const std::vector<GroundPatch>& current,
                                 const Eigen::Matrix3d& R_imu_delta) const;

 private:
  GroundRegistrationParams params_;
  rclcpp::Logger logger_;
  rclcpp::Clock clock_;
  static Eigen::Matrix3d Skew(const Eigen::Vector3d& v);

  static Eigen::Matrix3d ExpSO3(const Eigen::Vector3d& w);

  static Eigen::Vector3d LogSO3(const Eigen::Matrix3d& R);
  int FindNearestPatch(const Eigen::Vector3d& p, const Eigen::Vector3d& normal,
                       const std::vector<GroundPatch>& map) const;
};

class PlanarRegistration {
 public:
  PlanarRegistration(const PlanarRegistrationParams params,
                     const rclcpp::Logger& logger, const rclcpp::Clock& clock)
      : params_(params), logger_(logger), clock_(clock) {}

  PlanarRegistrationResult Align(const std::vector<Eigen::Vector2d>& map,
                                 const std::vector<Eigen::Vector2d>& current);
  std::vector<Eigen::Vector2d> ExtractPoints(
      const CloudMsg& cloud, const std::vector<PointLabels>& labels) const;

  std::vector<Eigen::Vector2d> Filter(
      const std::vector<Eigen::Vector2d>& inp) const;

 private:
  PlanarRegistrationParams params_;
  rclcpp::Logger logger_;
  rclcpp::Clock clock_;
};