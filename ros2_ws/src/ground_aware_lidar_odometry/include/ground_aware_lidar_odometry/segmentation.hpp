#pragma once
#include <math.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

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

struct SegmentationCommonParams {
  double min_range = 2.0;
  double max_range = 80.0;
};

struct GroundSegmentationParams {
  double cell_size = 1.0;
  double ground_height_threshold = 0.20;
  int min_points_per_cell = 5;
  int neighbor_radius = 1;
  int min_neighbor_cells = 6;
  double ground_z_quantile = 0.20;
  int grid_reserve = 50000;
  int smoothed_grid_reserve = 50000;
};

struct SegmentationParams {
  SegmentationCommonParams common;
  GroundSegmentationParams ground;
};
struct GroundPatchParams {
  double cell_size = 5.0;
  int min_points = 30;
  double max_thickness = 0.2;
  double min_normal_z = 0.90;  // ~30° slope
  double max_surface_variation = 0.03;
  int patch_reserve = 20000;
  int valid_patch_reserve = 5000;
  int num_threads = 0;
  double marker_normal_scale = 0.7;
  double marker_shaft_diameter = 0.08;
  double marker_head_diameter = 0.10;
  double marker_head_length = 0.12;
  double marker_lifetime = 0.2;
  double cell_marker_z_offset = 0.02;
  double cell_marker_height = 0.03;
  double cell_marker_alpha = 0.25;
};
struct GroundRegistrationParams {
  double max_match_distance = 2.0;
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
  double max_match_z_difference = 0.8;
  int k_nearest_neighbors = 8;
  double range_weight_coeff = 0.005;
  double condition_lambda_floor = 1e-9;
  double min_condition_eigenvalue = 1e-4;
  double max_condition_number = 1e6;
  double convergence_eps = 1e-5;
  double early_stop_worsen_rel_tol = 0.01;
  double early_stop_worsen_abs_tol = 1e-9;
};

struct PlanarRegistrationParams {
  double voxel_size = 1.0;  // m
  double max_match_distance = 1.0;
  int min_points_per_voxel = 8;
  bool use_cluster_representatives = true;
  int max_representatives_per_cluster = 5;
  int max_iterations = 15;
  int min_matches = 20;
  double damping = 1e-4;
  int line_k_nearest = 6;
  double max_line_fit_error = 0.5;
  double min_line_eigen_ratio = 6.0;
  double max_line_neighbor_distance = 3.0;
  double min_line_length = 2.0;
  int min_line_support = 4;
  int line_ransac_iterations = 200;
  int max_lines_per_frame = 120;
  double max_line_angle_deg = 25.0;
  double line_orientation_weight = 2.0;
  double translation_prior_weight = 10.0;

  double max_dx = 1.0;
  double max_dy = 1.0;
  double max_dyaw = 0.2;

  double max_dx_step = 0.5;
  double max_dy_step = 0.5;
  double max_dyaw_step = 0.1;

  double convergence_eps = 1e-5;
  double early_stop_worsen_rel_tol = 0.01;
  double early_stop_worsen_abs_tol = 1e-9;
  int grid_reserve = 50000;
};

struct GridCell {
  int count = 0;
  double min_z = std::numeric_limits<double>::infinity();
  // double max_z = -std::numeric_limits<double>::infinity();

  std::vector<double> zs;

  void CellGroundZ(double quantile) {
    if (zs.empty()) {
      min_z = std::numeric_limits<double>::quiet_NaN();
      return;
    }

    quantile = std::clamp(quantile, 0.0, 1.0);
    size_t k = static_cast<size_t>(quantile * static_cast<double>(zs.size()));
    k = std::min(k, zs.size() - 1);
    std::nth_element(zs.begin(), zs.begin() + static_cast<std::ptrdiff_t>(k),
                     zs.end());
    min_z = zs[k];

    // size_t m = static_cast<size_t>(0.80 * static_cast<double>(zs.size()));
    // m = std::min(m, zs.size() - 1);
    // max_z = zs[m];
  }
};

struct PatchCell {
  int count = 0;
  Eigen::Vector3d sum = Eigen::Vector3d::Zero();
  Eigen::Matrix3d sum_outer = Eigen::Matrix3d::Zero();
  void AddPoint(double x, double y, double z) {
    ++count;
    sum.x() += x;
    sum.y() += y;
    sum.z() += z;

    sum_outer(0, 0) += x * x;
    sum_outer(0, 1) += x * y;
    sum_outer(0, 2) += x * z;
    sum_outer(1, 1) += y * y;
    sum_outer(1, 2) += y * z;
    sum_outer(2, 2) += z * z;
  }

  Eigen::Matrix3d SumOuter() const {
    Eigen::Matrix3d out = sum_outer;
    out(1, 0) = out(0, 1);
    out(2, 0) = out(0, 2);
    out(2, 1) = out(1, 2);
    return out;
  }
};

struct SegmentationResult {
  std::vector<PointLabels> labels;
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
};

struct PlanarLine {
  Eigen::Vector2d center = Eigen::Vector2d::Zero();
  Eigen::Vector2d direction = Eigen::Vector2d::UnitX();
  Eigen::Vector2d normal = Eigen::Vector2d::UnitY();
  double z = 0.0;
  double length = 1.0;
  double fit_error = 0.0;
  int support = 0;
  double time = 0.0;
};

struct PlanarLineDebug {
  Eigen::Vector2d center = Eigen::Vector2d::Zero();
  Eigen::Vector2d direction = Eigen::Vector2d::UnitX();
  double z = 0.0;
  double length = 1.0;
  double residual = 0.0;
};

struct GroundPatch {
  Eigen::Vector3d centroid;
  Eigen::Vector3d normal;
  Eigen::Matrix3d covariance;
  double time = 0.0;
  double surface_variation = 0.0;
  double weight = 1.0;
  int support = 0;
  CellKey key;
};

struct Voxel2D {
  int count = 0;
  bool visited = false;
  Eigen::Vector2d sum = Eigen::Vector2d::Zero();
  Eigen::Matrix2d sum_outer = Eigen::Matrix2d::Zero();
  Eigen::Vector2d min_pt = Eigen::Vector2d::Constant(
      std::numeric_limits<double>::infinity());
  Eigen::Vector2d max_pt = Eigen::Vector2d::Constant(
      -std::numeric_limits<double>::infinity());

  void AddPoint(const Eigen::Vector2d& p) {
    ++count;
    sum += p;
    sum_outer += p * p.transpose();
    min_pt = min_pt.cwiseMin(p);
    max_pt = max_pt.cwiseMax(p);
  }
};

class Segmentation {
 public:
  Segmentation(const SegmentationParams& params) : params_(params) {
    grid_.reserve(params_.ground.grid_reserve);
    smoothed_ground_z_.reserve(params_.ground.smoothed_grid_reserve);
  }

  SegmentationResult SegmentGround(const CloudMsg& msg);

  sensor_msgs::msg::PointCloud2 MakeColoredCloud(
      const CloudMsg& cloud, const SegmentationResult& result) const;

 private:
  SegmentationParams params_;
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
      const std_msgs::msg::Header& header) const;

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
                                 const Eigen::Matrix3d& R_imu_prior,
                                 const Eigen::Matrix3d& R_initial,
                                 const Eigen::Vector3d& t_initial) const;

 private:
  GroundRegistrationParams params_;
  rclcpp::Logger logger_;
  mutable rclcpp::Clock clock_;
  static Eigen::Matrix3d Skew(const Eigen::Vector3d& v);

  static Eigen::Matrix3d ExpSO3(const Eigen::Vector3d& w);

  static Eigen::Vector3d LogSO3(const Eigen::Matrix3d& R);
  int FindNearestPatch(const Eigen::Vector3d& p, const Eigen::Vector3d& normal,
                       const std::vector<GroundPatch>& map) const;

  int FindNearestPatchKDTree(
      const Eigen::Vector3d& p, const Eigen::Vector3d& normal,
      const std::vector<GroundPatch>& map,
      const pcl::KdTreeFLANN<pcl::PointXYZ>& kdtree) const;
};

class PlanarRegistration {
 public:
  PlanarRegistration(const PlanarRegistrationParams params,
                     const rclcpp::Logger& logger, const rclcpp::Clock& clock)
      : params_(params), logger_(logger), clock_(clock) {}

  PlanarRegistrationResult Align(const std::vector<Eigen::Vector2d>& map,
                                 const std::vector<Eigen::Vector2d>& current,
                                 const Eigen::Matrix2d& R_initial,
                                 const Eigen::Vector2d& t_initial);

  PlanarRegistrationResult AlignLines(const std::vector<PlanarLine>& map,
                                      const std::vector<PlanarLine>& current,
                                      const Eigen::Matrix2d& R_initial,
                                      const Eigen::Vector2d& t_initial);

  std::vector<Eigen::Vector2d> ExtractAllPoints(const CloudMsg& cloud) const;
  std::vector<Eigen::Vector2d> ExtractPoints(
      const CloudMsg& cloud, const std::vector<PointLabels>& labels) const;
  std::vector<Eigen::Vector2d> ExtractFilteredPoints(
      const CloudMsg& cloud, const std::vector<PointLabels>& labels) const;

  std::vector<Eigen::Vector2d> Filter(
      const std::vector<Eigen::Vector2d>& inp) const;

  std::vector<PlanarLine> ExtractLines(
      const std::vector<Eigen::Vector2d>& points, double time = 0.0) const;

  visualization_msgs::msg::MarkerArray MakeLineMarkers(
      const std_msgs::msg::Header& header, const std::string& frame_id) const;

 private:
  PlanarRegistrationParams params_;
  rclcpp::Logger logger_;
  mutable rclcpp::Clock clock_;
  std::vector<PlanarLineDebug> last_line_debug_;
};
