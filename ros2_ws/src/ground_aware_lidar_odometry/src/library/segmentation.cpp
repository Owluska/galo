#include "ground_aware_lidar_odometry/segmentation.hpp"

#include <omp.h>

#include <chrono>
#include <iterator>
#include <random>

std::pair<int, int> Segmentation::GetIndexes(float x, float y) const {
  int ix = static_cast<int>(std::floor(x / params_.ground.cell_size));
  int iy = static_cast<int>(std::floor(y / params_.ground.cell_size));
  return std::make_pair(ix, iy);
}

void Segmentation::FillGrid(const CloudMsg& msg) {
  size_t n = static_cast<size_t>(msg.width);
  n *= static_cast<size_t>(msg.height);
  sensor_msgs::PointCloud2ConstIterator<float> x_it(msg, "x");
  sensor_msgs::PointCloud2ConstIterator<float> y_it(msg, "y");
  sensor_msgs::PointCloud2ConstIterator<float> z_it(msg, "z");
  grid_.clear();
  for (size_t idx = 0; idx < n; ++x_it, ++y_it, ++z_it, ++idx) {
    float x = *x_it;
    float y = *y_it;
    double z = static_cast<double>(*z_it);
    if (!IsFinitePoint(x, y, z)) continue;
    double r2 = x * x + y * y;
    if (r2 < params_.common.min_range * params_.common.min_range ||
        r2 > params_.common.max_range * params_.common.max_range) {
      continue;
    }
    auto [ix, iy] = GetIndexes(x, y);

    CellKey key{ix, iy};

    auto& cell = grid_[key];
    cell.count++;
    cell.zs.push_back(z);
  }
  for (auto& [key, cell] : grid_) {
    cell.CellGroundZ(params_.ground.ground_z_quantile);
  }
}

void Segmentation::FillSmoothedGrid() {
  smoothed_ground_z_.clear();
  for (const auto& [key, cell] : grid_) {
    smoothed_ground_z_[key] = GetNeighborGroundZ(key);
  }
}

double Segmentation::GetNeighborGroundZ(const CellKey& key) const {
  if (params_.ground.neighbor_radius == 0) {
    return std::numeric_limits<double>::quiet_NaN();
  }
  std::vector<double> zs;
  const int radius = params_.ground.neighbor_radius;
  const int side = 2 * radius + 1;
  zs.reserve(side * side);

  auto center_it = grid_.find(key);
  if (center_it != grid_.end() && !std::isnan(center_it->second.min_z)) {
    zs.emplace_back(center_it->second.min_z);
  }
  for (int dx = -radius; dx <= radius; ++dx) {
    for (int dy = -radius; dy <= radius; ++dy) {
      if (dx == 0 && dy == 0) continue;
      CellKey nk{key.x + dx, key.y + dy};
      auto it = grid_.find(nk);
      if (it == grid_.end() || std::isnan(it->second.min_z)) continue;

      const auto& cell = it->second;

      zs.emplace_back(cell.min_z);
    }
  }

  if (zs.size() < static_cast<size_t>(params_.ground.min_neighbor_cells)) {
    return std::numeric_limits<double>::quiet_NaN();
  }

  const size_t median_idx = zs.size() / 2;
  std::nth_element(zs.begin(),
                   zs.begin() + static_cast<std::ptrdiff_t>(median_idx),
                   zs.end());
  return zs[median_idx];  // median
}

SegmentationResult Segmentation::SegmentGround(const CloudMsg& msg) {
  FillGrid(msg);
  const bool smoothing_is_off = params_.ground.neighbor_radius == 0;
  if (!smoothing_is_off) {
    FillSmoothedGrid();
  }

  size_t n = static_cast<size_t>(msg.width);
  n *= static_cast<size_t>(msg.height);
  sensor_msgs::PointCloud2ConstIterator<float> x_it(msg, "x");
  sensor_msgs::PointCloud2ConstIterator<float> y_it(msg, "y");
  sensor_msgs::PointCloud2ConstIterator<float> z_it(msg, "z");

  SegmentationResult result;
  result.labels.assign(n, PointLabels::UNKNOWN);

  for (size_t idx = 0; idx < n; ++x_it, ++y_it, ++z_it, ++idx) {
    float x = *x_it;
    float y = *y_it;
    double z = static_cast<double>(*z_it);
    if (!IsFinitePoint(x, y, z)) continue;
    double r2 = x * x + y * y;
    if (r2 < params_.common.min_range * params_.common.min_range ||
        r2 > params_.common.max_range * params_.common.max_range) {
      continue;
    }
    auto [ix, iy] = GetIndexes(x, y);
    CellKey key{ix, iy};

    double ground_z = std::numeric_limits<double>::quiet_NaN();

    if (!smoothing_is_off) {
      auto gz_it = smoothed_ground_z_.find(key);
      if (gz_it != smoothed_ground_z_.end()) {
        ground_z = gz_it->second;
      }
    }

    if (std::isnan(ground_z)) {
      auto raw_it = grid_.find(key);
      if (raw_it == grid_.end() || std::isnan(raw_it->second.min_z) ||
          raw_it->second.count < params_.ground.min_points_per_cell) {
        continue;
      }

      ground_z = raw_it->second.min_z;
    }

    const double dz = z - ground_z;
    result.labels[idx] = std::abs(dz) <= params_.ground.ground_height_threshold
                             ? PointLabels::GROUND
                             : PointLabels::NON_GROUND;
  }

  return result;
}


sensor_msgs::msg::PointCloud2 Segmentation::MakeColoredCloud(
    const CloudMsg& cloud, const SegmentationResult& result) const {
  sensor_msgs::msg::PointCloud2 out;
  out.header = cloud.header;
  out.height = cloud.height;
  out.width = cloud.width;
  out.is_bigendian = false;
  out.is_dense = false;

  sensor_msgs::PointCloud2Modifier modifier(out);
  modifier.setPointCloud2FieldsByString(2, "xyz", "rgb");
  modifier.resize(static_cast<size_t>(cloud.width) * cloud.height);

  sensor_msgs::PointCloud2ConstIterator<float> x_in(cloud, "x");
  sensor_msgs::PointCloud2ConstIterator<float> y_in(cloud, "y");
  sensor_msgs::PointCloud2ConstIterator<float> z_in(cloud, "z");
  sensor_msgs::PointCloud2Iterator<float> x_out(out, "x");
  sensor_msgs::PointCloud2Iterator<float> y_out(out, "y");
  sensor_msgs::PointCloud2Iterator<float> z_out(out, "z");
  sensor_msgs::PointCloud2Iterator<uint8_t> r_out(out, "r");
  sensor_msgs::PointCloud2Iterator<uint8_t> g_out(out, "g");
  sensor_msgs::PointCloud2Iterator<uint8_t> b_out(out, "b");

  const size_t n = static_cast<size_t>(cloud.width) * cloud.height;
  for (size_t i = 0; i < n; ++i, ++x_in, ++y_in, ++z_in, ++x_out, ++y_out,
              ++z_out, ++r_out, ++g_out, ++b_out) {
    *x_out = *x_in;
    *y_out = *y_in;
    *z_out = *z_in;

    PointLabels label = PointLabels::UNKNOWN;
    if (i < result.labels.size()) label = result.labels[i];

    if (label == PointLabels::GROUND) {
      *r_out = 60;
      *g_out = 180;
      *b_out = 75;
    } else if (label == PointLabels::NON_GROUND) {
      *r_out = 240;
      *g_out = 160;
      *b_out = 60;
    } else {
      *r_out = 110;
      *g_out = 110;
      *b_out = 110;
    }
  }

  return out;
}

GroundPatchExtractor::GroundPatchExtractor(const GroundPatchParams& params)
    : params_(params) {
  patches_.reserve(static_cast<size_t>(params_.patch_reserve));
  valid_patches_.reserve(static_cast<size_t>(params_.valid_patch_reserve));
}

std::pair<int, int> GroundPatchExtractor::GetIndexes(float x, float y) const {
  int ix = static_cast<int>(std::floor(x / params_.cell_size));
  int iy = static_cast<int>(std::floor(y / params_.cell_size));
  return std::make_pair(ix, iy);
}

std::vector<GroundPatch> GroundPatchExtractor::Extract(
    const CloudMsg& cloud, const std::vector<PointLabels>& labels) {
  patches_.clear();
  valid_patches_.clear();

  const size_t n = static_cast<size_t>(cloud.width) * cloud.height;
  if (n == 0 || labels.size() != n) return valid_patches_;

  sensor_msgs::PointCloud2ConstIterator<float> x_it(cloud, "x");
  sensor_msgs::PointCloud2ConstIterator<float> y_it(cloud, "y");
  sensor_msgs::PointCloud2ConstIterator<float> z_it(cloud, "z");

  for (size_t idx = 0; idx < n; ++idx, ++x_it, ++y_it, ++z_it) {
    if (labels[idx] != PointLabels::GROUND) continue;
    const float x = *x_it;
    const float y = *y_it;
    const float z = *z_it;
    if (!IsFinitePoint(x, y, z)) continue;
    auto [ix, iy] = GetIndexes(x, y);
    patches_[{ix, iy}].AddPoint(x, y, z);
  }

  for (const auto& [key, cell] : patches_) {
    if (cell.count < params_.min_points) continue;

    const double inv_count = 1.0 / static_cast<double>(cell.count);
    const Eigen::Vector3d centroid = cell.sum * inv_count;
    Eigen::Matrix3d cov =
        cell.SumOuter() * inv_count - centroid * centroid.transpose();
    cov = 0.5 * (cov + cov.transpose());

    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver(cov);
    if (solver.info() != Eigen::Success) continue;

    Eigen::Vector3d normal = solver.eigenvectors().col(0);
    if (normal.z() < 0.0) normal = -normal;

    const Eigen::Vector3d eigenvalues = solver.eigenvalues().cwiseMax(0.0);
    const double eig_sum = std::max(eigenvalues.sum(), 1e-12);
    const double surface_variation = eigenvalues(0) / eig_sum;
    const double thickness = 2.0 * std::sqrt(std::max(eigenvalues(0), 0.0));

    if (normal.z() < params_.min_normal_z ||
        surface_variation > params_.max_surface_variation ||
        thickness > params_.max_thickness) {
      continue;
    }

    GroundPatch patch;
    patch.centroid = centroid;
    patch.normal = normal;
    patch.covariance = cov;
    patch.surface_variation = surface_variation;
    patch.weight = static_cast<double>(cell.count);
    patch.support = cell.count;
    patch.key = key;
    valid_patches_.push_back(patch);
  }

  return valid_patches_;
}

visualization_msgs::msg::MarkerArray
GroundPatchExtractor::MakeGroundPatchMarkers(
    const std_msgs::msg::Header& header) const {
  visualization_msgs::msg::MarkerArray markers;

  visualization_msgs::msg::Marker clear;
  clear.header = header;
  clear.ns = "ground_patches";
  clear.action = visualization_msgs::msg::Marker::DELETEALL;
  markers.markers.push_back(clear);

  int id = 0;
  for (const auto& patch : valid_patches_) {
    visualization_msgs::msg::Marker marker;
    marker.header = header;
    marker.ns = "ground_patches";
    marker.id = id++;
    marker.type = visualization_msgs::msg::Marker::ARROW;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.scale.x = params_.marker_shaft_diameter;
    marker.scale.y = params_.marker_head_diameter;
    marker.scale.z = params_.marker_head_length;
    marker.color.r = 0.1f;
    marker.color.g = 0.8f;
    marker.color.b = 0.2f;
    marker.color.a = 0.9f;
    marker.lifetime = rclcpp::Duration::from_seconds(params_.marker_lifetime);

    geometry_msgs::msg::Point p0;
    p0.x = patch.centroid.x();
    p0.y = patch.centroid.y();
    p0.z = patch.centroid.z();
    geometry_msgs::msg::Point p1;
    const Eigen::Vector3d end =
        patch.centroid + params_.marker_normal_scale * patch.normal;
    p1.x = end.x();
    p1.y = end.y();
    p1.z = end.z();
    marker.points.push_back(p0);
    marker.points.push_back(p1);
    markers.markers.push_back(marker);
  }

  return markers;
}

Eigen::Matrix3d GroundRegistration::Skew(const Eigen::Vector3d& v) {
  Eigen::Matrix3d S;
  S << 0.0, -v.z(), v.y(), v.z(), 0.0, -v.x(), -v.y(), v.x(), 0.0;
  return S;
}

Eigen::Matrix3d GroundRegistration::ExpSO3(const Eigen::Vector3d& w) {
  const double theta = w.norm();
  const Eigen::Matrix3d W = Skew(w);
  if (theta < 1e-9) return Eigen::Matrix3d::Identity() + W;
  return Eigen::Matrix3d::Identity() + std::sin(theta) / theta * W +
         (1.0 - std::cos(theta)) / (theta * theta) * W * W;
}

Eigen::Vector3d GroundRegistration::LogSO3(const Eigen::Matrix3d& R) {
  const double cos_theta = std::clamp((R.trace() - 1.0) * 0.5, -1.0, 1.0);
  const double theta = std::acos(cos_theta);
  Eigen::Vector3d w;
  w << R(2, 1) - R(1, 2), R(0, 2) - R(2, 0), R(1, 0) - R(0, 1);
  if (theta < 1e-9) return 0.5 * w;
  return theta / (2.0 * std::sin(theta)) * w;
}

int GroundRegistration::FindNearestPatch(
    const Eigen::Vector3d& p, const Eigen::Vector3d& normal,
    const std::vector<GroundPatch>& map) const {
  int best_idx = -1;
  double best_d2 = params_.max_match_distance * params_.max_match_distance;
  for (size_t i = 0; i < map.size(); ++i) {
    if (std::abs(normal.dot(map[i].normal)) < params_.min_normal_dot) continue;
    const double d2 = (p - map[i].centroid).squaredNorm();
    if (d2 < best_d2) {
      best_d2 = d2;
      best_idx = static_cast<int>(i);
    }
  }
  return best_idx;
}

int GroundRegistration::FindNearestPatchKDTree(
    const Eigen::Vector3d& p, const Eigen::Vector3d& normal,
    const std::vector<GroundPatch>& map,
    const pcl::KdTreeFLANN<pcl::PointXYZ>& kdtree) const {
  pcl::PointXYZ query;
  query.x = static_cast<float>(p.x());
  query.y = static_cast<float>(p.y());
  query.z = static_cast<float>(p.z());

  std::vector<int> indices(std::max(1, params_.k_nearest_neighbors));
  std::vector<float> dists2(std::max(1, params_.k_nearest_neighbors));
  const int found = kdtree.nearestKSearch(query, params_.k_nearest_neighbors,
                                          indices, dists2);
  const double max_d2 = params_.max_match_distance * params_.max_match_distance;
  for (int i = 0; i < found; ++i) {
    const int idx = indices[i];
    if (idx < 0 || static_cast<size_t>(idx) >= map.size()) continue;
    if (static_cast<double>(dists2[i]) > max_d2) continue;
    if (std::abs(normal.dot(map[idx].normal)) < params_.min_normal_dot) {
      continue;
    }
    if (std::abs(p.z() - map[idx].centroid.z()) >
        params_.max_match_z_difference) {
      continue;
    }
    return idx;
  }
  return -1;
}

GroundRegistrationResult GroundRegistration::Align(
    const std::vector<GroundPatch>& map,
    const std::vector<GroundPatch>& current, const Eigen::Matrix3d& R_imu_prior,
    const Eigen::Matrix3d& R_initial, const Eigen::Vector3d& t_initial) const {
  GroundRegistrationResult result;
  if (map.empty() || current.empty()) return result;

  pcl::PointCloud<pcl::PointXYZ>::Ptr map_cloud(
      new pcl::PointCloud<pcl::PointXYZ>());
  map_cloud->points.reserve(map.size());
  for (const auto& patch : map) {
    pcl::PointXYZ pt;
    pt.x = static_cast<float>(patch.centroid.x());
    pt.y = static_cast<float>(patch.centroid.y());
    pt.z = static_cast<float>(patch.centroid.z());
    map_cloud->points.push_back(pt);
  }
  map_cloud->width = static_cast<uint32_t>(map_cloud->points.size());
  map_cloud->height = 1;
  map_cloud->is_dense = true;
  pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
  kdtree.setInputCloud(map_cloud);

  Eigen::Matrix3d R = R_initial;
  Eigen::Vector3d t = t_initial;
  int final_matches = 0;
  double final_abs_residual_sum = 0.0;

  for (int iter = 0; iter < params_.max_iterations; ++iter) {
    Eigen::Matrix3d H = Eigen::Matrix3d::Zero();
    Eigen::Vector3d b = Eigen::Vector3d::Zero();
    int matches = 0;
    double abs_residual_sum = 0.0;

    for (const auto& cur : current) {
      const Eigen::Vector3d p = R * cur.centroid + t;
      const Eigen::Vector3d n = R * cur.normal;
      const int idx = FindNearestPatchKDTree(p, n, map, kdtree);
      if (idx < 0) continue;

      const auto& ref = map[static_cast<size_t>(idx)];
      const double r = ref.normal.dot(p - ref.centroid);
      Eigen::Vector3d J_rot =
          ref.normal.transpose() * (-R * Skew(cur.centroid));
      Eigen::Vector3d J;
      J << ref.normal.z(), J_rot.x(), J_rot.y();

      double weight = cur.weight;
      if (!std::isfinite(weight) || weight <= 0.0) weight = 1.0;
      const double range = cur.centroid.head<2>().norm();
      weight *= 1.0 / (1.0 + params_.range_weight_coeff * range * range);

      H += weight * J * J.transpose();
      b += weight * J * r;
      ++matches;
      abs_residual_sum += std::abs(r);
    }

    if (matches < params_.min_matches) {
      RCLCPP_WARN_THROTTLE(
          logger_, clock_, 1000,
          "GroundRegistration::Align: low amount of matches %d / %d", matches,
          params_.min_matches);
      result.valid = false;
      return result;
    }

    if (params_.use_imu_prior) {
      const Eigen::Vector3d rot_err = LogSO3(R_imu_prior.transpose() * R);
      H(1, 1) += params_.imu_roll_weight;
      H(2, 2) += params_.imu_pitch_weight;
      b(1) += params_.imu_roll_weight * rot_err.x();
      b(2) += params_.imu_pitch_weight * rot_err.y();
    }

    H.diagonal().array() += Eigen::Array3d(
        params_.damping_z, params_.damping_roll, params_.damping_pitch);
    Eigen::Vector3d dx = -H.ldlt().solve(b);
    if (!dx.allFinite()) return result;

    dx.x() = std::clamp(dx.x(), -params_.max_dz, params_.max_dz);
    dx.y() = std::clamp(dx.y(), -params_.max_roll, params_.max_roll);
    dx.z() = std::clamp(dx.z(), -params_.max_pitch, params_.max_pitch);

    t.z() += dx.x();
    R = R * ExpSO3(Eigen::Vector3d(dx.y(), dx.z(), 0.0));
    final_matches = matches;
    final_abs_residual_sum = abs_residual_sum;

    if (dx.norm() < params_.convergence_eps) break;
  }

  result.R = R;
  result.t = t;
  result.num_matches = final_matches;
  result.mean_abs_residual =
      final_matches > 0 ? final_abs_residual_sum / final_matches : 0.0;
  result.valid = final_matches >= params_.min_matches;
  return result;
}

std::vector<Eigen::Vector2d> PlanarRegistration::ExtractAllPoints(
    const CloudMsg& cloud) const {
  std::vector<Eigen::Vector2d> points;
  const size_t n = static_cast<size_t>(cloud.width) * cloud.height;
  if (n == 0) return points;
  points.reserve(n);

  sensor_msgs::PointCloud2ConstIterator<float> x_it(cloud, "x");
  sensor_msgs::PointCloud2ConstIterator<float> y_it(cloud, "y");
  for (size_t idx = 0; idx < n; ++idx, ++x_it, ++y_it) {
    const float x = *x_it;
    const float y = *y_it;
    if (!IsFinitePoint(x, y)) continue;
    points.emplace_back(x, y);
  }
  return points;
}

std::vector<Eigen::Vector2d> PlanarRegistration::ExtractPoints(
    const CloudMsg& cloud, const std::vector<PointLabels>& labels) const {
  std::vector<Eigen::Vector2d> points;
  const size_t n = static_cast<size_t>(cloud.width) * cloud.height;
  if (n == 0 || labels.size() != n) return points;
  points.reserve(n);

  sensor_msgs::PointCloud2ConstIterator<float> x_it(cloud, "x");
  sensor_msgs::PointCloud2ConstIterator<float> y_it(cloud, "y");
  for (size_t idx = 0; idx < n; ++idx, ++x_it, ++y_it) {
    if (labels[idx] != PointLabels::NON_GROUND) {
      continue;
    }
    const float x = *x_it;
    const float y = *y_it;
    if (!IsFinitePoint(x, y)) continue;
    points.emplace_back(x, y);
  }
  return points;
}

std::vector<Eigen::Vector2d> PlanarRegistration::ExtractFilteredPoints(
    const CloudMsg& cloud, const std::vector<PointLabels>& labels) const {
  const size_t n = static_cast<size_t>(cloud.width) * cloud.height;
  if (n == 0 || labels.size() != n) return {};

  std::unordered_map<CellKey, Voxel2D, CellKeyHash> grid;
  grid.reserve(static_cast<size_t>(params_.grid_reserve));

  sensor_msgs::PointCloud2ConstIterator<float> x_it(cloud, "x");
  sensor_msgs::PointCloud2ConstIterator<float> y_it(cloud, "y");
  for (size_t idx = 0; idx < n; ++idx, ++x_it, ++y_it) {
    if (labels[idx] != PointLabels::NON_GROUND) {
      continue;
    }
    const float x = *x_it;
    const float y = *y_it;
    if (!IsFinitePoint(x, y)) continue;
    const int ix = static_cast<int>(std::floor(x / params_.voxel_size));
    const int iy = static_cast<int>(std::floor(y / params_.voxel_size));
    grid[{ix, iy}].AddPoint(Eigen::Vector2d(x, y));
  }

  std::vector<Eigen::Vector2d> points;
  points.reserve(grid.size());
  if (!params_.use_cluster_representatives) {
    for (const auto& [key, v] : grid) {
      (void)key;
      if (v.count < params_.min_points_per_voxel) continue;
      points.push_back(v.sum / static_cast<double>(v.count));
    }
    return points;
  }

  struct PlanarCluster {
    int point_count = 0;
    Eigen::Vector2d sum = Eigen::Vector2d::Zero();
    std::vector<Eigen::Vector2d> voxel_centroids;
    void AddVoxel(const Voxel2D& voxel) {
      point_count += voxel.count;
      sum += voxel.sum;
      voxel_centroids.push_back(voxel.sum / static_cast<double>(voxel.count));
    }
  };

  std::vector<CellKey> stack;
  stack.reserve(grid.size());
  const int max_reps = std::max(1, params_.max_representatives_per_cluster);

  for (auto& [start_key, start_voxel] : grid) {
    if (start_voxel.visited ||
        start_voxel.count < params_.min_points_per_voxel) {
      continue;
    }

    PlanarCluster cluster;
    stack.clear();
    stack.push_back(start_key);
    start_voxel.visited = true;

    while (!stack.empty()) {
      const CellKey key = stack.back();
      stack.pop_back();
      auto voxel_it = grid.find(key);
      if (voxel_it == grid.end()) continue;
      cluster.AddVoxel(voxel_it->second);

      for (int dx = -1; dx <= 1; ++dx) {
        for (int dy = -1; dy <= 1; ++dy) {
          if (dx == 0 && dy == 0) continue;
          const CellKey neighbor{key.x + dx, key.y + dy};
          auto neighbor_it = grid.find(neighbor);
          if (neighbor_it == grid.end() || neighbor_it->second.visited ||
              neighbor_it->second.count < params_.min_points_per_voxel) {
            continue;
          }
          neighbor_it->second.visited = true;
          stack.push_back(neighbor);
        }
      }
    }

    if (cluster.point_count <= 0) continue;
    const size_t cluster_begin = points.size();
    const auto append_cluster_point = [&](const Eigen::Vector2d& p) {
      constexpr double min_dist2 = 1e-4;
      for (size_t i = cluster_begin; i < points.size(); ++i) {
        if ((points[i] - p).squaredNorm() < min_dist2) return;
      }
      points.push_back(p);
    };

    const Eigen::Vector2d centroid =
        cluster.sum / static_cast<double>(cluster.point_count);
    append_cluster_point(centroid);

    std::vector<std::pair<double, Eigen::Vector2d>> candidates;
    candidates.reserve(cluster.voxel_centroids.size());
    for (const auto& p : cluster.voxel_centroids) {
      candidates.emplace_back((p - centroid).squaredNorm(), p);
    }
    std::sort(candidates.begin(), candidates.end(),
              [](const auto& a, const auto& b) { return a.first > b.first; });

    for (const auto& [dist2, p] : candidates) {
      (void)dist2;
      if (static_cast<int>(points.size() - cluster_begin) >= max_reps) break;
      append_cluster_point(p);
    }
  }

  return points;
}

std::vector<Eigen::Vector2d> PlanarRegistration::Filter(
    const std::vector<Eigen::Vector2d>& inp) const {
  std::unordered_map<CellKey, Voxel2D, CellKeyHash> grid_;
  grid_.reserve(static_cast<size_t>(params_.grid_reserve));

  for (const auto& pt : inp) {
    int ix = std::floor(pt.x() / params_.voxel_size);
    int iy = std::floor(pt.y() / params_.voxel_size);
    auto& v = grid_[{ix, iy}];
    v.AddPoint(pt);
  }

  std::vector<Eigen::Vector2d> outp;
  outp.reserve(grid_.size());

  for (const auto& [key, v] : grid_) {
    if (v.count < params_.min_points_per_voxel) continue;
    outp.push_back(v.sum / v.count);
  }
  return outp;
}



std::vector<PlanarLine> PlanarRegistration::ExtractLines(
    const std::vector<Eigen::Vector2d>& points, double time) const {
  const auto start_time = std::chrono::steady_clock::now();
  std::vector<PlanarLine> lines;
  if (points.empty()) return lines;

  std::unordered_map<CellKey, Voxel2D, CellKeyHash> grid;
  grid.reserve(static_cast<size_t>(params_.grid_reserve));
  const double voxel_size = std::max(params_.voxel_size, 1e-3);
  for (const auto& p : points) {
    if (!IsFinitePoint(p.x(), p.y())) continue;
    const int ix = static_cast<int>(std::floor(p.x() / voxel_size));
    const int iy = static_cast<int>(std::floor(p.y() / voxel_size));
    grid[{ix, iy}].AddPoint(p);
  }

  const int min_voxel_points = std::max(1, params_.min_points_per_voxel);
  const int min_support = std::max(2, params_.min_line_support);
  const int max_lines = std::max(1, params_.max_lines_per_frame);
  const double min_center_separation =
      std::max(0.5, 0.5 * params_.min_line_length);
  const double min_center_separation2 =
      min_center_separation * min_center_separation;

  std::unordered_map<CellKey, size_t, CellKeyHash> candidate_index;
  candidate_index.reserve(grid.size());
  std::vector<CellKey> candidate_keys;
  std::vector<Eigen::Vector2d> candidates;
  candidate_keys.reserve(grid.size());
  candidates.reserve(grid.size());
  for (const auto& [key, voxel] : grid) {
    if (voxel.count < min_voxel_points) continue;
    candidate_index.emplace(key, candidates.size());
    candidate_keys.push_back(key);
    candidates.push_back(voxel.sum / static_cast<double>(voxel.count));
  }

  if (static_cast<int>(candidates.size()) < min_support) {
    RCLCPP_WARN_THROTTLE(
        logger_, clock_, 1000,
        "PlanarRegistration::ExtractLines: too few component candidates "
        "points=%zu voxels=%zu candidates=%zu min_support=%d",
        points.size(), grid.size(), candidates.size(), min_support);
    return lines;
  }

  const auto build_line = [&](const std::vector<size_t>& support,
                              PlanarLine& line) -> bool {
    if (static_cast<int>(support.size()) < min_support) return false;

    Eigen::Vector2d centroid = Eigen::Vector2d::Zero();
    for (const size_t idx : support) centroid += candidates[idx];
    centroid /= static_cast<double>(support.size());

    Eigen::Matrix2d cov = Eigen::Matrix2d::Zero();
    for (const size_t idx : support) {
      const Eigen::Vector2d d = candidates[idx] - centroid;
      cov += d * d.transpose();
    }
    cov /= static_cast<double>(support.size());
    cov = 0.5 * (cov + cov.transpose());

    Eigen::SelfAdjointEigenSolver<Eigen::Matrix2d> solver(cov);
    if (solver.info() != Eigen::Success) return false;
    const Eigen::Vector2d evals = solver.eigenvalues().cwiseMax(0.0);
    if (evals(1) <= 1e-9) return false;

    const double ratio = evals(1) / std::max(evals(0), 1e-9);
    const double min_ratio =
        static_cast<int>(support.size()) >= std::max(min_support + 4, 10)
            ? 0.5 * params_.min_line_eigen_ratio
            : params_.min_line_eigen_ratio;
    if (ratio < min_ratio) return false;

    Eigen::Vector2d dir = solver.eigenvectors().col(1);
    if (dir.x() < 0.0 || (std::abs(dir.x()) < 1e-9 && dir.y() < 0.0)) {
      dir = -dir;
    }
    dir.normalize();

    double min_proj = std::numeric_limits<double>::infinity();
    double max_proj = -std::numeric_limits<double>::infinity();
    double abs_error_sum = 0.0;
    const Eigen::Vector2d normal(-dir.y(), dir.x());
    for (const size_t idx : support) {
      const Eigen::Vector2d d = candidates[idx] - centroid;
      const double proj = dir.dot(d);
      min_proj = std::min(min_proj, proj);
      max_proj = std::max(max_proj, proj);
      abs_error_sum += std::abs(normal.dot(d));
    }

    const double length = max_proj - min_proj;
    if (length < params_.min_line_length) return false;

    const double mean_error = abs_error_sum / static_cast<double>(support.size());
    if (mean_error > params_.max_line_fit_error) return false;

    line.center = centroid + 0.5 * (min_proj + max_proj) * dir;
    line.direction = dir;
    line.normal = normal;
    line.length = length;
    line.fit_error = mean_error;
    line.support = static_cast<int>(support.size());
    line.time = time;
    return true;
  };

  std::vector<char> visited(candidates.size(), 0);
  std::vector<CellKey> stack;
  std::vector<size_t> component;
  std::vector<PlanarLine> all_lines;
  stack.reserve(candidates.size());
  component.reserve(candidates.size());

  for (size_t start_idx = 0; start_idx < candidate_keys.size(); ++start_idx) {
    if (visited[start_idx]) continue;

    stack.clear();
    component.clear();
    stack.push_back(candidate_keys[start_idx]);
    visited[start_idx] = 1;

    while (!stack.empty()) {
      const CellKey key = stack.back();
      stack.pop_back();
      const auto idx_it = candidate_index.find(key);
      if (idx_it == candidate_index.end()) continue;
      component.push_back(idx_it->second);

      for (int dx = -1; dx <= 1; ++dx) {
        for (int dy = -1; dy <= 1; ++dy) {
          if (dx == 0 && dy == 0) continue;
          const CellKey neighbor{key.x + dx, key.y + dy};
          const auto neighbor_it = candidate_index.find(neighbor);
          if (neighbor_it == candidate_index.end()) continue;
          const size_t neighbor_idx = neighbor_it->second;
          if (visited[neighbor_idx]) continue;
          visited[neighbor_idx] = 1;
          stack.push_back(neighbor);
        }
      }
    }

    PlanarLine line;
    if (build_line(component, line)) {
      all_lines.push_back(line);
    }
  }

  std::sort(all_lines.begin(), all_lines.end(), [](const PlanarLine& a,
                                                   const PlanarLine& b) {
    const double score_a = static_cast<double>(a.support) *
                           std::max(a.length, 0.0) /
                           std::max(a.fit_error, 0.05);
    const double score_b = static_cast<double>(b.support) *
                           std::max(b.length, 0.0) /
                           std::max(b.fit_error, 0.05);
    return score_a > score_b;
  });

  for (const auto& line : all_lines) {
    bool duplicate = false;
    for (const auto& existing : lines) {
      const double angle_cos = std::abs(existing.direction.dot(line.direction));
      if (angle_cos > 0.98 &&
          (existing.center - line.center).squaredNorm() <
              min_center_separation2) {
        duplicate = true;
        break;
      }
    }
    if (!duplicate) lines.push_back(line);
    if (static_cast<int>(lines.size()) >= max_lines) break;
  }

  if (lines.empty()) {
    constexpr size_t kMaxFallbackCandidates = 250;
    std::vector<size_t> fallback_active;
    fallback_active.reserve(std::min(candidates.size(), kMaxFallbackCandidates));
    const double step = candidates.size() > kMaxFallbackCandidates
                            ? static_cast<double>(candidates.size()) /
                                  static_cast<double>(kMaxFallbackCandidates)
                            : 1.0;
    double pos = 0.0;
    for (size_t i = 0; i < std::min(candidates.size(), kMaxFallbackCandidates);
         ++i, pos += step) {
      fallback_active.push_back(static_cast<size_t>(pos));
    }

    std::mt19937 rng(17);
    const int fallback_iterations =
        std::max(10, std::min(params_.line_ransac_iterations, 40));
    const int fallback_max_lines = std::min(max_lines, 6);
    const double inlier_dist = std::max(0.05, params_.max_line_fit_error);
    const double inlier_dist2 = inlier_dist * inlier_dist;
    std::vector<size_t> best_inliers;
    std::vector<size_t> inliers;
    best_inliers.reserve(fallback_active.size());
    inliers.reserve(fallback_active.size());

    while (static_cast<int>(lines.size()) < fallback_max_lines &&
           static_cast<int>(fallback_active.size()) >= min_support) {
      best_inliers.clear();
      std::uniform_int_distribution<size_t> dist(0, fallback_active.size() - 1);
      for (int iter = 0; iter < fallback_iterations; ++iter) {
        const size_t ai = dist(rng);
        size_t bi = dist(rng);
        if (ai == bi) continue;
        const Eigen::Vector2d a = candidates[fallback_active[ai]];
        const Eigen::Vector2d b = candidates[fallback_active[bi]];
        Eigen::Vector2d dir = b - a;
        const double norm = dir.norm();
        if (norm < params_.min_line_length) continue;
        dir /= norm;
        const Eigen::Vector2d normal(-dir.y(), dir.x());

        inliers.clear();
        double min_proj = std::numeric_limits<double>::infinity();
        double max_proj = -std::numeric_limits<double>::infinity();
        for (const size_t idx : fallback_active) {
          const Eigen::Vector2d delta = candidates[idx] - a;
          const double lateral = normal.dot(delta);
          if (lateral * lateral <= inlier_dist2) {
            inliers.push_back(idx);
            const double proj = dir.dot(delta);
            min_proj = std::min(min_proj, proj);
            max_proj = std::max(max_proj, proj);
          }
        }
        const double span = max_proj - min_proj;
        if (span < params_.min_line_length) continue;
        if (inliers.size() > best_inliers.size()) best_inliers = inliers;
      }

      PlanarLine line;
      if (!build_line(best_inliers, line)) break;
      lines.push_back(line);

      std::vector<size_t> remaining;
      remaining.reserve(fallback_active.size());
      const Eigen::Vector2d n = line.normal.normalized();
      const Eigen::Vector2d d = line.direction.normalized();
      const double half_length = 0.5 * line.length + voxel_size;
      for (const size_t idx : fallback_active) {
        const Eigen::Vector2d delta = candidates[idx] - line.center;
        if (std::abs(n.dot(delta)) <= inlier_dist &&
            std::abs(d.dot(delta)) <= half_length) {
          continue;
        }
        remaining.push_back(idx);
      }
      if (remaining.size() == fallback_active.size()) break;
      fallback_active.swap(remaining);
    }
  }

  if (lines.empty()) {
    RCLCPP_WARN_THROTTLE(
        logger_, clock_, 1000,
        "PlanarRegistration::ExtractLines: component and fallback fit rejected "
        "all candidates points=%zu voxels=%zu candidates=%zu min_support=%d "
        "min_length=%.2f max_error=%.2f eigen_ratio=%.2f",
        points.size(), grid.size(), candidates.size(), min_support,
        params_.min_line_length, params_.max_line_fit_error,
        params_.min_line_eigen_ratio);
  }

  const auto elapsed_ms =
      std::chrono::duration_cast<std::chrono::milliseconds>(
          std::chrono::steady_clock::now() - start_time)
          .count();
  if (elapsed_ms > 200) {
    RCLCPP_WARN_THROTTLE(
        logger_, clock_, 1000,
        "PlanarRegistration::ExtractLines slow: %ld ms points=%zu "
        "voxels=%zu candidates=%zu components=%zu lines=%zu",
        elapsed_ms, points.size(), grid.size(), candidates.size(),
        all_lines.size(), lines.size());
  }
  return lines;
}

PlanarRegistrationResult PlanarRegistration::AlignLines(
    const std::vector<PlanarLine>& map, const std::vector<PlanarLine>& current,
    const Eigen::Matrix2d& R_initial, const Eigen::Vector2d& t_initial) {
  PlanarRegistrationResult result;
  result.R = R_initial;
  result.t = t_initial;
  last_line_debug_.clear();

  if (map.empty() || current.empty()) {
    RCLCPP_WARN_THROTTLE(
        logger_, clock_, 1000,
        "PlanarRegistration::AlignLines: either map or current lines are empty: "
        "current=%zu, map=%zu",
        current.size(), map.size());
    return result;
  }

  Eigen::Matrix2d R = R_initial;
  Eigen::Vector2d t = t_initial;
  const double yaw_initial = std::atan2(R_initial(1, 0), R_initial(0, 0));
  const double max_total_dx = std::max(0.1, params_.max_dx_step);
  const double max_total_dy = std::max(0.1, params_.max_dy_step);
  const double max_total_dyaw = std::max(0.02, 3.0 * params_.max_dyaw_step);
  const double max_match_dist = std::max(0.0, params_.max_match_distance);
  const double max_angle = std::clamp(params_.max_line_angle_deg, 0.0, 90.0) *
                           M_PI / 180.0;
  const double orient_w = std::max(0.0, params_.line_orientation_weight);

  struct LineMatch {
    PlanarLine current;
    PlanarLine map;
  };
  std::vector<LineMatch> matches;
  matches.reserve(current.size());

  int final_matches = 0;
  double final_residual_sum = 0.0;

  for (int iter = 0; iter < params_.max_iterations; ++iter) {
    Eigen::Matrix3d H = Eigen::Matrix3d::Zero();
    Eigen::Vector3d b = Eigen::Vector3d::Zero();

    const double translation_prior_weight =
        std::max(0.0, params_.translation_prior_weight);
    if (translation_prior_weight > 0.0) {
      H(0, 0) += translation_prior_weight;
      H(1, 1) += translation_prior_weight;
    }

    matches.clear();
    std::vector<PlanarLineDebug> line_debug_current;
    int num_matches = 0;
    double residual_sum = 0.0;
    double cost_sum = 0.0;

    for (const auto& cur_line : current) {
      if (cur_line.direction.squaredNorm() < 1e-9) continue;
      const Eigen::Vector2d p = R * cur_line.center + t;
      Eigen::Vector2d dir = R * cur_line.direction;
      dir.normalize();

      int best_idx = -1;
      double best_score = std::numeric_limits<double>::infinity();
      double best_angle = 0.0;
      for (size_t i = 0; i < map.size(); ++i) {
        const auto& map_line = map[i];
        if (map_line.direction.squaredNorm() < 1e-9 ||
            map_line.normal.squaredNorm() < 1e-9) {
          continue;
        }
        Eigen::Vector2d map_dir = map_line.direction.normalized();
        double dot = std::clamp(dir.dot(map_dir), -1.0, 1.0);
        double angle = std::acos(std::abs(dot));
        if (angle > max_angle) continue;

        Eigen::Vector2d map_normal = map_line.normal.normalized();
        const Eigen::Vector2d delta = p - map_line.center;
        const double lateral = std::abs(map_normal.dot(delta));
        if (lateral > max_match_dist) continue;

        const double along = std::abs(map_dir.dot(delta));
        const double half_support =
            0.5 * (std::max(cur_line.length, 0.0) +
                   std::max(map_line.length, 0.0));
        const double max_along_gap = half_support + max_match_dist;
        if (along > max_along_gap) continue;

        const double score = lateral + 0.05 * along + angle;
        if (score < best_score) {
          best_score = score;
          best_idx = static_cast<int>(i);
          best_angle = angle;
        }
      }
      if (best_idx < 0) continue;

      PlanarLine map_line = map[static_cast<size_t>(best_idx)];
      map_line.direction.normalize();
      map_line.normal.normalize();
      if (dir.dot(map_line.direction) < 0.0) {
        map_line.direction = -map_line.direction;
      }
      if (map_line.normal.dot(p - map_line.center) < 0.0) {
        map_line.normal = -map_line.normal;
      }

      Eigen::Vector2d d_yaw_local;
      d_yaw_local << -cur_line.center.y(), cur_line.center.x();
      const Eigen::Vector2d d_yaw_map = R * d_yaw_local;

      const double r_pos = map_line.normal.dot(p - map_line.center);
      Eigen::Matrix<double, 1, 3> J_pos;
      J_pos << map_line.normal.x(), map_line.normal.y(),
          map_line.normal.dot(d_yaw_map);

      const double support_w = std::sqrt(
          std::max(1.0, static_cast<double>(std::min(cur_line.support,
                                                     map_line.support))));
      const double length_w = std::sqrt(std::max(0.5,
          std::min(cur_line.length, map_line.length)));
      const double w_pos = std::min(5.0, support_w * length_w);
      H += w_pos * J_pos.transpose() * J_pos;
      b += w_pos * J_pos.transpose() * r_pos;

      double cross = map_line.direction.x() * dir.y() -
                     map_line.direction.y() * dir.x();
      double dot = std::clamp(map_line.direction.dot(dir), -1.0, 1.0);
      const double r_ang = std::atan2(cross, dot);
      if (orient_w > 0.0) {
        Eigen::Matrix<double, 1, 3> J_ang;
        J_ang << 0.0, 0.0, 1.0;
        const double w_ang = orient_w * w_pos;
        H += w_ang * J_ang.transpose() * J_ang;
        b += w_ang * J_ang.transpose() * r_ang;
      }

      matches.push_back({cur_line, map_line});
      line_debug_current.push_back({map_line.center, map_line.direction,
                                    map_line.z, map_line.length,
                                    std::abs(r_pos)});
      ++num_matches;
      residual_sum += std::abs(r_pos) + std::abs(r_ang);
      cost_sum += r_pos * r_pos + orient_w * r_ang * r_ang;
      (void)best_angle;
    }

    if (num_matches < params_.min_matches) {
      RCLCPP_WARN_THROTTLE(
          logger_, clock_, 1000,
          "PlanarRegistration::AlignLines: low amount of line matches %d / %d",
          num_matches, params_.min_matches);
      result.valid = false;
      return result;
    }

    Eigen::Matrix3d H_damped = H;
    H_damped.diagonal().array() += params_.damping;
    Eigen::Vector3d dx = -H_damped.ldlt().solve(b);

    if (!dx.allFinite()) {
      RCLCPP_WARN(logger_,
                  "PlanarRegistration::AlignLines: dx has invalid values %s",
                  VectorToString(dx).c_str());
      result.valid = false;
      return result;
    }

    if (std::abs(dx.x()) > params_.max_dx ||
        std::abs(dx.y()) > params_.max_dy ||
        std::abs(dx.z()) > params_.max_dyaw) {
      RCLCPP_WARN(logger_,
                  "PlanarRegistration::AlignLines: dx has too big values %s",
                  VectorToString(dx).c_str());
      result.valid = false;
      return result;
    }

    dx.x() = std::clamp(dx.x(), -params_.max_dx_step, params_.max_dx_step);
    dx.y() = std::clamp(dx.y(), -params_.max_dy_step, params_.max_dy_step);
    dx.z() = std::clamp(dx.z(), -params_.max_dyaw_step, params_.max_dyaw_step);

    const double dtheta = dx.z();
    const double c = std::cos(dtheta);
    const double s = std::sin(dtheta);
    Eigen::Matrix2d dR;
    dR << c, -s, s, c;

    const Eigen::Matrix2d candidate_R = R * dR;
    const Eigen::Vector2d candidate_t = t + Eigen::Vector2d(dx.x(), dx.y());
    const double candidate_yaw =
        std::atan2(candidate_R(1, 0), candidate_R(0, 0));
    if (std::abs(candidate_t.x() - t_initial.x()) > max_total_dx ||
        std::abs(candidate_t.y() - t_initial.y()) > max_total_dy ||
        std::abs(std::atan2(std::sin(candidate_yaw - yaw_initial),
                            std::cos(candidate_yaw - yaw_initial))) >
            max_total_dyaw) {
      RCLCPP_WARN_THROTTLE(
          logger_, clock_, 1000,
          "PlanarRegistration::AlignLines: cumulative correction too big "
          "dt=[%.3f, %.3f] yaw=%.2f deg limits=[%.3f, %.3f, %.2f deg]",
          candidate_t.x() - t_initial.x(), candidate_t.y() - t_initial.y(),
          std::atan2(std::sin(candidate_yaw - yaw_initial),
                     std::cos(candidate_yaw - yaw_initial)) *
              180.0 / M_PI,
          max_total_dx, max_total_dy, max_total_dyaw * 180.0 / M_PI);
      break;
    }

    const double current_cost =
        num_matches > 0 ? cost_sum / num_matches
                        : std::numeric_limits<double>::infinity();
    double candidate_residual_sum = 0.0;
    double candidate_cost_sum = 0.0;
    for (const auto& match : matches) {
      const Eigen::Vector2d p_new =
          candidate_R * match.current.center + candidate_t;
      Eigen::Vector2d dir_new = candidate_R * match.current.direction;
      dir_new.normalize();
      const double r_pos = match.map.normal.dot(p_new - match.map.center);
      double cross = match.map.direction.x() * dir_new.y() -
                     match.map.direction.y() * dir_new.x();
      double dot = std::clamp(match.map.direction.dot(dir_new), -1.0, 1.0);
      const double r_ang = std::atan2(cross, dot);
      candidate_residual_sum += std::abs(r_pos) + std::abs(r_ang);
      candidate_cost_sum += r_pos * r_pos + orient_w * r_ang * r_ang;
    }
    const double candidate_cost =
        num_matches > 0 ? candidate_cost_sum / num_matches
                        : std::numeric_limits<double>::infinity();

    const double rel_tol = std::max(0.0, params_.early_stop_worsen_rel_tol);
    const double abs_tol = std::max(0.0, params_.early_stop_worsen_abs_tol);
    const bool worsened =
        candidate_cost > current_cost * (1.0 + rel_tol) + abs_tol;

    if (worsened) {
      final_matches = num_matches;
      final_residual_sum = residual_sum;
      last_line_debug_ = line_debug_current;
      RCLCPP_DEBUG(
          logger_,
          "PlanarRegistration::AlignLines: early stop at iter %d, cost %.6f -> %.6f",
          iter, current_cost, candidate_cost);
      break;
    }

    R = candidate_R;
    t = candidate_t;
    final_matches = num_matches;
    final_residual_sum = candidate_residual_sum;
    last_line_debug_ = line_debug_current;

    if (dx.norm() < params_.convergence_eps) break;
  }

  result.R = R;
  result.t = t;
  result.mean_residual =
      final_matches > 0 ? final_residual_sum / final_matches : 0.0;
  result.matches = final_matches;
  result.valid = final_matches >= params_.min_matches;
  return result;
}

visualization_msgs::msg::MarkerArray PlanarRegistration::MakeLineMarkers(
    const std_msgs::msg::Header& header, const std::string& frame_id) const {
  visualization_msgs::msg::MarkerArray markers;

  visualization_msgs::msg::Marker clear;
  clear.header = header;
  clear.header.frame_id = frame_id;
  clear.ns = "planar_line_registration";
  clear.id = 0;
  clear.action = visualization_msgs::msg::Marker::DELETEALL;
  markers.markers.push_back(clear);

  visualization_msgs::msg::Marker lines;
  lines.header = clear.header;
  lines.ns = "planar_line_registration";
  lines.id = 1;
  lines.type = visualization_msgs::msg::Marker::LINE_LIST;
  lines.action = visualization_msgs::msg::Marker::ADD;
  lines.pose.orientation.w = 1.0;
  lines.scale.x = 0.20;
  lines.color.r = 0.1f;
  lines.color.g = 0.7f;
  lines.color.b = 1.0f;
  lines.color.a = 0.9f;
  lines.lifetime = rclcpp::Duration::from_seconds(1.0);

  constexpr size_t kMaxLines = 250;
  const size_t step = last_line_debug_.size() > kMaxLines
                          ? last_line_debug_.size() / kMaxLines + 1
                          : 1;
  for (size_t i = 0; i < last_line_debug_.size(); i += step) {
    const auto& line = last_line_debug_[i];
    Eigen::Vector2d dir = line.direction;
    if (dir.squaredNorm() < 1e-9) continue;
    dir.normalize();
    const double line_length = std::clamp(line.length, 0.5, 8.0);
    const Eigen::Vector2d a = line.center - 0.5 * line_length * dir;
    const Eigen::Vector2d b = line.center + 0.5 * line_length * dir;

    geometry_msgs::msg::Point pa;
    pa.x = a.x();
    pa.y = a.y();
    pa.z = line.z;
    geometry_msgs::msg::Point pb;
    pb.x = b.x();
    pb.y = b.y();
    pb.z = line.z;
    lines.points.push_back(pa);
    lines.points.push_back(pb);
  }

  markers.markers.push_back(lines);
  return markers;
}

PlanarRegistrationResult PlanarRegistration::Align(
    const std::vector<Eigen::Vector2d>& map,
    const std::vector<Eigen::Vector2d>& current,
    const Eigen::Matrix2d& R_initial, const Eigen::Vector2d& t_initial) {
  PlanarRegistrationResult result;
  result.R = R_initial;
  result.t = t_initial;
  last_line_debug_.clear();

  if (map.empty() || current.empty()) {
    RCLCPP_WARN_THROTTLE(
        logger_, clock_, 1000,
        "PlanarRegistration::Align: either map or current points are empty: "
        "current=%zu, map=%zu",
        current.size(), map.size());
    return result;
  }

  pcl::PointCloud<pcl::PointXYZ>::Ptr map_cloud(
      new pcl::PointCloud<pcl::PointXYZ>());
  map_cloud->points.reserve(map.size());
  for (const auto& p : map) {
    pcl::PointXYZ pt;
    pt.x = static_cast<float>(p.x());
    pt.y = static_cast<float>(p.y());
    pt.z = 0.0f;
    map_cloud->points.push_back(pt);
  }
  map_cloud->width = static_cast<uint32_t>(map_cloud->points.size());
  map_cloud->height = 1;
  map_cloud->is_dense = true;

  pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
  kdtree.setInputCloud(map_cloud);

  Eigen::Matrix2d R = R_initial;
  Eigen::Vector2d t = t_initial;

  const double max_match_dist2 =
      params_.max_match_distance * params_.max_match_distance;
  const int line_k = std::max(2, params_.line_k_nearest);
  if (map.size() < static_cast<size_t>(line_k)) {
    RCLCPP_WARN_THROTTLE(
        logger_, clock_, 1000,
        "PlanarRegistration::Align: not enough map points for line fit %zu / %d",
        map.size(), line_k);
    return result;
  }
  const double max_line_var =
      params_.max_line_fit_error * params_.max_line_fit_error;
  const double max_neighbor_dist2 = params_.max_line_neighbor_distance *
                                    params_.max_line_neighbor_distance;
  const double min_line_var =
      params_.min_line_length * params_.min_line_length / 16.0;

  struct PlanarMatch {
    Eigen::Vector2d cur_pt;
    Eigen::Vector2d ref_pt;
    Eigen::Vector2d normal;
    bool point_to_line = false;
  };

  std::vector<PlanarMatch> planar_matches;
  planar_matches.reserve(current.size());

  int final_matches = 0;
  double final_residual_sum = 0.0;

  for (int iter = 0; iter < params_.max_iterations; ++iter) {
    Eigen::Matrix3d H = Eigen::Matrix3d::Zero();
    Eigen::Vector3d b = Eigen::Vector3d::Zero();

    const double translation_prior_weight =
        std::max(0.0, params_.translation_prior_weight);
    if (translation_prior_weight > 0.0) {
      H(0, 0) += translation_prior_weight;
      H(1, 1) += translation_prior_weight;
    }

    planar_matches.clear();
    std::vector<PlanarLineDebug> line_debug_current;
    int num_matches = 0;
    double residual_sum = 0.0;
    double cost_sum = 0.0;

    for (const auto& cur_pt : current) {
      const Eigen::Vector2d p = R * cur_pt + t;

      pcl::PointXYZ query;
      query.x = static_cast<float>(p.x());
      query.y = static_cast<float>(p.y());
      query.z = 0.0f;

      Eigen::Vector2d d_yaw_local;
      d_yaw_local << -cur_pt.y(), cur_pt.x();
      const Eigen::Vector2d d_yaw_map = R * d_yaw_local;

      {
        std::vector<int> indices(static_cast<size_t>(line_k));
        std::vector<float> dists2(static_cast<size_t>(line_k));
        const int found = kdtree.nearestKSearch(query, line_k, indices, dists2);
        if (found < line_k || static_cast<double>(dists2[0]) > max_match_dist2 ||
            static_cast<double>(dists2[static_cast<size_t>(found - 1)]) >
                max_neighbor_dist2) {
          continue;
        }

        Eigen::Vector2d centroid = Eigen::Vector2d::Zero();
        for (int i = 0; i < found; ++i) {
          centroid += map[static_cast<size_t>(indices[static_cast<size_t>(i)])];
        }
        centroid /= static_cast<double>(found);

        Eigen::Matrix2d cov = Eigen::Matrix2d::Zero();
        for (int i = 0; i < found; ++i) {
          const Eigen::Vector2d d =
              map[static_cast<size_t>(indices[static_cast<size_t>(i)])] -
              centroid;
          cov += d * d.transpose();
        }
        cov /= static_cast<double>(found);
        cov = 0.5 * (cov + cov.transpose());

        Eigen::SelfAdjointEigenSolver<Eigen::Matrix2d> solver(cov);
        if (solver.info() != Eigen::Success) continue;
        const Eigen::Vector2d evals = solver.eigenvalues().cwiseMax(0.0);
        if (evals(1) <= 1e-9 || evals(1) < min_line_var ||
            evals(0) > max_line_var) {
          continue;
        }
        const double ratio = evals(1) / std::max(evals(0), 1e-9);
        if (ratio < params_.min_line_eigen_ratio) continue;
        const double line_length = std::max(params_.min_line_length,
                                            4.0 * std::sqrt(evals(1)));

        Eigen::Vector2d normal = solver.eigenvectors().col(0);
        if (normal.dot(p - centroid) < 0.0) normal = -normal;

        const double r = normal.dot(p - centroid);
        Eigen::Matrix<double, 1, 3> J;
        J << normal.x(), normal.y(), normal.dot(d_yaw_map);

        H += J.transpose() * J;
        b += J.transpose() * r;
        planar_matches.push_back({cur_pt, centroid, normal, true});
        line_debug_current.push_back({centroid,
                                      Eigen::Vector2d(-normal.y(), normal.x()),
                                      0.0, line_length, std::abs(r)});
        ++num_matches;
        residual_sum += std::abs(r);
        cost_sum += r * r;
      }
    }

    if (num_matches < params_.min_matches) {
      RCLCPP_WARN_THROTTLE(
          logger_, clock_, 1000,
          "PlanarRegistration::Align: low amount of %s matches %d / %d",
          "line", num_matches, params_.min_matches);
      result.valid = false;
      return result;
    }

    Eigen::Matrix3d H_damped = H;
    H_damped.diagonal().array() += params_.damping;
    Eigen::Vector3d dx = -H_damped.ldlt().solve(b);

    if (!dx.allFinite()) {
      RCLCPP_WARN(logger_,
                  "PlanarRegistration::Align: dx has invalid values %s",
                  VectorToString(dx).c_str());
      result.valid = false;
      return result;
    }

    if (std::abs(dx.x()) > params_.max_dx ||
        std::abs(dx.y()) > params_.max_dy ||
        std::abs(dx.z()) > params_.max_dyaw) {
      RCLCPP_WARN(logger_,
                  "PlanarRegistration::Align: dx has too big values %s",
                  VectorToString(dx).c_str());
      result.valid = false;
      return result;
    }

    dx.x() = std::clamp(dx.x(), -params_.max_dx_step, params_.max_dx_step);
    dx.y() = std::clamp(dx.y(), -params_.max_dy_step, params_.max_dy_step);
    dx.z() = std::clamp(dx.z(), -params_.max_dyaw_step, params_.max_dyaw_step);

    const double dtheta = dx.z();
    const double c = std::cos(dtheta);
    const double ss = std::sin(dtheta);
    Eigen::Matrix2d dR;
    dR << c, -ss, ss, c;

    const Eigen::Matrix2d candidate_R = R * dR;
    const Eigen::Vector2d candidate_t = t + Eigen::Vector2d(dx.x(), dx.y());

    const double current_cost =
        num_matches > 0 ? cost_sum / num_matches
                        : std::numeric_limits<double>::infinity();
    double candidate_residual_sum = 0.0;
    double candidate_cost_sum = 0.0;
    for (const auto& match : planar_matches) {
      const Eigen::Vector2d p_new = candidate_R * match.cur_pt + candidate_t;
      if (match.point_to_line) {
        const double r = match.normal.dot(p_new - match.ref_pt);
        candidate_residual_sum += std::abs(r);
        candidate_cost_sum += r * r;
      } else {
        const Eigen::Vector2d r = p_new - match.ref_pt;
        candidate_residual_sum += r.norm();
        candidate_cost_sum += r.squaredNorm();
      }
    }
    const double candidate_cost =
        num_matches > 0 ? candidate_cost_sum / num_matches
                        : std::numeric_limits<double>::infinity();

    const double rel_tol = std::max(0.0, params_.early_stop_worsen_rel_tol);
    const double abs_tol = std::max(0.0, params_.early_stop_worsen_abs_tol);
    const bool worsened =
        candidate_cost > current_cost * (1.0 + rel_tol) + abs_tol;

    if (worsened) {
      final_matches = num_matches;
      final_residual_sum = residual_sum;
      last_line_debug_ = line_debug_current;
      RCLCPP_DEBUG(
          logger_,
          "PlanarRegistration::Align: early stop at iter %d, cost %.6f -> %.6f",
          iter, current_cost, candidate_cost);
      break;
    }

    R = candidate_R;
    t = candidate_t;
    final_matches = num_matches;
    final_residual_sum = candidate_residual_sum;
    last_line_debug_ = line_debug_current;

    if (dx.norm() < params_.convergence_eps) break;
  }

  result.R = R;
  result.t = t;
  result.mean_residual =
      final_matches > 0 ? final_residual_sum / final_matches : 0.0;
  result.matches = final_matches;
  result.valid = final_matches >= params_.min_matches;
  return result;
}
