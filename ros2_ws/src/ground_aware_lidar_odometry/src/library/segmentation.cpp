#include "ground_aware_lidar_odometry/segmentation.hpp"

namespace {

void ResetCellValues(
    std::unordered_map<CellKey, GridCell, CellKeyHash>& map) {
  for (auto& [key, cell] : map) {
    cell.count = 0;
    cell.min_z = std::numeric_limits<double>::infinity();
    cell.zs.clear();
  }
}

void ResetCellValues(
    std::unordered_map<CellKey, PatchCell, CellKeyHash>& map) {
  for (auto& [key, cell] : map) {
    cell.count = 0;
    cell.sum.setZero();
    cell.sum_outer.setZero();
  }
}

void ResetCellValues(std::unordered_map<CellKey, Voxel2D, CellKeyHash>& map) {
  for (auto& [key, cell] : map) {
    cell.count = 0;
    cell.sum.setZero();
  }
}

}  // namespace

std::pair<int, int> Segmentation::GetIndexes(float x, float y) const {
  int ix = static_cast<int>(std::floor(x / params_.cell_size));
  int iy = static_cast<int>(std::floor(y / params_.cell_size));
  return std::make_pair(ix, iy);
}

void Segmentation::FillGrid(const CloudMsg& msg) {
  size_t n = static_cast<size_t>(msg.width);
  n *= static_cast<size_t>(msg.height);
  sensor_msgs::PointCloud2ConstIterator<float> x_it(msg, "x");
  sensor_msgs::PointCloud2ConstIterator<float> y_it(msg, "y");
  sensor_msgs::PointCloud2ConstIterator<float> z_it(msg, "z");
  ResetCellValues(grid_);
  for (size_t idx = 0; idx < n; ++x_it, ++y_it, ++z_it, ++idx) {
    float x = *x_it;
    float y = *y_it;
    double z = static_cast<double>(*z_it);
    if (!IsFinitePoint(x, y, z)) continue;
    double r2 = x * x + y * y;
    if (r2 < params_.min_range * params_.min_range ||
        r2 > params_.max_range * params_.max_range) {
      continue;
    }
    auto [ix, iy] = GetIndexes(x, y);

    CellKey key{ix, iy};

    auto& cell = grid_[key];
    cell.count++;
    cell.zs.push_back(z);
  }
  for (auto it = grid_.begin(); it != grid_.end();) {
    if (it->second.count == 0) {
      it = grid_.erase(it);
    } else {
      ++it;
    }
  }
  for (auto& [key, cell] : grid_) {
    cell.CellGroundZ(params_.ground_z_quantile);
  }
}

void Segmentation::FillSmoothedGrid() {
  smoothed_ground_z_.clear();
  for (const auto& [key, cell] : grid_) {
    smoothed_ground_z_[key] = GetNeighborGroundZ(key);
  }
}

double Segmentation::GetNeighborGroundZ(const CellKey& key) const {
  neighbor_ground_zs_.clear();
  const int radius = std::max(1, params_.neighbor_radius);
  const auto min_neighbor_cells =
      static_cast<size_t>(std::max(0, params_.min_neighbor_cells));

  for (int dx = -radius; dx <= radius; ++dx) {
    for (int dy = -radius; dy <= radius; ++dy) {
      CellKey nk{key.x + dx, key.y + dy};

      auto it = grid_.find(nk);
      if (it == grid_.end()) continue;

      const auto& cell = it->second;
      if (cell.count < params_.min_points_per_cell) continue;

      neighbor_ground_zs_.push_back(cell.min_z);
    }
  }

  if (neighbor_ground_zs_.size() < min_neighbor_cells) {
    return std::numeric_limits<double>::quiet_NaN();
  }

  const size_t median_idx = neighbor_ground_zs_.size() / 2;
  std::nth_element(
      neighbor_ground_zs_.begin(),
      neighbor_ground_zs_.begin() + static_cast<std::ptrdiff_t>(median_idx),
      neighbor_ground_zs_.end());
  return neighbor_ground_zs_[median_idx];  // median
}

SegmentationResult Segmentation::Classify(const CloudMsg& msg) {
  SegmentationResult res;
  FillGrid(msg);
  FillSmoothedGrid();
  sensor_msgs::msg::PointCloud2 ground, non_ground;
  size_t n = static_cast<size_t>(msg.width);
  n *= static_cast<size_t>(msg.height);
  sensor_msgs::PointCloud2ConstIterator<float> x_it(msg, "x");
  sensor_msgs::PointCloud2ConstIterator<float> y_it(msg, "y");
  sensor_msgs::PointCloud2ConstIterator<float> z_it(msg, "z");
  res.labels.assign(n, PointLabels::UNKNOWN);
  for (size_t idx = 0; idx < n; ++x_it, ++y_it, ++z_it, ++idx) {
    float x = *x_it;
    float y = *y_it;
    double z = static_cast<double>(*z_it);
    if (!IsFinitePoint(x, y, z)) continue;
    double r2 = x * x + y * y;
    if (r2 < params_.min_range * params_.min_range ||
        r2 > params_.max_range * params_.max_range) {
      continue;
    }
    auto [ix, iy] = GetIndexes(x, y);
    CellKey key{ix, iy};
    auto gz_it = smoothed_ground_z_.find(key);
    if (gz_it == smoothed_ground_z_.end()) {
      continue;
    }
    double ground_z = gz_it->second;
    if (std::isnan(ground_z)) {
      continue;
    }
    double dz = z - ground_z;

    if (std::abs(dz) < params_.ground_height_threshold) {
      res.labels[idx] = PointLabels::GROUND;
    } else {
      res.labels[idx] = PointLabels::NON_GROUND;
    }
  }
  return res;
}

sensor_msgs::msg::PointCloud2 Segmentation::MakeColoredCloud(
    const CloudMsg& cloud, const SegmentationResult& result) const {
  const auto& in = cloud;

  sensor_msgs::msg::PointCloud2 out;
  out.header = in.header;
  out.height = in.height;
  out.width = in.width;
  out.is_bigendian = false;
  out.is_dense = false;

  sensor_msgs::PointCloud2Modifier modifier(out);
  modifier.setPointCloud2FieldsByString(2, "xyz", "rgb");
  modifier.resize(in.width * in.height);

  sensor_msgs::PointCloud2ConstIterator<float> x_in(in, "x");
  sensor_msgs::PointCloud2ConstIterator<float> y_in(in, "y");
  sensor_msgs::PointCloud2ConstIterator<float> z_in(in, "z");

  sensor_msgs::PointCloud2Iterator<float> x_out(out, "x");
  sensor_msgs::PointCloud2Iterator<float> y_out(out, "y");
  sensor_msgs::PointCloud2Iterator<float> z_out(out, "z");
  sensor_msgs::PointCloud2Iterator<uint8_t> r_out(out, "r");
  sensor_msgs::PointCloud2Iterator<uint8_t> g_out(out, "g");
  sensor_msgs::PointCloud2Iterator<uint8_t> b_out(out, "b");

  const size_t n =
      static_cast<size_t>(in.width) * static_cast<size_t>(in.height);

  for (size_t i = 0; i < n; ++i, ++x_in, ++y_in, ++z_in, ++x_out, ++y_out,
              ++z_out, ++r_out, ++g_out, ++b_out) {
    *x_out = *x_in;
    *y_out = *y_in;
    *z_out = *z_in;

    PointLabels label = PointLabels::UNKNOWN;
    if (i < result.labels.size()) {
      label = result.labels[i];
    }

    switch (label) {
      case PointLabels::GROUND:
        *r_out = 0;
        *g_out = 255;
        *b_out = 0;
        break;

      case PointLabels::NON_GROUND:
        *r_out = 255;
        *g_out = 0;
        *b_out = 0;
        break;

      case PointLabels::UNKNOWN:
      default:
        *r_out = 120;
        *g_out = 120;
        *b_out = 120;
        break;
    }
  }

  return out;
}

GroundPatchExtractor::GroundPatchExtractor(const GroundPatchParams& params)
    : params_(params) {
  patches_.reserve(params_.patch_reserve);
  valid_patches_.reserve(params_.valid_patch_reserve);
}

std::pair<int, int> GroundPatchExtractor::GetIndexes(float x, float y) const {
  int ix = static_cast<int>(std::floor(x / params_.cell_size));
  int iy = static_cast<int>(std::floor(y / params_.cell_size));
  return std::make_pair(ix, iy);
}

std::vector<GroundPatch> GroundPatchExtractor::Extract(
    const CloudMsg& cloud, const std::vector<PointLabels>& labels) {
  ResetCellValues(patches_);
  valid_patches_.clear();
  size_t n = static_cast<size_t>(cloud.width);
  n *= static_cast<size_t>(cloud.height);
  if (n == 0 || labels.size() != n) return valid_patches_;
  const double patch_time = rclcpp::Time(cloud.header.stamp).seconds();

  sensor_msgs::PointCloud2ConstIterator<float> x_it(cloud, "x");
  sensor_msgs::PointCloud2ConstIterator<float> y_it(cloud, "y");
  sensor_msgs::PointCloud2ConstIterator<float> z_it(cloud, "z");

  for (size_t idx = 0; idx < n; ++x_it, ++y_it, ++z_it, ++idx) {
    double x = static_cast<double>(*x_it);
    double y = static_cast<double>(*y_it);
    double z = static_cast<double>(*z_it);
    if (!IsFinitePoint(x, y, z)) continue;
    auto label = labels[idx];
    if (label != PointLabels::GROUND) continue;

    auto [ix, iy] = GetIndexes(x, y);
    CellKey key{ix, iy};
    PatchCell& cell = patches_[key];
    cell.AddPoint(Eigen::Vector3d(x, y, z));
  }

  for (auto it = patches_.begin(); it != patches_.end();) {
    if (it->second.count == 0) {
      it = patches_.erase(it);
    } else {
      ++it;
    }
  }

  for (const auto& [key, patch_cell] : patches_) {
    const int N = patch_cell.count;
    if (N < params_.min_points) continue;

    const double inv_N = 1.0 / static_cast<double>(N);

    Eigen::Vector3d mean = patch_cell.sum * inv_N;

    Eigen::Matrix3d cov =
        patch_cell.sum_outer * inv_N - mean * mean.transpose();

    cov = 0.5 * (cov + cov.transpose());  // numerical safety

    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver(cov);
    if (solver.info() != Eigen::Success) continue;

    Eigen::Vector3d eigenvalues = solver.eigenvalues();
    Eigen::Matrix3d eigenvectors = solver.eigenvectors();

    Eigen::Vector3d normal = eigenvectors.col(0);
    if (normal.z() < 0.0) normal = -normal;

    Eigen::Vector3d centroid = mean;

    double lambda0 = std::max(0.0, eigenvalues(0));
    double lambda1 = std::max(0.0, eigenvalues(1));
    double lambda2 = std::max(1e-12, eigenvalues(2));

    double thickness = std::sqrt(lambda0);
    if (thickness > params_.max_thickness) continue;
    if (normal.z() < params_.min_normal_z) continue;

    double sum_lambda = lambda0 + lambda1 + lambda2 + 1e-12;
    double surface_variation = lambda0 / sum_lambda;

    if (surface_variation > params_.max_surface_variation) continue;

    GroundPatch patch;
    patch.centroid = centroid;
    patch.normal = normal;
    patch.covariance = cov;
    patch.time = patch_time;
    patch.support = N;
    patch.weight = static_cast<double>(N);
    patch.surface_variation = surface_variation;
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
  clear.id = 0;
  clear.action = visualization_msgs::msg::Marker::DELETEALL;
  markers.markers.push_back(clear);

  visualization_msgs::msg::Marker clear_cells;
  clear_cells.header = header;
  clear_cells.ns = "ground_patch_cells";
  clear_cells.id = 0;
  clear_cells.action = visualization_msgs::msg::Marker::DELETEALL;
  markers.markers.push_back(clear_cells);
  int id = 1;

  for (const auto& patch : valid_patches_) {
    visualization_msgs::msg::Marker marker;
    marker.header = header;
    marker.ns = "ground_patches";
    marker.id = id++;
    marker.type = visualization_msgs::msg::Marker::ARROW;
    marker.action = visualization_msgs::msg::Marker::ADD;

    geometry_msgs::msg::Point p0;
    p0.x = patch.centroid.x();
    p0.y = patch.centroid.y();
    p0.z = patch.centroid.z();

    geometry_msgs::msg::Point p1;
    p1.x = patch.centroid.x() + params_.marker_normal_scale * patch.normal.x();
    p1.y = patch.centroid.y() + params_.marker_normal_scale * patch.normal.y();
    p1.z = patch.centroid.z() + params_.marker_normal_scale * patch.normal.z();

    marker.points.push_back(p0);
    marker.points.push_back(p1);

    marker.scale.x = params_.marker_shaft_diameter;
    marker.scale.y = params_.marker_head_diameter;
    marker.scale.z = params_.marker_head_length;

    marker.color.r = 0.0f;
    marker.color.g = 0.6f;
    marker.color.b = 1.0f;
    marker.color.a = 1.0f;

    marker.lifetime = rclcpp::Duration::from_seconds(params_.marker_lifetime);

    markers.markers.push_back(marker);

    visualization_msgs::msg::Marker cell_marker;
    cell_marker.header = header;
    cell_marker.ns = "ground_patch_cells";
    cell_marker.id = id++;
    cell_marker.type = visualization_msgs::msg::Marker::CUBE;
    cell_marker.action = visualization_msgs::msg::Marker::ADD;

    cell_marker.pose.position.x = patch.centroid.x();
    cell_marker.pose.position.y = patch.centroid.y();
    cell_marker.pose.position.z =
        patch.centroid.z() + params_.cell_marker_z_offset;

    cell_marker.pose.orientation.w = 1.0;

    cell_marker.scale.x = params_.cell_size;
    cell_marker.scale.y = params_.cell_size;
    cell_marker.scale.z = params_.cell_marker_height;

    cell_marker.color.r = 0.0f;
    cell_marker.color.g = 0.8f;
    cell_marker.color.b = 1.0f;
    cell_marker.color.a = static_cast<float>(params_.cell_marker_alpha);

    cell_marker.lifetime =
        rclcpp::Duration::from_seconds(params_.marker_lifetime);

    markers.markers.push_back(cell_marker);
  }

  return markers;
}

Eigen::Matrix3d GroundRegistration::Skew(const Eigen::Vector3d& v) {
  Eigen::Matrix3d S;
  S << 0.0, -v.z(), v.y(), v.z(), 0.0, -v.x(), -v.y(), v.x(), 0.0;
  return S;
}

Eigen::Matrix3d GroundRegistration::ExpSO3(const Eigen::Vector3d& w) {
  double theta = w.norm();
  if (theta < 1e-12) {
    return Eigen::Matrix3d::Identity() + Skew(w);
  }
  Eigen::Vector3d a = w / theta;
  Eigen::Matrix3d A = Skew(a);

  return Eigen::Matrix3d::Identity() + std::sin(theta) * A +
         (1 - std::cos(theta)) * A * A;
}

int GroundRegistration::FindNearestPatch(
    const Eigen::Vector3d& p, const Eigen::Vector3d& normal,
    const std::vector<GroundPatch>& map) const {
  int best_idx = -1;
  double best_dist2 = params_.max_match_distance * params_.max_match_distance;

  for (size_t i = 0; i < map.size(); ++i) {
    const auto& m = map[i];

    double normal_dot = normal.normalized().dot(m.normal.normalized());
    if (normal_dot < params_.min_normal_dot) {
      continue;
    }
    double dist2 = (p - m.centroid).squaredNorm();
    if (dist2 < best_dist2) {
      best_dist2 = dist2;
      best_idx = static_cast<int>(i);
    }
  }

  return best_idx;
}

Eigen::Vector3d GroundRegistration::LogSO3(const Eigen::Matrix3d& R) {
  double cos_theta = (R.trace() - 1.0) * 0.5;
  cos_theta = std::clamp(cos_theta, -1.0, 1.0);

  double theta = std::acos(cos_theta);

  if (theta < 1e-12) {
    return Eigen::Vector3d::Zero();
  }

  Eigen::Vector3d w;
  w << R(2, 1) - R(1, 2), R(0, 2) - R(2, 0), R(1, 0) - R(0, 1);

  w *= 0.5 * theta / std::sin(theta);
  return w;
}

int GroundRegistration::FindNearestPatchKDTree(
    const Eigen::Vector3d& p, const Eigen::Vector3d& normal,
    const std::vector<GroundPatch>& map,
    const pcl::KdTreeFLANN<pcl::PointXYZ>& kdtree) const {
  if (map.empty()) return -1;

  pcl::PointXYZ query;
  query.x = static_cast<float>(p.x());
  query.y = static_cast<float>(p.y());
  query.z = 0.0f;

  const int K = std::max(params_.k_nearest_neighbors, 1);

  std::vector<int> indices(K);
  std::vector<float> dists2(K);

  int found = kdtree.nearestKSearch(query, K, indices, dists2);
  if (found <= 0) return -1;

  const double max_dist2 =
      params_.max_match_distance * params_.max_match_distance;

  int best_idx = -1;
  double best_dist2 = max_dist2;

  Eigen::Vector3d normal_n = normal.normalized();

  for (int j = 0; j < found; ++j) {
    int idx = indices[j];
    if (idx < 0 || static_cast<size_t>(idx) >= map.size()) continue;

    if (static_cast<double>(dists2[j]) > max_dist2) continue;

    const auto& m = map[idx];

    double normal_dot = normal_n.dot(m.normal.normalized());
    if (normal_dot < params_.min_normal_dot) continue;

    if (static_cast<double>(dists2[j]) < best_dist2) {
      best_dist2 = static_cast<double>(dists2[j]);
      best_idx = idx;
    }
  }

  return best_idx;
}

GroundRegistrationResult GroundRegistration::Align(
    const std::vector<GroundPatch>& map,
    const std::vector<GroundPatch>& current, const Eigen::Matrix3d& R_imu_prior,
    const Eigen::Matrix3d& R_initial, const Eigen::Vector3d& t_initial) const {
  GroundRegistrationResult result;

  if (current.empty() || map.empty()) {
    RCLCPP_WARN(logger_,
                "GroundRegistration::Align: either map or current patches are "
                "empty: %zu, %zu",
                current.size(), map.size());
    return result;
  }
  pcl::PointCloud<pcl::PointXYZ>::Ptr map_cloud(
      new pcl::PointCloud<pcl::PointXYZ>());

  map_cloud->points.reserve(map.size());

  for (const auto& patch : map) {
    pcl::PointXYZ pt;
    pt.x = static_cast<float>(patch.centroid.x());
    pt.y = static_cast<float>(patch.centroid.y());
    pt.z = 0.0f;
    map_cloud->points.push_back(pt);
  }

  map_cloud->width = static_cast<uint32_t>(map_cloud->points.size());
  map_cloud->height = 1;
  map_cloud->is_dense = true;

  pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
  kdtree.setInputCloud(map_cloud);

  // ------------------------------------------------------------
  // Transform estimated by this function:
  //
  //   p_map = R * p_current + t
  //
  // where:
  //   p_current is a ground patch centroid in the current LiDAR frame,
  //   p_map     is the same patch centroid expressed in the map frame.
  //
  // R_initial and t_initial should be the current best estimate of:
  //
  //   map <- current_lidar
  //
  // Usually this is the previous global pose.
  //
  // This optimizer only changes:
  //
  //   state = [dz, droll, dpitch]
  //
  // It does not estimate x, y, or yaw. Those come from planar registration.
  // ------------------------------------------------------------
  Eigen::Matrix3d R = R_initial;
  Eigen::Vector3d t = t_initial;

  int final_matches = 0;
  double final_abs_residual_sum = 0.0;

  for (int iter = 0; iter < params_.max_iterations; ++iter) {
    // Normal equation system:
    //
    //   H * dx = -b
    //
    // State increment:
    //
    //   dx = [dz, droll, dpitch]
    //
    // dz is a map-frame vertical translation correction.
    // droll and dpitch are local/current LiDAR-frame rotation corrections
    // because we use a right SO(3) update:
    //
    //   R_new = R * Exp([droll, dpitch, 0])
    Eigen::Matrix3d H = Eigen::Matrix3d::Zero();
    Eigen::Vector3d b = Eigen::Vector3d::Zero();

    int num_matches = 0;
    double abs_residual_sum = 0.0;

    double min_x = std::numeric_limits<double>::infinity();
    double max_x = -std::numeric_limits<double>::infinity();
    double min_y = std::numeric_limits<double>::infinity();
    double max_y = -std::numeric_limits<double>::infinity();

    for (const auto& cur : current) {
      // Transform current patch centroid and normal into map frame:
      //
      //   p          = R * cur.centroid + t
      //   cur_normal = R * cur.normal
      //
      // Both are used for correspondence search.
      Eigen::Vector3d p = R * cur.centroid + t;
      Eigen::Vector3d cur_normal = R * cur.normal;

      // Find nearest map patch in map frame.
      //
      // The search uses transformed XY position and checks normal consistency.
      int match_idx = FindNearestPatchKDTree(p, cur_normal, map, kdtree);
      if (match_idx < 0) {
        continue;
      }

      min_x = std::min(min_x, p.x());
      max_x = std::max(max_x, p.x());
      min_y = std::min(min_y, p.y());
      max_y = std::max(max_y, p.y());

      const auto& mp = map[match_idx];

      const Eigen::Vector3d& q = mp.centroid;
      const Eigen::Vector3d& n = mp.normal;

      // Point-to-plane residual in map frame:
      //
      //   r = n_map^T * (p_current_in_map - q_map)
      //
      // where:
      //   n is the matched map patch normal,
      //   q is the matched map patch centroid.
      //
      // Positive/negative sign is okay as long as the Jacobian uses the same
      // convention, because the update solves dx = -H^-1 b.
      double r = n.dot(p - q);
      if (r > params_.max_match_z_difference) {
        continue;
      }
      // Translation part of the Jacobian.
      //
      // This optimizer only updates z:
      //
      //   p_new = p + [0, 0, dz]
      //
      // Therefore:
      //
      //   dr / dz = n.z
      double J_dz = n.z();

      // Rotation part of the Jacobian for right perturbation.
      //
      // Current transform:
      //
      //   p = R * cur.centroid + t
      //
      // Right SO(3) update:
      //
      //   R_new = R * Exp(dtheta)
      //
      // For small dtheta:
      //
      //   Exp(dtheta) * cur ≈ cur + dtheta x cur
      //                    ≈ cur - skew(cur) * dtheta
      //
      // Therefore:
      //
      //   dp / dtheta = -R * skew(cur)
      //
      // Residual:
      //
      //   r = n^T * p
      //
      // so:
      //
      //   dr / dtheta = n^T * dp/dtheta
      //                = -n^T * R * skew(cur)
      //
      // We keep only roll and pitch components. Yaw is intentionally not
      // optimized here because yaw comes from planar registration.
      Eigen::Matrix3d cur_skew = Skew(cur.centroid);
      Eigen::RowVector3d J_rot = -n.transpose() * R * cur_skew;

      double J_roll = J_rot.x();
      double J_pitch = J_rot.y();
      // State order:
      //
      //   dx = [dz, droll, dpitch]
      Eigen::Vector3d J;
      J << J_dz, J_roll, J_pitch;
      // Patch weight.
      //
      // Start with support-based patch weight. If it is invalid, fall back
      // to 1.
      double weight = cur.weight;
      if (!std::isfinite(weight) || weight <= 0.0) {
        weight = 1.0;
      }

      // Down-weight far patches.
      //
      // Far ground patches often have larger noise and less reliable normals.
      double range = cur.centroid.head<2>().norm();
      double range_weight =
          1.0 / (1.0 + params_.range_weight_coeff * range * range);
      weight *= range_weight;

      // Accumulate weighted normal equations:
      //
      //   H += w * J * J^T
      //   b += w * J * r
      //
      // Later:
      //
      //   dx = -H^-1 * b
      H += weight * J * J.transpose();
      b += weight * J * r;

      ++num_matches;
      abs_residual_sum += std::abs(r);
    }

    if (num_matches < params_.min_matches) {
      RCLCPP_WARN(logger_,
                  "GroundRegistration::Align: low amount of matches %d %d",
                  num_matches, params_.min_matches);
      result.valid = false;
      return result;
    }

    if (params_.use_imu_prior) {
      Eigen::Matrix3d R_err = R_imu_prior.transpose() * R;
      Eigen::Vector3d rot_err = LogSO3(R_err);

      {
        double r_roll = rot_err.x();
        Eigen::Vector3d J;
        J << 0.0, 1.0, 0.0;

        H += params_.imu_roll_weight * J * J.transpose();
        b += params_.imu_roll_weight * J * r_roll;
      }

      {
        double r_pitch = rot_err.y();
        Eigen::Vector3d J;
        J << 0.0, 0.0, 1.0;

        H += params_.imu_pitch_weight * J * J.transpose();
        b += params_.imu_pitch_weight * J * r_pitch;
      }
    }
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> es(H);
    Eigen::Vector3d evals = es.eigenvalues();

    double lambda_min = evals(0);
    double lambda_max = evals(2);
    double condition =
        lambda_max / std::max(lambda_min, params_.condition_lambda_floor);
    bool degenerate = lambda_min < params_.min_condition_eigenvalue ||
                      condition > params_.max_condition_number;

    // Add separate damping for z, roll, and pitch.
    //
    // This improves numerical stability and also lets you tune how much each
    // component is allowed to move.
    Eigen::Matrix3d H_damped = H;
    H_damped(0, 0) += params_.damping_z;
    H_damped(1, 1) += params_.damping_roll;
    H_damped(2, 2) += params_.damping_pitch;

    // Solve Gauss-Newton step:
    //
    //   dx = -H^-1 * b
    //
    // dx = [dz, droll, dpitch]
    Eigen::Vector3d dx = -H_damped.ldlt().solve(b);

    if (!dx.allFinite()) {
      result.valid = false;
      RCLCPP_WARN(logger_, "GroundRegistration::Align: dx has infinite values");
      return result;
    }

    double x_span = max_x - min_x;
    double y_span = max_y - min_y;

    // If there is no IMU prior and the system is degenerate, suppress
    // roll/pitch. Otherwise ground registration can invent tilt from weak
    // correspondences.
    if (!params_.use_imu_prior && degenerate) {
      dx(1) = 0.0;
      dx(2) = 0.0;
    }

    // Pitch needs enough x-direction coverage.
    //
    // Intuition:
    //   pitch changes height as a function of x.
    //   If x coverage is small, pitch is weakly observable.
    if (x_span < params_.min_x_span_for_pitch && !params_.use_imu_prior) {
      RCLCPP_WARN(logger_, "Pitch is weakly observable");
      dx(2) = 0.0;
    }

    // Roll needs enough y-direction coverage.
    //
    // Intuition:
    //   roll changes height as a function of y.
    //   If y coverage is small, roll is weakly observable.
    if (y_span < params_.min_y_span_for_roll && !params_.use_imu_prior) {
      RCLCPP_WARN(logger_, "Roll is weakly observable");
      dx(1) = 0.0;
    }

    // Limit per-iteration step size.
    //
    // The final correction can still be larger after several iterations, but no
    // single iteration can make a dangerous jump.
    dx(0) = std::clamp(dx(0), -params_.max_dz, params_.max_dz);
    dx(1) = std::clamp(dx(1), -params_.max_roll, params_.max_roll);
    dx(2) = std::clamp(dx(2), -params_.max_pitch, params_.max_pitch);

    // Apply z correction in map frame.
    //
    // This optimizer does not update x/y translation.
    t.z() += dx(0);

    // Apply right SO(3) update for local/body-frame roll-pitch correction.
    //
    //   R_new = R * Exp([droll, dpitch, 0])
    //
    // Yaw correction is zero because yaw is handled by planar registration.
    Eigen::Vector3d dtheta(dx(1), dx(2), 0.0);
    R = R * ExpSO3(dtheta);

    final_matches = num_matches;
    final_abs_residual_sum = abs_residual_sum;

    if (dx.norm() < params_.convergence_eps) {
      break;
    }
  }

  RCLCPP_WARN_EXPRESSION(
      logger_, final_matches < params_.min_matches,
      "GroundRegistration::Align: low amount of final matches %d %d",
      final_matches, params_.min_matches);

  // Final estimated transform:
  //
  //   result.R, result.t : map <- current_lidar
  //
  // Meaning:
  //
  //   p_map = result.R * p_current + result.t
  //
  // Only z, roll, and pitch were optimized here.
  // x, y, and yaw should be merged from planar registration.
  result.R = R;
  result.t = t;
  result.num_matches = final_matches;
  result.mean_abs_residual =
      final_matches > 0 ? final_abs_residual_sum / final_matches : 0.0;
  result.valid = final_matches >= params_.min_matches;

  return result;
}

std::vector<Eigen::Vector2d> PlanarRegistration::ExtractPoints(
    const CloudMsg& cloud, const std::vector<PointLabels>& labels) const {
  std::vector<Eigen::Vector2d> points;
  size_t n = static_cast<size_t>(cloud.width);
  n *= static_cast<size_t>(cloud.height);
  if (n == 0 || labels.size() != n) return points;
  points.reserve(n);
  sensor_msgs::PointCloud2ConstIterator<float> x_it(cloud, "x");
  sensor_msgs::PointCloud2ConstIterator<float> y_it(cloud, "y");

  for (size_t idx = 0; idx < n; ++x_it, ++y_it, ++idx) {
    float x = *x_it;
    float y = *y_it;
    if (!IsFinitePoint(x, y)) continue;
    if (labels[idx] == PointLabels::NON_GROUND) {
      points.emplace_back(Eigen::Vector2d(x, y));
    }
  }
  return points;
}

std::vector<Eigen::Vector2d> PlanarRegistration::ExtractFilteredPoints(
    const CloudMsg& cloud, const std::vector<PointLabels>& labels) const {
  size_t n = static_cast<size_t>(cloud.width);
  n *= static_cast<size_t>(cloud.height);
  if (n == 0 || labels.size() != n) return {};

  ResetCellValues(extraction_grid_);

  sensor_msgs::PointCloud2ConstIterator<float> x_it(cloud, "x");
  sensor_msgs::PointCloud2ConstIterator<float> y_it(cloud, "y");

  for (size_t idx = 0; idx < n; ++x_it, ++y_it, ++idx) {
    if (labels[idx] != PointLabels::NON_GROUND) continue;

    float x = *x_it;
    float y = *y_it;
    if (!IsFinitePoint(x, y)) continue;

    int ix = static_cast<int>(std::floor(x / params_.voxel_size));
    int iy = static_cast<int>(std::floor(y / params_.voxel_size));
    auto& v = extraction_grid_[{ix, iy}];
    v.count++;
    v.sum += Eigen::Vector2d(x, y);
  }

  for (auto it = extraction_grid_.begin(); it != extraction_grid_.end();) {
    if (it->second.count == 0) {
      it = extraction_grid_.erase(it);
    } else {
      ++it;
    }
  }

  std::vector<Eigen::Vector2d> points;
  points.reserve(extraction_grid_.size());

  for (const auto& [key, v] : extraction_grid_) {
    if (v.count < params_.min_points_per_voxel) continue;
    points.push_back(v.sum / v.count);
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
    v.count++;
    v.sum += pt;
  }

  std::vector<Eigen::Vector2d> outp;
  outp.reserve(grid_.size());

  for (const auto& [key, v] : grid_) {
    if (v.count < params_.min_points_per_voxel) continue;
    outp.push_back(v.sum / v.count);
  }
  return outp;
}

PlanarRegistrationResult PlanarRegistration::Align(
    const std::vector<Eigen::Vector2d>& map,
    const std::vector<Eigen::Vector2d>& current,
    const Eigen::Matrix2d& R_initial, const Eigen::Vector2d& t_initial) {
  PlanarRegistrationResult result;

  if (map.empty() || current.empty()) {
    RCLCPP_WARN(logger_,
                "PlanarRegistration::Align: either map or current points are "
                "empty: current=%zu, map=%zu",
                current.size(), map.size());
    return result;
  }

  // ------------------------------------------------------------
  // Build KD-tree for map points.
  //
  // `map` points are already expressed in the global/map frame.
  // The KD-tree is therefore queried in the map frame.
  //
  // The map is fixed during this Align() call, so the KD-tree is built once
  // per call and reused for all ICP iterations.
  // ------------------------------------------------------------
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

  // ------------------------------------------------------------
  // Transform estimated by this function:
  //
  //   p_map = R * p_current + t
  //
  // where:
  //   p_current is a 2D point in the current LiDAR frame,
  //   p_map     is the same point expressed in the map frame.
  //
  // R_initial and t_initial should therefore be the current best estimate of:
  //
  //   map <- current_lidar
  //
  // Usually this comes from the previous global pose.
  // ------------------------------------------------------------
  Eigen::Matrix2d R = R_initial;
  Eigen::Vector2d t = t_initial;

  int final_matches = 0;
  double final_residual_sum = 0.0;

  const double max_match_dist2 =
      params_.max_match_distance * params_.max_match_distance;

  for (int iter = 0; iter < params_.max_iterations; ++iter) {
    // Normal equation system:
    //
    //   H * dx = -b
    //
    // State increment:
    //
    //   dx = [dtx, dty, dyaw]
    //
    // dtx, dty are map-frame translation corrections.
    // dyaw is a local/current-frame yaw correction because we use right update:
    //
    //   R_new = R * dR
    //   t_new = t + dt
    Eigen::Matrix3d H = Eigen::Matrix3d::Zero();
    Eigen::Vector3d b = Eigen::Vector3d::Zero();

    int num_matches = 0;
    double residual_sum = 0.0;

    for (const auto& cur_pt : current) {
      // Transform current LiDAR-frame point into map frame using the current
      // estimate:
      //
      //   p = R * cur_pt + t
      //
      // This transformed point is what must be matched against the map.
      Eigen::Vector2d p = R * cur_pt + t;

      // --------------------------------------------------------
      // KD-tree nearest-neighbor search in map frame.
      //
      // Important:
      //   query = transformed point p
      //
      // Do not query with raw cur_pt, because cur_pt is still in the current
      // LiDAR frame while the KD-tree stores points in the map frame.
      // --------------------------------------------------------
      pcl::PointXYZ query;
      query.x = static_cast<float>(p.x());
      query.y = static_cast<float>(p.y());
      query.z = 0.0f;

      std::vector<int> indices(1);
      std::vector<float> dists2(1);

      int found = kdtree.nearestKSearch(query, 1, indices, dists2);

      if (found <= 0) {
        continue;
      }

      if (static_cast<double>(dists2[0]) > max_match_dist2) {
        continue;
      }

      const Eigen::Vector2d matched_pt = map[indices[0]];

      // Point-to-point residual in map frame:
      //
      //   r = p_estimated_map - p_matched_map
      //   r = R * cur_pt + t - matched_pt
      //
      // r is 2D:
      //
      //   r = [rx, ry]
      Eigen::Vector2d r = p - matched_pt;

      // Jacobian for right yaw update.
      //
      // We update rotation as:
      //
      //   R_new = R * dR
      //
      // For small dyaw:
      //
      //   dR * cur_pt ≈ cur_pt + dyaw * [-cur_pt.y, cur_pt.x]
      //
      // So the local derivative is:
      //
      //   d(cur_pt) / d_yaw = [-cur_pt.y, cur_pt.x]
      //
      // Since the point is then transformed by R into the map frame:
      //
      //   d(p_map) / d_yaw = R * [-cur_pt.y, cur_pt.x]
      Eigen::Vector2d d_yaw_local;
      d_yaw_local << -cur_pt.y(), cur_pt.x();

      Eigen::Vector2d d_yaw_map = R * d_yaw_local;

      // Jacobian of residual r with respect to:
      //
      //   dx = [dtx, dty, dyaw]
      //
      // Translation correction is additive in map frame:
      //
      //   dr / dtx = [1, 0]
      //   dr / dty = [0, 1]
      //
      // Yaw correction is right-multiplied, so:
      //
      //   dr / dyaw = R * [-cur_pt.y, cur_pt.x]
      Eigen::Matrix<double, 2, 3> J;
      J << 1.0, 0.0, d_yaw_map.x(), 0.0, 1.0, d_yaw_map.y();

      double weight = 1.0;

      // Accumulate weighted normal equations:
      //
      //   H += J^T * J
      //   b += J^T * r
      //
      // Later we solve:
      //
      //   dx = -H^-1 * b
      H += weight * J.transpose() * J;
      b += weight * J.transpose() * r;

      ++num_matches;
      residual_sum += r.norm();
    }

    if (num_matches < params_.min_matches) {
      RCLCPP_WARN(logger_,
                  "PlanarRegistration::Align: low amount of matches %d / %d",
                  num_matches, params_.min_matches);
      result.valid = false;
      return result;
    }

    // Add diagonal damping for numerical stability.
    // This is similar to a simple Levenberg-Marquardt regularization.
    Eigen::Matrix3d H_damped = H;
    H_damped.diagonal().array() += params_.damping;

    // Solve Gauss-Newton step:
    //
    //   dx = -H^-1 * b
    //
    // dx = [dtx, dty, dyaw]
    Eigen::Vector3d dx = -H_damped.ldlt().solve(b);

    if (!dx.allFinite()) {
      RCLCPP_WARN(logger_,
                  "PlanarRegistration::Align: dx has invalid values %s",
                  VectorToString(dx).c_str());
      result.valid = false;
      return result;
    }

    // Reject obviously bad optimizer jumps before applying them.
    // This protects the map from large wrong correspondences.
    if (std::abs(dx.x()) > params_.max_dx ||
        std::abs(dx.y()) > params_.max_dy ||
        std::abs(dx.z()) > params_.max_dyaw) {
      RCLCPP_WARN(logger_,
                  "PlanarRegistration::Align: dx has too big values %s",
                  VectorToString(dx).c_str());
      result.valid = false;
      return result;
    }

    // Limit per-iteration step size.
    // The total correction can still be larger after several iterations.
    dx.x() = std::clamp(dx.x(), -params_.max_dx_step, params_.max_dx_step);
    dx.y() = std::clamp(dx.y(), -params_.max_dy_step, params_.max_dy_step);
    dx.z() = std::clamp(dx.z(), -params_.max_dyaw_step, params_.max_dyaw_step);

    const double dtheta = dx.z();
    const double c = std::cos(dtheta);
    const double s = std::sin(dtheta);

    Eigen::Matrix2d dR;
    dR << c, -s, s, c;

    Eigen::Vector2d dt(dx.x(), dx.y());

    // Apply right update.
    //
    // Rotation:
    //
    //   R_new = R * dR
    //
    // This means dyaw is applied in the local/current LiDAR frame.
    //
    // Translation:
    //
    //   t_new = t + dt
    //
    // dt is additive in the map frame.
    //
    // This is consistent with the Jacobian above:
    //
    //   d(p_map) / dyaw = R * [-cur_pt.y, cur_pt.x]
    R = R * dR;
    t = t + dt;

    final_matches = num_matches;
    final_residual_sum = residual_sum;

    if (dx.norm() < params_.convergence_eps) {
      break;
    }
  }

  // Final estimated transform:
  //
  //   result.R, result.t : map <- current_lidar
  //
  // Meaning:
  //
  //   p_map = result.R * p_current + result.t
  result.R = R;
  result.t = t;
  result.mean_residual =
      final_matches > 0 ? final_residual_sum / final_matches : 0.0;
  result.matches = final_matches;
  result.valid = final_matches >= params_.min_matches;

  return result;
}
