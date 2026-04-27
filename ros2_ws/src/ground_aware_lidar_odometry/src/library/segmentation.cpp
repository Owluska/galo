#include "ground_aware_lidar_odometry/segmentation.hpp"
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
  grid_.clear();
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
    cell.min_z = std::min(cell.min_z, z);
  }
}

void Segmentation::FillSmoothedGrid() {
  smoothed_ground_z_.clear();
  for (const auto& [key, cell] : grid_) {
    smoothed_ground_z_[key] = GetNeighborGroundZ(key);
  }
}

double Segmentation::GetNeighborGroundZ(const CellKey& key) const {
  std::vector<double> zs;

  for (int dx = -1; dx <= 1; ++dx) {
    for (int dy = -1; dy <= 1; ++dy) {
      CellKey nk{key.x + dx, key.y + dy};

      auto it = grid_.find(nk);
      if (it == grid_.end()) continue;

      const auto& cell = it->second;
      if (cell.count < params_.min_points_per_cell) continue;

      zs.push_back(cell.min_z);
    }
  }

  if (zs.empty()) {
    return std::numeric_limits<double>::quiet_NaN();
  }

  std::sort(zs.begin(), zs.end());
  return zs[zs.size() / 2];  // median
}

SegmentationResult Segmentation::Classify(const CloudMsg& msg) {
  SegmentationResult res;
  res.msg = msg;
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
      res.labels[idx] = PointLabels::NON_GROUND;
      continue;
    }
    double ground_z = gz_it->second;
    res.labels[idx] = (z - ground_z) < params_.ground_height_threshold
                          ? PointLabels::GROUND
                          : PointLabels::NON_GROUND;
  }
  return res;
}

sensor_msgs::msg::PointCloud2 Segmentation::MakeColoredCloud(
    const SegmentationResult& result) const {
  const auto& in = result.msg;

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
  patches_.reserve(20000);
  valid_patches_.reserve(5000);
}

std::pair<int, int> GroundPatchExtractor::GetIndexes(float x, float y) const {
  int ix = static_cast<int>(std::floor(x / params_.cell_size));
  int iy = static_cast<int>(std::floor(y / params_.cell_size));
  return std::make_pair(ix, iy);
}

std::vector<GroundPatch> GroundPatchExtractor::Extract(
    const CloudMsg& cloud, const std::vector<PointLabels>& labels) {
  size_t n = static_cast<size_t>(cloud.width);
  n *= static_cast<size_t>(cloud.height);
  if (n == 0 || labels.size() != n) return valid_patches_;
  patches_.clear();
  valid_patches_.clear();
  sensor_msgs::PointCloud2ConstIterator<float> x_it(cloud, "x");
  sensor_msgs::PointCloud2ConstIterator<float> y_it(cloud, "y");
  sensor_msgs::PointCloud2ConstIterator<float> z_it(cloud, "z");

  for (size_t idx = 0; idx < n; ++x_it, ++y_it, ++z_it, ++idx) {
    double x = static_cast<double>(*x_it);
    double y = static_cast<double>(*y_it);
    double z = static_cast<double>(*z_it);
    auto label = labels[idx];
    if (label != PointLabels::GROUND) continue;

    auto [ix, iy] = GetIndexes(x, y);
    CellKey key{ix, iy};
    PatchCell& cell = patches_[key];
    Eigen::Vector3d point(x, y, z);
    cell.points.push_back(point);
  }

  for (const auto& [key, patch_cell] : patches_) {
    const auto& pts = patch_cell.points;
    const int N = pts.size();
    if (N < params_.min_points) continue;
    Eigen::Vector3d centroid = Eigen::Vector3d::Zero();

    for (const auto& pt : pts) {
      centroid += pt;
    }
    centroid /= static_cast<double>(N);
    Eigen::Matrix3d cov = Eigen::Matrix3d::Zero();
    for (const auto& p : pts) {
      Eigen::Vector3d d = p - centroid;
      cov += d * d.transpose();  // outer product;
    }
    cov /= static_cast<double>(N);
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver(cov);
    if (solver.info() != Eigen::Success) {
      continue;
    }
    Eigen::Vector3d eigenvalues = solver.eigenvalues();
    Eigen::Matrix3d eigenvectors = solver.eigenvectors();
    Eigen::Vector3d normal = eigenvectors.col(0);  // smallest eigenvalue

    if (normal.z() < 0.0) {  // make normal point upward
      normal = -normal;
    }
    double lambda0 = eigenvalues(0);
    double thickness = std::sqrt(lambda0);
    if (thickness > params_.max_thickness) {
      continue;
    }
    if (normal.z() < params_.min_normal_z) {
      continue;
    }
    double lambda1 = eigenvalues(1);
    double lambda2 = eigenvalues(2);

    double planarity = (lambda1 - lambda0) / lambda2;
    if (planarity < params_.min_planarity && lambda2 > 1e-4) continue;
    GroundPatch patch;
    patch.centroid = centroid;
    patch.normal = normal;
    patch.covariance = cov;
    patch.support = N;
    patch.weight = static_cast<double>(N);
    patch.planarity = planarity;
    patch.key = key;

    valid_patches_.push_back(patch);
  }
  return valid_patches_;
}

visualization_msgs::msg::MarkerArray
GroundPatchExtractor::MakeGroundPatchMarkers(
    const std_msgs::msg::Header& header, double normal_scale) const {
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
    p1.x = patch.centroid.x() + normal_scale * patch.normal.x();
    p1.y = patch.centroid.y() + normal_scale * patch.normal.y();
    p1.z = patch.centroid.z() + normal_scale * patch.normal.z();

    marker.points.push_back(p0);
    marker.points.push_back(p1);

    marker.scale.x = 0.08;  // shaft diameter
    marker.scale.y = 0.10;  // head diameter
    marker.scale.z = 0.12;  // head length

    marker.color.r = 0.0f;
    marker.color.g = 0.6f;
    marker.color.b = 1.0f;
    marker.color.a = 1.0f;

    marker.lifetime = rclcpp::Duration::from_seconds(0.2);

    markers.markers.push_back(marker);

    visualization_msgs::msg::Marker cell_marker;
    cell_marker.header = header;
    cell_marker.ns = "ground_patch_cells";
    cell_marker.id = id++;
    cell_marker.type = visualization_msgs::msg::Marker::CUBE;
    cell_marker.action = visualization_msgs::msg::Marker::ADD;

    cell_marker.pose.position.x = patch.centroid.x();
    cell_marker.pose.position.y = patch.centroid.y();
    cell_marker.pose.position.z = patch.centroid.z() + 0.02;

    cell_marker.pose.orientation.w = 1.0;

    cell_marker.scale.x = params_.cell_size;
    cell_marker.scale.y = params_.cell_size;
    cell_marker.scale.z = 0.03;

    cell_marker.color.r = 0.0f;
    cell_marker.color.g = 0.8f;
    cell_marker.color.b = 1.0f;
    cell_marker.color.a = 0.25f;

    cell_marker.lifetime = rclcpp::Duration::from_seconds(0.2);

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

    double normal_dot = normal.dot(m.normal);
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

GroundRegistrationResult GroundRegistration::Align(
    const std::vector<GroundPatch>& map,
    const std::vector<GroundPatch>& current) {
  GroundRegistrationResult result;
  if (current.empty() || map.empty()) {
    return result;
  }
  Eigen::Matrix3d R = Eigen::Matrix3d::Identity();
  Eigen::Vector3d t = Eigen::Vector3d::Zero();

  int final_matches = 0;
  double final_abs_residual_sum = 0;

  for (int iter = 0; iter < params_.max_iterations; ++iter) {
    Eigen::Matrix3d H = Eigen::Matrix3d::Zero();
    Eigen::Vector3d b = Eigen::Vector3d::Zero();
    int num_matches = 0;
    double abs_residual_sum = 0.0;
    for (const auto& cur : current) {
      Eigen::Vector3d p = R * cur.centroid + t;
      Eigen::Vector3d cur_normal = R * cur_normal;

      int match_idx = FindNearestPatch(p, cur_normal, map);
      if (match_idx < 0) {
        continue;
      }

      const auto& mp = map[match_idx];
      const Eigen::Vector3d& q = mp.centroid;
      const Eigen::Vector3d& n = mp.normal;

      // Point-to-plane residual:
      //
      // r = n^T (R * p_cur + t - q)
      //
      double r = n.dot(p - q);
      // Full SE3 Jacobian:
      //
      // J = [ n^T , -n^T * [p]_x ]
      //
      // But we only solve:
      //   x = [ dz, roll, pitch ]
      //
      // dz column:
      double J_dz = n.z();

      // Rotation columns:
      Eigen::Matrix3d p_skew = Skew(p);
      Eigen::RowVector3d J_rot = -n.transpose() * p_skew;

      double J_roll = J_rot.x();
      double J_pitch = J_rot.y();

      Eigen::Vector3d J;
      J << J_dz, J_roll, J_pitch;

      double weight = cur.weight;
      if (!std::isfinite(weight) || weight <= 0.0) {
        weight = 1.0;
      }

      H += weight * J * J.transpose();
      b += weight * J * r;

      ++num_matches;
      abs_residual_sum += std::abs(r);
    }

    if (num_matches < params_.min_matches) {
      result.valid = false;
      return result;
    }

    Eigen::Vector3d dx = -H.ldlt().solve(b);

    if (!dx.allFinite()) {
      result.valid = false;
      return result;
    }

    if (dx.norm() > params_.max_update_norm) {
      result.valid = false;
      return result;
    }

    double dz = dx(0);
    double droll = dx(1);
    double dpitch = dx(2);

    t.z() += dz;

    Eigen::Vector3d dtheta;
    dtheta << droll, dpitch, 0.0;

    R = ExpSO3(dtheta) * R;

    final_matches = num_matches;
    final_abs_residual_sum = abs_residual_sum;

    if (dx.norm() < 1e-5) {
      break;
    }
  }
  result.R = R;
  result.t = t;
  result.mean_abs_residual =
      final_matches > 0 ? final_abs_residual_sum / final_matches : 0.0;
  result.valid = final_matches >= params_.min_matches;
  return result;
}