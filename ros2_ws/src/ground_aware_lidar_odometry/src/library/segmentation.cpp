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