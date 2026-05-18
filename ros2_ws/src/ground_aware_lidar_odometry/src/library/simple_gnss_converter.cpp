#include "ground_aware_lidar_odometry/simple_gnss_converter.hpp"

GnssLocalConverter::GnssLocalConverter(const GnssLocalizationParams& params)
    : params_(params) {
  int zone_tmp;
  bool northp_tmp;
  double gamma, k;

  GeographicLib::UTMUPS::Forward(params_.base_lat, params_.base_lon, zone_tmp,
                                 northp_tmp, base_easting_, base_northing_,
                                 gamma, k, params_.zone);

  if (zone_tmp != params_.zone) {
    RCLCPP_WARN(logger_, "Base point projected to UTM zone %d, expected %d",
                zone_tmp, params_.zone);
  }
  if (northp_tmp != params_.northp) {
    RCLCPP_WARN(logger_, "Base point projected to UTM zone %s, expected %s",
                northp_tmp, params_.northp);
  }
}

Eigen::Vector3d GnssLocalConverter::ToLocal(double lon, double lat,
                                            double altitude) const {
  int zone_tmp;
  bool northp_tmp;
  double easting, northing;
  double gamma, k;
  try {
    GeographicLib::UTMUPS::Forward(lat, lon, zone_tmp, northp_tmp, easting,
                                   northing, gamma, k, params_.zone);
  } catch (const GeographicLib::GeographicErr& ex) {
    RCLCPP_WARN(logger_, "Simple gnss converter: %s", ex.what());
  }

  if (zone_tmp != params_.zone) {
    RCLCPP_WARN(logger_, "Base point projected to UTM zone %d, expected %d",
                zone_tmp, params_.zone);
  }
  if (northp_tmp != params_.northp) {
    RCLCPP_WARN(logger_, "Base point projected to UTM zone %s, expected %s",
                northp_tmp, params_.northp);
  }

  Eigen::Vector3d p;
  p.x() = easting - base_easting_;
  p.y() = northing - base_northing_;
  p.z() = altitude;
  return p;
}

double GnssLocalConverter::YawFromGnssHeading(double heading_deg) {
  const double heading_rad = DegToRad(heading_deg);
  return -NormalizeAngle0To2Pi(heading_rad);
}
