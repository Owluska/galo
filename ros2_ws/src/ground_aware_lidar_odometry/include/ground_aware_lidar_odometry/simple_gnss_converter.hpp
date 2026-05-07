#include <GeographicLib/UTMUPS.hpp>

#include "ground_aware_lidar_odometry/utils.hpp"

struct GnssLocalizationParams {
  double base_lon = 75.46162227777778;
  double base_lat = 51.708812183333336;
  int zone = 43;
  bool northp = true;
};

class GnssLocalConverter {
 public:
  GnssLocalConverter(const GnssLocalizationParams& params);

  Eigen::Vector3d ToLocal(double lon, double lat, double altitude) const;
  double YawFromGnssHeading(double heading_deg);

 private:
  GnssLocalizationParams params_;

  double base_easting_;
  double base_northing_;

  rclcpp::Logger logger_ = rclcpp::get_logger("gnss_converter");
};