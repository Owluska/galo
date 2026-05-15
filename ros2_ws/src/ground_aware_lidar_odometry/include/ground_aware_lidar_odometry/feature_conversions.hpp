#pragma once

#include "ground_aware_lidar_odometry/frontend.hpp"
#include "ground_aware_lidar_odometry/msg/frame_features.hpp"

namespace ground_aware_lidar_odometry {

msg::FrameFeatures ToMsg(const ::FrameFeatures& features);
::FrameFeatures FromMsg(const msg::FrameFeatures& msg);

}  // namespace ground_aware_lidar_odometry
