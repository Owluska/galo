# Ground Aware LiDAR Odometry

`ground_aware_lidar_odometry` is a ROS 2 package for LiDAR odometry using ground-aware registration and planar object registration.

The system estimates LiDAR motion by combining:

- IMU-assisted rotational prediction
- LiDAR deskewing
- Ground segmentation
- Ground patch extraction
- Ground-based roll, pitch, and height correction
- 2D planar registration for horizontal translation and yaw
- Local map accumulation using recent ground and object frames

The project is currently experimental and intended for development and testing of ground-aware odometry pipelines.

---

## Overview

The odometry pipeline processes each LiDAR scan as follows:

1. Synchronize and deskew the LiDAR cloud using IMU angular velocity.
2. Segment the cloud into ground and non-ground points.
3. Extract ground patches from ground-labeled points.
4. Extract and voxel-filter planar/object points from non-ground points.
5. Register the current planar points against the accumulated object map.
6. Register current ground patches against the accumulated ground map.
7. Merge the planar and ground registration results into a global pose estimate.
8. Transform the current scan features into the map frame and update local maps.

The system keeps separate local maps for:

- Ground patches
- Planar/object points

These maps are rebuilt from a sliding window of recent frames.

---

## Main Components

### LiDAR Deskewing

The deskew module compensates for LiDAR motion distortion using IMU angular velocity.

Input:

- Raw LiDAR `sensor_msgs/msg/PointCloud2`
- IMU angular velocity

Output:

- Deskewed point cloud

Published topic:

```txt
/GALO/deskewed_cloud
````

### Ground Segmentation

Ground segmentation is performed using a 2D grid over the LiDAR scan.

For each grid cell, the minimum observed `z` value is estimated and smoothed using neighboring cells. Points close to the local ground height are classified as ground.

Labels:

* `GROUND`
* `NON_GROUND`
* `UNKNOWN`

Debug output:

```txt
/GALO/colored_cloud
```

### Ground Patch Extraction

Ground points are grouped into grid cells. For each cell, a local covariance matrix is computed and eigen-decomposition is used to estimate:

* Patch centroid
* Patch normal
* Thickness
* Planarity
* Support point count

Invalid patches are rejected using thresholds such as:

* Minimum number of points
* Maximum thickness
* Minimum normal `z`
* Minimum planarity

Debug output:

```txt
/GALO/ground_patch_normals
```

### Ground Registration

Ground registration aligns current ground patches to the accumulated ground map.

It estimates mainly:

* Vertical translation `z`
* Roll correction
* Pitch correction

The registration uses point-to-plane residuals:

```txt
r = n_map · (p_current_transformed - p_map)
```

Nearest ground patches are found using a KD-tree.

An IMU prior can be used to stabilize roll and pitch.

### Planar Registration

Planar registration aligns current non-ground 2D points against the accumulated object map.

It estimates:

* `x` translation
* `y` translation
* Yaw rotation

The registration uses nearest-neighbor ICP-style matching with a KD-tree.

Only the XY projection is used.

---

## ROS Interfaces

### Subscribed Topics

```txt
/Sensor/lidar_front/rslidar_points
```

Type:

```txt
sensor_msgs/msg/PointCloud2
```

Raw LiDAR point cloud.

```txt
/Sensor/imu_front/data
```

Type:

```txt
sensor_msgs/msg/Imu
```

IMU orientation and angular velocity.

### Published Topics

```txt
/GALO/deskewed_cloud
```

Type:

```txt
sensor_msgs/msg/PointCloud2
```

Deskewed LiDAR cloud.

```txt
/GALO/colored_cloud
```

Type:

```txt
sensor_msgs/msg/PointCloud2
```

Debug cloud with ground/non-ground coloring.

```txt
/GALO/ground_patch_normals
```

Type:

```txt
visualization_msgs/msg/MarkerArray
```

Ground patch normal markers.

```txt
/GALO/translation
```

Type:

```txt
geometry_msgs/msg/Point
```

Debug translation output.

```txt
/GALO/eulers
```

Type:

```txt
geometry_msgs/msg/Point
```

Debug roll, pitch, yaw output from GALO.

```txt
/GALO/imu_eulers
```

Type:

```txt
geometry_msgs/msg/Point
```

Debug roll, pitch, yaw output from IMU delta.

---

## Dependencies

This package depends on:

* ROS 2
* `rclcpp`
* `std_msgs`
* `sensor_msgs`
* `geometry_msgs`
* `visualization_msgs`
* `tf2_ros`
* `tf2_sensor_msgs`
* `Eigen3`
* `PCL`
* `pcl_conversions`
* `user_msgs`

For ROS 2 Humble, install common dependencies with:

```bash
sudo apt update
sudo apt install \
  ros-humble-pcl-conversions \
  ros-humble-pcl-ros \
  libpcl-dev \
  libeigen3-dev
```

---

## Building

Clone the package into a ROS 2 workspace:

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone <your-repository-url> ground_aware_lidar_odometry
```

Build with `colcon`:

```bash
cd ~/ros2_ws
colcon build --packages-select ground_aware_lidar_odometry
```

Source the workspace:

```bash
source install/setup.bash
```

---

## Running

Run the node directly:

```bash
ros2 run ground_aware_lidar_odometry node
```
but better via launch file:

```bash
ros2 launch ground_aware_lidar_odometry galo.launch.py
```

or in sim mode:

```bash
ros2 launch ground_aware_lidar_odometry galo.launch.py use_sim_time:=True
```

if TF publication needed:

```bash
ros2 launch static_tf_publisher static_tf.launch.py
```

if Foxglove needed:

```bash
ros2 launch foxglove_bridge foxglove_bridge_launch.xml
```

example for running bag comand:

```bash
ros2 bag play 109_2025-08-14-19-24-54_1213e_ros2 --clock --topics /tf /Sensor/imu_front/data /Sensor/lidar_front/rslidar_points /Sensor/gnss/trimble_nmea_gga /Sensor/gnss/orientation /SC/state /SC/pure_state /FB/wangle_feedback /FB/wheel_speed_feedback -r 0.5
```
---

## TF Requirements

The node expects a static or dynamic transform between the IMU frame and LiDAR frame.

Internally, it looks up:

```txt
imu_frame <- lidar_frame
```

This transform is used to convert IMU rotation delta into the LiDAR frame.

Make sure the transform is available before running the odometry node.

Example static transform command:

```bash
ros2 run tf2_ros static_transform_publisher \
  x y z roll pitch yaw imu_frame lidar_frame
```

Replace the values with the calibrated IMU-to-LiDAR extrinsics.

---

## Debugging

The node prints timing information such as:

```txt
Time measurements (ms): deskew - 3, segmentation - 4, objects_extraction - 2, ground_extraction - 10, planar_registration - 30, ground_registration - 1
```

Useful debug logs include:

```txt
GALO pose
Ground result
IMU delta
Reg debug
Pose debug
```

Recommended checks:

* Ground registration should not produce large roll/pitch drift.
* IMU delta should be small between consecutive LiDAR scans.
* Planar registration should have enough matches.
* Ground registration should have enough matches.
* `dz` should not drift rapidly unless the vehicle is actually changing elevation.
* Mean residuals should remain bounded.

---

## Performance Notes

The registration modules use KD-trees for nearest-neighbor search.

This avoids the original brute-force `O(N²)` matching cost and significantly reduces registration time.

Ground registration and planar registration both build KD-trees from their current local maps. Further optimization is possible by caching KD-trees and rebuilding them only when the local map changes.

---

## Known Limitations

* The system is currently experimental.
* Ground registration can become unstable if ground patches are sparse or incorrectly segmented.
* Planar registration may fail in open areas with few non-ground structures.
* Roll and pitch should usually be constrained by the IMU prior.
* Large accumulated map errors can affect future scan-to-map registration.
* The current implementation uses local sliding-window maps rather than a global map optimization backend.

---

## Suggested Visualization

Useful RViz displays:

* `/GALO/deskewed_cloud`
* `/GALO/colored_cloud`
* `/GALO/ground_patch_normals`
* TF tree
* Odometry/path topic if added later

---

## Future Work

Possible improvements:

* Publish a proper `nav_msgs/msg/Odometry` message.
* Publish a `nav_msgs/msg/Path`.
* Add configurable parameters through ROS 2 parameters.
* Cache KD-trees instead of rebuilding them every frame.
* Add robust loss functions for registration residuals.
* Add outlier rejection for bad ground patches.
* Add pose covariance estimation.
* Add bag replay examples.
* Add launch and RViz configuration files.
* Add unit tests for registration and transform logic.

---

## License

TODO: Add license.

---

## Author

TODO: Add author information.


