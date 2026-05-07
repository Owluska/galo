cmake_minimum_required(VERSION 3.8)
project(ground_aware_lidar_odometry)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(robot_localization REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(nmea_msgs REQUIRED)
find_package(gps_msgs REQUIRED)
find_package(PCL REQUIRED COMPONENTS common kdtree)
find_package(pcl_conversions REQUIRED)
find_package(qarl_msgs REQUIRED)

set(LIB_SOURCES
  src/library/deskew.cpp
  src/library/segmentation.cpp
  src/library/utils.cpp
  src/library/params.cpp
  src/library/simple_gnss_converter.cpp
)

# Manually specify GeographicLib
set(GEOGRAPHICLIB_LIBRARY "/usr/lib/x86_64-linux-gnu/libGeographic.so")
add_library(GeographicLib SHARED IMPORTED)
set_target_properties(GeographicLib PROPERTIES IMPORTED_LOCATION "${GEOGRAPHICLIB_LIBRARY}")

add_executable(node src/node.cpp ${LIB_SOURCES})

target_include_directories(node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(
  node
  rclcpp
  std_msgs
  visualization_msgs
  sensor_msgs
  nav_msgs
  geometry_msgs
  tf2_ros
  tf2_sensor_msgs
  tf2_geometry_msgs
  pcl_conversions
  qarl_msgs
  robot_localization
)

# Plain-signature link libraries – matches ament_target_dependencies style
target_link_libraries(node
  Eigen3::Eigen
  ${PCL_LIBRARIES}
  GeographicLib
)

target_compile_definitions(node PUBLIC ${PCL_DEFINITIONS})

install(
  TARGETS node
  DESTINATION lib/${PROJECT_NAME}
)
install(
  DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()