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
find_package(rclcpp_components REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(robot_localization REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(common_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(nmea_msgs REQUIRED)
find_package(gps_msgs REQUIRED)
find_package(PCL REQUIRED COMPONENTS common kdtree)
find_package(pcl_conversions REQUIRED)
find_package(qarl_msgs REQUIRED)
find_package(OpenMP REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  msg/Point2D.msg
  msg/GroundPatch.msg
  msg/FrameFeatures.msg
  DEPENDENCIES std_msgs sensor_msgs geometry_msgs
)

set(LIB_SOURCES
  src/library/deskew.cpp
  src/library/segmentation.cpp
  src/library/frontend.cpp
  src/library/feature_conversions.cpp
  src/library/utils.cpp
  src/library/params.cpp
  src/library/simple_gnss_converter.cpp
  src/library/prediction.cpp
)

# Manually specify GeographicLib
set(GEOGRAPHICLIB_LIBRARY "/usr/lib/x86_64-linux-gnu/libGeographic.so")
add_library(GeographicLib SHARED IMPORTED)
set_target_properties(GeographicLib PROPERTIES IMPORTED_LOCATION "${GEOGRAPHICLIB_LIBRARY}")

add_library(galo_odometry_component SHARED src/node.cpp ${LIB_SOURCES})
add_executable(node src/node_main.cpp)
add_library(galo_deskew_component SHARED
  src/deskew_component.cpp
  src/library/deskew.cpp
  src/library/prediction.cpp
  src/library/utils.cpp
)
add_library(galo_frontend_component SHARED
  src/frontend_component.cpp
  src/library/frontend.cpp
  src/library/feature_conversions.cpp
  src/library/segmentation.cpp
  src/library/utils.cpp
)
add_executable(deskew_node src/deskew_node.cpp)
add_executable(frontend_node src/frontend_node.cpp)

target_include_directories(galo_odometry_component PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_include_directories(node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_include_directories(galo_deskew_component PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_include_directories(galo_frontend_component PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_include_directories(deskew_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_include_directories(frontend_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(
  galo_odometry_component
  rclcpp
  rclcpp_components
  std_msgs
  visualization_msgs
  sensor_msgs
  nav_msgs
  geometry_msgs
  common_msgs
  tf2_ros
  tf2_sensor_msgs
  tf2_geometry_msgs
  pcl_conversions
  qarl_msgs
  robot_localization
)

ament_target_dependencies(
  galo_deskew_component
  rclcpp
  rclcpp_components
  sensor_msgs
  geometry_msgs
  common_msgs
  tf2_ros
  qarl_msgs
  robot_localization
)

ament_target_dependencies(
  galo_frontend_component
  rclcpp
  rclcpp_components
  std_msgs
  visualization_msgs
  sensor_msgs
  geometry_msgs
  pcl_conversions
)

ament_target_dependencies(
  node
  rclcpp
)

ament_target_dependencies(
  deskew_node
  rclcpp
)

ament_target_dependencies(
  frontend_node
  rclcpp
)

# Plain-signature link libraries – matches ament_target_dependencies style
target_link_libraries(galo_odometry_component
  Eigen3::Eigen
  ${PCL_LIBRARIES}
  GeographicLib
  OpenMP::OpenMP_CXX
)
target_link_libraries(node
  galo_odometry_component
)
target_link_libraries(galo_deskew_component
  Eigen3::Eigen
  OpenMP::OpenMP_CXX
)
target_link_libraries(galo_frontend_component
  Eigen3::Eigen
  ${PCL_LIBRARIES}
)
target_link_libraries(deskew_node
  galo_deskew_component
)
target_link_libraries(frontend_node
  galo_frontend_component
)

target_compile_definitions(galo_odometry_component PUBLIC ${PCL_DEFINITIONS})
target_compile_definitions(galo_deskew_component PUBLIC ${PCL_DEFINITIONS})
target_compile_definitions(galo_frontend_component PUBLIC ${PCL_DEFINITIONS})

rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} rosidl_typesupport_cpp)
target_link_libraries(galo_odometry_component "${cpp_typesupport_target}")
target_link_libraries(galo_frontend_component "${cpp_typesupport_target}")

rclcpp_components_register_nodes(
  galo_deskew_component
  "ground_aware_lidar_odometry::GaloDeskewComponent"
)
rclcpp_components_register_nodes(
  galo_frontend_component
  "ground_aware_lidar_odometry::GaloFrontendComponent"
)
rclcpp_components_register_nodes(
  galo_odometry_component
  "GaloOdometryComponent"
)

install(
  TARGETS node deskew_node frontend_node galo_deskew_component galo_frontend_component galo_odometry_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
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

ament_export_dependencies(rosidl_default_runtime)

ament_package()
