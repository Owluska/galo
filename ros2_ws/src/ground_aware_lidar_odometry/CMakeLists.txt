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
find_package(sensor_msgs REQUIRED)
find_package(user_msgs REQUIRED)

set(LIB_SOURCES
  src/library/deskew.cpp
  src/library/segmentation.cpp
  src/library/utils.cpp

)
add_executable(node src/node.cpp ${LIB_SOURCES})
ament_target_dependencies(node rclcpp std_msgs sensor_msgs user_msgs)
target_link_libraries(node
  Eigen3::Eigen
)
target_include_directories(node PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    ${EIGEN3_INCLUDE_DIRS}
  )

install(TARGETS
  node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include/
  DESTINATION include)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()