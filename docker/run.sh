docker run -it \
  --name ros2-humble-dev \
  --net=host \
  --ipc=host \
  -v ~/dev/lidar-odometry-light/ros2_ws:/ros2_ws \
  ros2-humble-dev