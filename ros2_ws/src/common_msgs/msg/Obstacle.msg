std_msgs/Header header
uint32 id
geometry_msgs/Point position
float32 yaw
float32 length
float32 width
float32 height
float32 distance_to
float32 distance_to_collision
float32 distance_from_trajectory
geometry_msgs/Point[] rectangle_vertices
string type
# obstacle confirmed - 1 (detected in several frames) or detected - 0
uint8 confirmed
# stop type -  0 means no effect on truck, higher values - various stop algorithms
uint8 stop_type
# sensor type of detect
uint8 source
GridFeature grid_feature
geometry_msgs/Point lidar_position

