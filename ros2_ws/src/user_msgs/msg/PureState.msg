std_msgs/Header header

geometry_msgs/Pose pose
geometry_msgs/Twist velocity
geometry_msgs/Vector3 linear_acceleration

int8 position_quality
int8 orientation_quality
int8 linear_velocity_quality
int8 angular_velocity_quality
int8 linear_acceleration_quality

int8 POSITION_LOW_QUALITY=10
int8 POSITION_NO_GNSS_QUALITY=11
int8 POSITION_NO_IMU_QUALITY=12
int8 POSITION_HIGH_QUALITY=13

int8 HIGH_QUALITY=1
int8 LOW_QUALITY=0