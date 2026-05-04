std_msgs/Header header
# Pose contains: position and orientation
geometry_msgs/Pose pose
# Velocity can be: linear and angular. Velocity in vehicle frame.
geometry_msgs/Twist velocity
# Linear acceleration in vehicle frame without gravity.
geometry_msgs/Vector3 linear_acceleration

int8 position_quality
int8 orientation_quality
int8 linear_velocity_quality
int8 angular_velocity_quality
int8 linear_acceleration_quality

# Position quality constants
# Bad position quality: no imu and no gnss data sources
int8 POSITION_LOW_QUALITY=10
# Bad position quality: no gnss and has imu
int8 POSITION_NO_GNSS_QUALITY=11
# Bad position quality: has gnss and no imu
int8 POSITION_NO_IMU_QUALITY=12
# Good position quality
int8 POSITION_HIGH_QUALITY=13

# Other quality constants
# Good quality constant
int8 HIGH_QUALITY=1
# Bad quality constant
int8 LOW_QUALITY=0