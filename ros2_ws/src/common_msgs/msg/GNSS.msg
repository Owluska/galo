std_msgs/Header header
float64 latitude
float64 longitude
float32 height

geometry_msgs/Vector3 velocity

float32 yaw
float32 pitch
float32 roll
bool yaw_valid
bool pitch_valid
bool roll_valid

# Speed over ground (horizontal)
float32 speed
# Movement direction, not useful on low speeds
float32 heading

# Quality according to NMEA-0183 standard
uint8 quality
uint8 satellite_count

# week number since Jan 1980
int16 week_num
# offset builtin_interfaces/Time since Mond of current week in ms
int32 week_time