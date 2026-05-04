int8 GEAR_DRIVE = 1
int8 GEAR_NEUTRAL = 0
int8 GEAR_REVERSE = -1

std_msgs/Header header
float32 throttle
float32 dynamic_brake
float32 mechanical_brake
int8 gear
bool parking
