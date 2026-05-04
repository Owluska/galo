int32 MAX_BRAKE = 100
int32 MAX_HYDRAULIC_BRAKE = 100
int32 GEAR_NEUTRAL = 0
int32 GEAR_FORWARD = 1
int32 GEAR_BACKWARD = -1

std_msgs/Header header
int32 throttle
int32 brake
int32 hydraulic_brake
int32 gear
bool parking
bool loading_brake
