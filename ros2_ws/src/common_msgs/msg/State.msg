std_msgs/Header header
common_msgs/Header info
float32 speed
geometry_msgs/Point position
float32 yaw
float32 wheel_angle
uint8 quality
uint64 distance
uint8 status
int8 direction
uint32 weight
geometry_msgs/Point correction
common_msgs/RegimeChangeReason reason

uint8 MODE_OFF = 0
uint8 MODE_AUTONOMOUS = 1
uint8 MODE_OPERATOR_SUPERVISION = 2
uint8 MODE_OPERATOR_CONTROL = 3

int8 DIR_FORWARD = 1
int8 DIR_BACKWARD = -1
int8 IDLING = 0
