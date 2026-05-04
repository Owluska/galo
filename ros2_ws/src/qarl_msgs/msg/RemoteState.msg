int8 PLATFORM_POSITION_LOWER = -1
int8 PLATFORM_POSITION_INTERMEDIATE = 0
int8 PLATFORM_POSITION_UPPER = 1
int8 PLATFORM_MODE_UP = -1
int8 PLATFORM_MODE_HOLD = 0
int8 PLATFORM_MODE_DOWN = 1

uint8 MODE_OFF = 0
uint8 MODE_AUTONOMOUS = 1
uint8 MODE_OPERATOR_SUPERVISION = 2
uint8 MODE_OPERATOR_CONTROL = 3

std_msgs/Header header
common_msgs/Header info
uint8 iplace_id

float32 speed
geometry_msgs/Point position
geometry_msgs/Twist velocity
geometry_msgs/Vector3 linear_acceleration
float32 yaw
float32 pitch
float32 roll
float32 wheel_angle
int8 quality
uint8 status
uint8 vehicle_state

float32 slope

int8 position_quality
int8 orientation_quality
int8 linear_velocity_quality
int8 angular_velocity_quality
int8 linear_acceleration_quality

uint8 accel
uint8 electro_brakes
uint8 work_brakes
int8 gear
bool parking
bool loading_brake

float32 platform_position
int8 platform_mode

float32 steering_wheel_position
float32 oil_pressure
float32 fuel_level
uint32 weight

common_msgs/ObstacleArray obstacle_array
