std_msgs/Header header
common_msgs/Header info
float32 accel_angle  # angle on which accelation pedal pushed
float32 brake_angle  # angle on which brake pedal pushed
float32 hydraulic_brake_angle  # angle on which hydraulic brake pedal pushed
float32 wheel_angle  # angle from zero position of steer wheel
uint8 parking
int8 gear
# mode 0 - under steerage
# mode 1 - remote control
# mode 2 - autonomous
uint8 mode
