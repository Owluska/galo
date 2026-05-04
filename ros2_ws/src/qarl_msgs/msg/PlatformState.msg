string RELATIVE_POSITION_UNKNOWN=UNKNOWN
string RELATIVE_POSITION_LOWER=LOWER
string RELATIVE_POSITION_UPPER=UPPER
string RELATIVE_POSITION_INTERMEDIATE=INTERMEDIATE

std_msgs/Header header
string relative_position
float32 absolute_position
int8 quality

# Low quality level when something wrong happend with platform absolute position. 
# Example: not moving when mode MOVE_DOWN or has big jumps when in mode HOLD
int8 QUALITY_LOW=0
# Quality with minor deviation. Use when relative position in LOW, control mode HOLD, 
# but absolute platform position not equals 0 but in permitted bound
int8 QUALITY_MINOR_DEVIATION=1
int8 QUALITY_HIGH=2
int8 QUALITY_NO_INFORMATION=10
