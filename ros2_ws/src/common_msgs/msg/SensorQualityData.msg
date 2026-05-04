# Time of last message from sensor
float32 last_stamp
# sensor data quality
int8 quality

# Sensors quality constants
# Well work
int8 HIGH_QUALITY=1
# Data from sensor with delay
int8 LOW_QUALITY=0

# Gnss quality
# Sensor works well
int8 HIGH_GNSS_QUALITY=2
# No diff. correction in gnss data
int8 NO_DIFF_CORRECTION_GNSS_QUALITY=1
# Gnss data is delayed
int8 LOW_GNSS_QUALITY=0