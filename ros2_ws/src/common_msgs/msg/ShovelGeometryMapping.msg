string object_id
uint8 object_type
uint32[] int_parameters
float32[] float_parameters
string[] string_parameters

uint8 OTHER = 0
uint8 LOADER = 1
uint8 TRUCK = 2
uint8 AREA = 3
uint8 ROAD = 4

uint8 LOAD_ZONE = 0
uint8 UNLOAD_ZONE = 1
uint8 PARKING_ZONE = 2
uint8 CROSSROAD_ZONE = 3
uint8 SAFE_ZONE = 4
uint8 PIT_ZONE = 5
uint8 BLUEPRINT_LOAD_AREA = 6
uint8 DIFFERENCE_LOAD_AREA = 7

# shovel type
uint8 WHEEL = 0
uint8 HYDRO = 1
uint8 DRAGLINE = 2

# road type
uint8 TRAJECTORIES = 0
uint8 MANEUVER_FWD = 1
uint8 MANEUVER_BWD = 2
