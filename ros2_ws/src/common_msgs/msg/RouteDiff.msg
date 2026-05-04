std_msgs/Header header
common_msgs/Header info
string direction

# This field defines points number to delete from the previous message.
# For example, if there are 10 points and htis field equals 3, then
# first three points have to be deleted, and fourth (with index 3) point
# will be the first in the route.
# Value -1 means that all points must be deleted (e.g. the route completely changes or has zero points).
int16 first_point

# New points. They have to be added to the end of the route.
geometry_msgs/Point[] added_points

# Hereinafter point indices will refer to new-build route.

# Target speeds for points whose speed has been changed, whether they are new or old.
int16 speed_first_point
int16 speed_last_point
float32[] target_speeds

# Regimes.
RegimeInterval[] regimes