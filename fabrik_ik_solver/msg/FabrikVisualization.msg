# FABRIK visualization data for J-points and S-points

builtin_interfaces/Time timestamp

# S-points: Segment base positions (9 points for 8 segments: S0 to S8)
geometry_msgs/Point[] s_points

# J-points: Joint positions (9 points for 8 segments: J0 to J8)
geometry_msgs/Point[] j_points

# Target that was solved for
geometry_msgs/Point target

# Approach point (if used)
geometry_msgs/Point approach_point
bool has_approach_point
