# Motor command for all 8 segments (24 motors total: 3 per segment)
# Motor positions for prismatic joints (z-axis displacement)

builtin_interfaces/Time timestamp
bool converged                    # Whether IK converged
int32 iterations                  # Number of iterations used
float64 final_error              # Final error in meters

# Motor positions in meters [z_A1, z_B1, z_C1, z_A2, z_B2, z_C2, ..., z_A8, z_B8, z_C8]
float64[] motor_positions        # Length: 24 (8 segments × 3 motors)

# Joint angles: [roll1, pitch1, prism1, roll2, pitch2, prism2, ...]
float64[] joint_angles           # Length: 24 (8 segments × 3 joints)
