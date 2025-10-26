"""
Motion Planning Parameters
===========================
Parameters for trajectory smoothing, inverse kinematics, and motion control.
"""

# =============================================================================
# RUCKIG MOTION PLANNING
# =============================================================================

RUCKIG_DELTA_TIME = 0.01
"""Control cycle time for Ruckig updates (10ms = 100Hz)"""

MOTOR_MAX_VELOCITY = 0.002
"""Maximum motor velocity (2 mm/s = 0.002 m/s)"""

MOTOR_MAX_ACCELERATION = 0.064
"""Maximum motor acceleration (64 mm/s² = 0.064 m/s²)"""

MOTOR_MAX_JERK = 0.516
"""Maximum motor jerk - rate of change of acceleration (516 mm/s³ = 0.516 m/s³)"""

RUCKIG_SYNCHRONIZATION = 'time'
"""Synchronization mode: 'time' = all motors reach target simultaneously"""

RUCKIG_CONTROL_INTERFACE = 'position'
"""Control interface: 'position' = full control over position, velocity, acceleration"""

RUCKIG_INTERMEDIATE_TARGET_VELOCITY = None
"""Target velocity for intermediate waypoints (set to MOTOR_MAX_VELOCITY at runtime)"""

RUCKIG_FINAL_TARGET_VELOCITY = 0.0
"""Target velocity for final waypoint (always stop)"""

RUCKIG_TARGET_ACCELERATION = 0.0
"""Target acceleration at waypoints (always zero for smooth arrival)"""

RUCKIG_MAX_LOOKAHEAD_WAYPOINTS = 10
"""Maximum number of waypoints to look ahead for smooth trajectory planning"""

RUCKIG_PUBLISH_RATE = 100.0
"""Trajectory publishing rate in Hz"""

RUCKIG_INITIAL_VELOCITY = 0.0
"""Initial velocity when starting from rest"""

RUCKIG_INITIAL_ACCELERATION = 0.0
"""Initial acceleration when starting from rest"""

# =============================================================================
# FABRIK IK SOLVER
# =============================================================================

FABRIK_TOLERANCE = 0.001
"""Convergence tolerance in meters (1mm)"""

FABRIK_MAX_ITERATIONS = 50
"""Maximum FABRIK iterations before giving up"""

FABRIK_APPROACH_CHANGE_THRESHOLD = 1e-6
"""Threshold to detect approach vector changes (meters)"""

FABRIK_TARGET_OFFSET_ON_APPROACH_CHANGE = 1e-4
"""Small offset added to target when approach changes (0.1mm)"""

FABRIK_ENABLE_VISUALIZATION = True
"""Enable publishing of FABRIK visualization markers"""

FABRIK_USE_HOT_START = True
"""Use previous solution as starting point for next iteration"""

# =============================================================================
# MOTOR TO JOINT CONVERSION
# =============================================================================

MOTOR_TO_JOINT_DIVISION_EPSILON = 1e-10
"""Small value to prevent division by zero in Fermat point calculation"""

MOTOR_TO_JOINT_PRISMATIC_MULTIPLIER = 2.0
"""Multiplier to convert Fermat point Z to prismatic joint extension"""

MOTOR_TO_JOINT_FERMAT_ANGLE_OFFSET = 1.0471975511965976  # math.pi / 3
"""Angle offset for Fermat point calculation (60° = π/3 radians)"""
