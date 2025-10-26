"""
Robot dimensional constants for the delta robot.
These values are duplicated from modular_robot.xacro for easy Python access.

MASTER SOURCE: modular_robot.xacro
If you change values, update both files!
"""

import math

# Segment dimensions (in meters)
BASE_HEIGHT = 0.0115        # 11.5mm - height from base to joint1
SEGMENT_OFFSET = 0.0615     # 61.5mm - offset from joint2 to joint3 (and middle to joint5)
PRISMATIC_RANGE = 0.0108    # ±10.8mm - prismatic joint range
REVOLUTE_LIMIT = 0.5236     # ±30° in radians
ACTUATOR_RADIUS = 0.02485   # 24.85mm - radius of actuator triangle for FK/IK calculations

# Derived values
REVOLUTE_LIMIT_DEG = math.degrees(REVOLUTE_LIMIT)  # 30°
TOTAL_SEGMENT_HEIGHT = BASE_HEIGHT + SEGMENT_OFFSET * 2 + BASE_HEIGHT  # ~146mm nominal
FABRIK_CONE_HALF_ANGLE = REVOLUTE_LIMIT * 2  # 60° (2 × 30°) for FABRIK cone constraint

# Robot configuration
NUM_SEGMENTS = 8  # Full robot has 8 segments (can test with 3-8)
JOINTS_PER_SEGMENT = 3  # joint1 (revolute X), joint2 (revolute Y), joint3 (prismatic Z)
NUM_MOTORS = NUM_SEGMENTS * 3  # 24 motors total (3 per segment)

# ============================================================================
# RUCKIG MOTION PLANNING PARAMETERS
# ============================================================================
# These parameters define the physical limits and motion constraints for the
# trajectory smoothing system using Ruckig library.

# Control cycle time for Ruckig updates (in seconds)
# This determines how often new trajectory points are calculated
RUCKIG_DELTA_TIME = 0.01  # 10ms = 100Hz update rate

# Maximum motor velocities (m/s)
# Physical limit of the linear motors
MOTOR_MAX_VELOCITY = 0.002  # 2 mm/s = 0.002 m/s

# Maximum motor accelerations (m/s²)
# Physical acceleration limit
MOTOR_MAX_ACCELERATION = 0.012  # 12 mm/s² = 0.012 m/s²

# Maximum motor jerk (m/s³)
# Rate of change of acceleration - limits mechanical shock
MOTOR_MAX_JERK = 0.032  # 32 mm/s³ = 0.032 m/s³

# Ruckig synchronization mode
# 'time': All motors reach target simultaneously (FABRIK solutions hit at same time)
RUCKIG_SYNCHRONIZATION = 'time'

# Ruckig control interface
# 'position': Full control over position, velocity, acceleration
RUCKIG_CONTROL_INTERFACE = 'position'

# Intermediate waypoint behavior
# For intermediate waypoints: use non-zero velocity to pass through without stopping
# For final waypoint: use zero velocity to come to complete stop
# This is handled dynamically in the trajectory smoother node:
#   - Intermediate waypoints → target_velocity = MOTOR_MAX_VELOCITY (pass through fast)
#   - Final waypoint → target_velocity = 0.0 (stop)
RUCKIG_INTERMEDIATE_TARGET_VELOCITY = None  # Will be set to MOTOR_MAX_VELOCITY in code
RUCKIG_FINAL_TARGET_VELOCITY = 0.0  # Always stop at final waypoint
RUCKIG_TARGET_ACCELERATION = 0.0  # Always zero acceleration at waypoints




