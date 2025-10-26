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
