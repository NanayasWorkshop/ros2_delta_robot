"""
Robot Physical Parameters
=========================
Physical dimensions and limits of the delta robot hardware.

This is the MASTER SOURCE for all physical robot parameters.
URDF, kinematics, and control nodes should read from here.
"""

import math

# =============================================================================
# SEGMENT DIMENSIONS (meters)
# =============================================================================

BASE_HEIGHT = 0.0115
"""Height from base to joint1 (11.5mm)"""

SEGMENT_OFFSET = 0.0615
"""Offset from joint2 to joint3, and middle to joint5 (61.5mm)"""

PRISMATIC_RANGE = 0.0108
"""Maximum prismatic joint travel (±10.8mm)"""

REVOLUTE_LIMIT = 0.5236
"""Maximum revolute joint angle (±30° in radians)"""

ACTUATOR_RADIUS = 0.02485
"""Radius of actuator triangle for FK/IK calculations (24.85mm)"""

# =============================================================================
# MOTOR BASE POSITIONS (angles in radians)
# =============================================================================

MOTOR_A_ANGLE = math.pi / 2.0
"""Motor A position: 90° (top, +Y direction)"""

MOTOR_B_ANGLE = -math.pi / 6.0
"""Motor B position: -30° = 330° (bottom right)"""

MOTOR_C_ANGLE = -5.0 * math.pi / 6.0
"""Motor C position: -150° = 210° (bottom left)"""

# =============================================================================
# DERIVED VALUES
# =============================================================================

REVOLUTE_LIMIT_DEG = math.degrees(REVOLUTE_LIMIT)
"""Revolute limit in degrees (30°)"""

TOTAL_SEGMENT_HEIGHT = BASE_HEIGHT + SEGMENT_OFFSET * 2 + BASE_HEIGHT  # ~146mm nominal
"""Total height of one segment including base heights and offsets"""

FABRIK_CONE_HALF_ANGLE = REVOLUTE_LIMIT * 2
"""FABRIK cone constraint: 60° half-angle (2 × 30°)"""

# =============================================================================
# ROBOT CONFIGURATION
# =============================================================================

NUM_SEGMENTS = 8
"""Number of segments in the robot"""

JOINTS_PER_SEGMENT = 3
"""Joints per segment: joint1 (revolute X), joint2 (revolute Y), joint3 (prismatic Z)"""

NUM_MOTORS = NUM_SEGMENTS * 3
"""Total number of motors (24 = 8 segments × 3 motors)"""

# =============================================================================
# MASS PROPERTIES (kg)
# =============================================================================

BASE_MASS = 0.45
"""Mass of segment base link"""

LINK_MASS = 0.1
"""Mass of individual links (link1-6)"""

MIDDLE_MASS = 1.26
"""Mass of middle platform link"""

# =============================================================================
# INERTIA VALUES
# =============================================================================

BASE_INERTIA = 0.003
"""Inertia tensor diagonal values for base link"""

LINK_INERTIA = 0.001
"""Inertia tensor diagonal values for individual links"""

MIDDLE_INERTIA = 0.010
"""Inertia tensor diagonal values for middle link"""
