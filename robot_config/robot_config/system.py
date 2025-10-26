"""
System Parameters
=================
ROS2 system configuration: frame names, queue sizes, rates, tolerances.
"""

# =============================================================================
# TF FRAME NAMES
# =============================================================================

FRAME_MAP = 'map'
"""Root TF frame"""

FRAME_WORLD = 'world'
"""World reference frame"""

FRAME_BASE_LINK = 'base_link'
"""Robot base link frame"""

FRAME_TARGET = 'target'
"""Target marker frame"""

FRAME_DIRECTION = 'direction'
"""Direction marker frame"""

# =============================================================================
# TRAJECTORY TRACKING
# =============================================================================

TRAJECTORY_TRACKED_FRAMES = ['target', 'direction']
"""List of TF frames to track"""

TRAJECTORY_REFERENCE_FRAME = 'world'
"""Reference frame for trajectory recording"""

TRAJECTORY_SAMPLE_RATE = 1.0
"""Trajectory sampling rate in Hz"""

TRAJECTORY_POSITION_TOLERANCE = 1e-6
"""Minimum position change to record new trajectory point (meters)"""

TRAJECTORY_WARNING_THROTTLE = 1.0
"""Seconds between repeated warning messages"""

# =============================================================================
# ROS2 QUEUE SIZES
# =============================================================================

QUEUE_SIZE_DEFAULT = 10
"""Default queue size for all publishers and subscribers"""

QUEUE_SIZE_LARGE = 100
"""Large queue size for high-frequency topics"""

QUEUE_SIZE_SMALL = 5
"""Small queue size for low-frequency topics"""

# =============================================================================
# STATIC TRANSFORMS
# =============================================================================

STATIC_TF_MAP_TO_WORLD = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
"""Static transform from map to world (x, y, z, roll, pitch, yaw)"""

STATIC_TF_WORLD_TO_BASE = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
"""Static transform from world to base_link (x, y, z, roll, pitch, yaw)"""

# =============================================================================
# TOPIC NAMES
# =============================================================================

TOPIC_TRAJECTORY = '/trajectory'
"""Trajectory data topic"""

TOPIC_TRAJECTORY_STATUS = '/trajectory_status'
"""Trajectory point count topic"""

TOPIC_CLEAR_TRAJECTORY = '/clear_trajectory'
"""Clear all trajectory points topic"""

TOPIC_CLEAR_OLDEST = '/clear_oldest'
"""Remove oldest N trajectory points topic"""

TOPIC_MOTOR_COMMANDS = '/motor_commands'
"""Raw motor commands from FABRIK"""

TOPIC_SMOOTHED_MOTOR_COMMANDS = '/smoothed_motor_commands'
"""Smoothed motor commands from Ruckig"""

TOPIC_JOINT_STATES = '/joint_states'
"""Joint states for robot_state_publisher"""

TOPIC_FABRIK_VISUALIZATION = '/fabrik_visualization'
"""FABRIK visualization data"""

TOPIC_FABRIK_MARKERS = '/fabrik_markers'
"""FABRIK RViz markers"""

TOPIC_FABRIK_MOTOR_POSITIONS = '/fabrik/motor_positions'
"""Raw FABRIK motor positions for plotting"""

TOPIC_RUCKIG_CURRENT_POSITION = '/ruckig/current_position'
"""Ruckig current position for plotting"""

TOPIC_RUCKIG_CURRENT_VELOCITY = '/ruckig/current_velocity'
"""Ruckig current velocity for plotting"""

TOPIC_RUCKIG_CURRENT_ACCELERATION = '/ruckig/current_acceleration'
"""Ruckig current acceleration for plotting"""

TOPIC_RUCKIG_TARGET_POSITION = '/ruckig/target_position'
"""Ruckig target position for plotting"""

TOPIC_RUCKIG_BUFFER_SIZE = '/ruckig/buffer_size'
"""Ruckig waypoint buffer size"""
