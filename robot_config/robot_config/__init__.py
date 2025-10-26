"""
Robot Configuration Package
============================

Centralized configuration for the delta robot system.
All parameters are organized into logical modules:

- physical: Robot dimensions, limits, mass properties
- motion: Ruckig, FABRIK, and motion planning parameters
- visualization: RViz markers, colors, sizes
- system: ROS2 frame names, topic names, queue sizes

Usage:
    from robot_config import physical, motion, visualization, system

    # Or import specific values
    from robot_config.physical import NUM_SEGMENTS, ACTUATOR_RADIUS
    from robot_config.motion import FABRIK_TOLERANCE
    from robot_config.visualization import MARKER_TARGET_COLOR
    from robot_config.system import TOPIC_MOTOR_COMMANDS

This replaces the old robot_constants.py file and provides better organization.
"""

# Import all submodules for convenient access
from . import physical
from . import motion
from . import visualization
from . import system

__version__ = '1.0.0'
__all__ = ['physical', 'motion', 'visualization', 'system']
