"""
Visualization Parameters
========================
Parameters for RViz markers, colors, sizes, and transparency.
"""

# =============================================================================
# INTERACTIVE MARKERS
# =============================================================================

# Target Marker
MARKER_TARGET_POSITION = [0.150, 0.0, 0.600]
"""Initial position for target marker (150mm, 0mm, 600mm)"""

MARKER_TARGET_COLOR = [1.0, 1.0, 0.0]
"""Target marker color RGB (yellow)"""

MARKER_TARGET_SIZE = 0.06
"""Target marker diameter (60mm)"""

# Direction Marker
MARKER_DIRECTION_POSITION = [0.0, 0.0, 0.600]
"""Initial position for direction marker (0mm, 0mm, 600mm)"""

MARKER_DIRECTION_COLOR = [0.0, 1.0, 1.0]
"""Direction marker color RGB (cyan)"""

MARKER_DIRECTION_SIZE = 0.04
"""Direction marker diameter (40mm)"""

# Marker Properties
MARKER_ALPHA = 0.8
"""Marker transparency (0.0 = transparent, 1.0 = opaque)"""

MARKER_CONTROL_SCALE = 0.15
"""Size of interactive marker control handles"""

MARKER_TF_BROADCAST_RATE = 0.1
"""TF broadcast rate for markers in seconds (10Hz)"""

# =============================================================================
# FABRIK VISUALIZATION
# =============================================================================

# S-Points (blue spheres)
FABRIK_S_POINT_SIZE = 0.030
"""S-point sphere diameter (30mm)"""

FABRIK_S_POINT_COLOR = [0.0, 0.0, 1.0]
"""S-point color RGB (blue)"""

FABRIK_S_POINT_ALPHA = 0.8
"""S-point transparency"""

# J-Points (green spheres)
FABRIK_J_POINT_SIZE = 0.030
"""J-point sphere diameter (30mm)"""

FABRIK_J_POINT_COLOR = [0.0, 1.0, 0.0]
"""J-point color RGB (green)"""

FABRIK_J_POINT_ALPHA = 0.8
"""J-point transparency"""

# Chain Lines
FABRIK_S_CHAIN_LINE_WIDTH = 0.005
"""S-point chain line width (5mm)"""

FABRIK_J_CHAIN_LINE_WIDTH = 0.005
"""J-point chain line width (5mm)"""

FABRIK_S_CHAIN_COLOR = [0.0, 0.0, 1.0]
"""S-chain line color RGB (blue)"""

FABRIK_J_CHAIN_COLOR = [0.0, 1.0, 0.0]
"""J-chain line color RGB (green)"""

FABRIK_CHAIN_ALPHA = 0.6
"""Chain line transparency"""

# =============================================================================
# URDF/ROBOT MODEL
# =============================================================================

URDF_MESH_SCALE = [0.001, 0.001, 0.001]
"""Mesh scale factors (STL files are in mm, convert to meters)"""

URDF_BASE_COLOR = [0.5, 0.5, 0.5, 1.0]
"""Base link color RGBA (gray)"""

URDF_MIDDLE_COLOR = [0.0, 0.0, 1.0, 1.0]
"""Middle link color RGBA (blue)"""

URDF_LINK_COLOR = [0.7, 0.7, 0.7, 1.0]
"""Default link color RGBA (light gray)"""
