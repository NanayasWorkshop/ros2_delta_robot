"""
FABRIK Inverse Kinematics Module

Forward And Backward Reaching Inverse Kinematics solver for the 8-segment delta robot.

Modules:
    - fabrik_solver: Main solver orchestrator (use this for IK solving)
    - fabrik_initialization: S-point and J-point generation (cold/hot start)
    - fabrik_iteration: Single FABRIK iteration (backward + forward pass)
    - fabrik_constraints: Cone constraints for joint angle limits
    - fabrik_kinematics: Segment position to motor/joint calculations
"""

from .fabrik_solver import FabrikSolver
from .fabrik_initialization import FabrikInitialization
from .fabrik_iteration import FabrikIteration
from .fabrik_constraints import FabrikConeConstraint
from .fabrik_kinematics import calculate_kinematics

__all__ = [
    'FabrikSolver',
    'FabrikInitialization',
    'FabrikIteration',
    'FabrikConeConstraint',
    'calculate_kinematics'
]
