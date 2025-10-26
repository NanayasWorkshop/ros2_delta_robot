#!/usr/bin/env python3
"""
FABRIK Cone Constraint Module

Implements the 120° cone constraint for FABRIK iterations.
Prevents unrealistic bending between segments.
"""

import numpy as np

from robot_config import motion as motion_config


class FabrikConeConstraint:
    """Cone constraint for FABRIK solver."""

    # 120° full cone angle = 60° half angle (derived from REVOLUTE_LIMIT)
    CONE_HALF_ANGLE_RAD = motion_config.FABRIK_CONE_HALF_ANGLE  # 2×30° = 60° in radians
    CONE_HALF_ANGLE_DEG = np.degrees(motion_config.FABRIK_CONE_HALF_ANGLE)

    @staticmethod
    def project_onto_cone(direction: np.ndarray,
                          cone_axis: np.ndarray,
                          cone_half_angle_rad: float = CONE_HALF_ANGLE_RAD) -> np.ndarray:
        """
        Project direction vector onto cone surface if it violates constraint.

        The cone constraint ensures the angle between direction and cone_axis
        is at most cone_half_angle_rad (default 60°).

        Args:
            direction: Desired direction vector (any length)
            cone_axis: Cone central axis (will be normalized)
            cone_half_angle_rad: Half-angle of cone in radians (default π/3)

        Returns:
            Corrected direction vector (same magnitude as input if corrected,
            otherwise returns input unchanged)
        """
        # Normalize vectors for angle calculation
        direction_norm = direction / np.linalg.norm(direction)
        cone_axis_norm = cone_axis / np.linalg.norm(cone_axis)

        # Calculate angle between direction and cone axis
        dot_product = np.clip(np.dot(direction_norm, cone_axis_norm), -1.0, 1.0)
        angle = np.arccos(dot_product)

        # Check if inside cone (no correction needed)
        if angle <= cone_half_angle_rad:
            return direction

        # Project onto cone surface
        # Decompose direction into parallel and perpendicular components
        parallel_component = dot_product * cone_axis_norm
        perpendicular_component = direction_norm - parallel_component

        # Normalize perpendicular component
        perp_magnitude = np.linalg.norm(perpendicular_component)
        if perp_magnitude < 1e-10:
            # Direction is exactly along axis, return it
            return direction

        perp_norm = perpendicular_component / perp_magnitude

        # Reconstruct direction on cone surface at exactly cone_half_angle
        cos_angle = np.cos(cone_half_angle_rad)
        sin_angle = np.sin(cone_half_angle_rad)
        corrected_direction_norm = cos_angle * cone_axis_norm + sin_angle * perp_norm

        # Preserve original magnitude
        original_magnitude = np.linalg.norm(direction)
        return corrected_direction_norm * original_magnitude

    @staticmethod
    def get_cone_half_angle_rad() -> float:
        """Get default cone half-angle in radians (60°)."""
        return FabrikConeConstraint.CONE_HALF_ANGLE_RAD

    @staticmethod
    def get_cone_half_angle_deg() -> float:
        """Get default cone half-angle in degrees (60°)."""
        return FabrikConeConstraint.CONE_HALF_ANGLE_DEG
