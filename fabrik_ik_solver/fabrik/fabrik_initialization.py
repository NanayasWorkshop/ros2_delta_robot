#!/usr/bin/env python3
"""
FABRIK Initialization Module

Generates S-points (segment centers) and J-points (joint positions).
Supports cold start (straight line) and hot start (from robot state).
"""

import numpy as np
import sys
import os
from ament_index_python.packages import get_package_share_directory

# Import config from delta_robot_description installed package
config_path = os.path.join(get_package_share_directory('delta_robot_description'), 'config')
sys.path.insert(0, config_path)
from robot_constants import BASE_HEIGHT, SEGMENT_OFFSET, NUM_SEGMENTS


class FabrikInitialization:
    """Initialization of FABRIK solver with S-points and J-points."""

    SEGMENT_SPACING = 2 * SEGMENT_OFFSET + 2 * BASE_HEIGHT  # 146mm

    @staticmethod
    def create_straight_chain_s_points(num_segments: int = NUM_SEGMENTS) -> np.ndarray:
        """
        Create S-points in straight vertical line (cold start).

        Returns:
            Array of shape (num_segments+1, 3) with S0 to S{num_segments}
        """
        s_points = np.zeros((num_segments + 1, 3), dtype=np.float64)
        s_points[0] = np.array([0.0, 0.0, 0.0])

        for i in range(1, num_segments + 1):
            height = i * FabrikInitialization.SEGMENT_SPACING
            s_points[i] = np.array([0.0, 0.0, height])

        return s_points

    @staticmethod
    def create_s_points_from_joint_states(joint_states: np.ndarray) -> np.ndarray:
        """
        Extract S-points from robot joint states (hot start).

        TODO: Implement using Forward Kinematics.
        """
        raise NotImplementedError("Hot start from joint states requires FK integration")

    @staticmethod
    def validate_s_points(s_points: np.ndarray) -> bool:
        """Validate S-points array."""
        if s_points.ndim != 2 or s_points.shape[1] != 3:
            return False
        if s_points.shape[0] < 2:
            return False
        if not np.all(np.isfinite(s_points)):
            return False
        if not np.allclose(s_points[0], [0, 0, 0], atol=1e-6):
            return False
        z_coords = s_points[:, 2]
        if not np.all(z_coords[1:] >= z_coords[:-1]):
            return False
        return True

    @staticmethod
    def calculate_j_points_from_s_points(s_points: np.ndarray) -> np.ndarray:
        """
        Calculate J-points from S-points using equal length constraint.

        For each segment: dist(S[i-1], J[i]) == dist(J[i], S[i])
        J-point lies on line from S[i-1] in direction (J[i-1] → S[i])

        Args:
            s_points: Array of shape (num_segments+1, 3)

        Returns:
            j_points: Array of shape (num_segments+2, 3) - J0 to J{num_segments+1}
                     For 8 segments: J0 to J9 (10 J-points total)
        """
        num_segments = len(s_points) - 1
        j_points = np.zeros((num_segments + 2, 3), dtype=np.float64)
        j_points[0] = s_points[0]  # J0 = S0

        # Calculate J1 to J8 (intermediate joints)
        for i in range(1, num_segments + 1):
            if i == 1:
                line_start = s_points[0]
                line_direction = np.array([0.0, 0.0, 1.0])
                target_s_point = s_points[1]
            else:
                line_start = s_points[i - 1]
                direction_vector = s_points[i] - j_points[i - 1]
                line_direction = direction_vector / np.linalg.norm(direction_vector)
                target_s_point = s_points[i]

            j_points[i] = FabrikInitialization._find_equal_length_point(
                line_start, line_direction, s_points[i-1], target_s_point
            )

        # J9 = S8 (last J-point coincides with last S-point)
        j_points[num_segments + 1] = s_points[num_segments].copy()

        return j_points

    @staticmethod
    def _find_equal_length_point(line_start: np.ndarray,
                                   line_direction: np.ndarray,
                                   point_a: np.ndarray,
                                   point_b: np.ndarray) -> np.ndarray:
        """
        Find point P on line where: dist(point_a, P) == dist(P, point_b)

        Line: P(t) = line_start + t * line_direction
        Solve: ||P(t) - point_a||² = ||P(t) - point_b||²

        Solution: t = (||line_start - point_a||² - ||line_start - point_b||²) / (2 * d·(point_a - point_b))
        """
        v1 = line_start - point_a
        v2 = line_start - point_b
        v_diff = point_a - point_b

        v1_squared = np.dot(v1, v1)
        v2_squared = np.dot(v2, v2)
        denominator = 2.0 * np.dot(line_direction, v_diff)

        if abs(denominator) < 1e-10:
            # Degenerate case: use midpoint
            midpoint = (point_a + point_b) / 2.0
            v_to_mid = midpoint - line_start
            t = np.dot(v_to_mid, line_direction)
            return line_start + t * line_direction

        t = (v1_squared - v2_squared) / denominator
        return line_start + t * line_direction

    @staticmethod
    def get_segment_spacing() -> float:
        """Get distance between consecutive S-points (0.146m)."""
        return FabrikInitialization.SEGMENT_SPACING

    @staticmethod
    def cold_start(num_segments: int = NUM_SEGMENTS) -> tuple:
        """
        Initialize FABRIK with cold start (straight vertical chain).

        Returns:
            tuple: (s_points, j_points)
                - s_points: Array of shape (num_segments+1, 3)
                - j_points: Array of shape (num_segments+2, 3)
        """
        s_points = FabrikInitialization.create_straight_chain_s_points(num_segments)
        j_points = FabrikInitialization.calculate_j_points_from_s_points(s_points)
        return s_points, j_points

    @staticmethod
    def calculate_joint_distances(j_points: np.ndarray) -> np.ndarray:
        """
        Calculate distances between consecutive J-points.

        Args:
            j_points: Array of shape (num_joints, 3)

        Returns:
            Array of distances of shape (num_joints-1,)
        """
        distances = np.zeros(len(j_points) - 1, dtype=np.float64)
        for i in range(len(j_points) - 1):
            distances[i] = np.linalg.norm(j_points[i+1] - j_points[i])
        return distances
