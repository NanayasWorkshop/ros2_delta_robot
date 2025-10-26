#!/usr/bin/env python3
"""
FABRIK Solver - Master Orchestrator
====================================
Main solver class that orchestrates the complete FABRIK IK solving process.
Encapsulates initialization, iteration loop, convergence checking, and motor calculation.
"""

import numpy as np
import warnings
from typing import Tuple, Dict

from .fabrik_initialization import FabrikInitialization
from .fabrik_iteration import FabrikIteration


class FabrikSolver:
    """
    Master FABRIK solver that orchestrates the complete IK solving process.

    This class encapsulates:
    - Initialization (cold/hot start)
    - Iteration loop with convergence checking
    - Motor position calculation from solved S-points
    """

    def __init__(self, num_segments: int, default_tolerance: float = 0.001,
                 default_max_iterations: int = 50):
        """
        Initialize FABRIK solver.

        Args:
            num_segments: Number of segments in the robot
            default_tolerance: Default convergence tolerance in meters (default 1mm)
            default_max_iterations: Default maximum iterations (default 50)
        """
        self.num_segments = num_segments
        self.default_tolerance = default_tolerance
        self.default_max_iterations = default_max_iterations

        # State for hot start
        self.last_s_points = None
        self.last_j_points = None
        self.has_previous_solution = False

    def solve(self,
              target: np.ndarray,
              approach_point: np.ndarray = None,
              use_hot_start: bool = False,
              tolerance: float = None,
              max_iterations: int = None) -> Dict:
        """
        Solve inverse kinematics using FABRIK algorithm.

        Args:
            target: Target end-effector position [x, y, z] in meters
            approach_point: Optional approach point for J7 direction control [x, y, z] in meters
            use_hot_start: Whether to use previous solution as starting point
            tolerance: Convergence tolerance in meters (uses default if None)
            max_iterations: Maximum iterations (uses default if None)

        Returns:
            Dictionary containing:
                - 'converged': bool - Whether IK converged
                - 's_points': np.ndarray - Final S-point positions
                - 'j_points': np.ndarray - Final J-point positions
                - 'iterations': int - Number of iterations used
                - 'final_error': float - Final error in meters
                - 'motor_positions': list - Motor positions [z_A1, z_B1, z_C1, z_A2, ...] in meters
                - 'joint_angles': list - Joint angles [roll1, pitch1, prism1, roll2, ...] in radians/meters
        """
        # Use default parameters if not provided
        if tolerance is None:
            tolerance = self.default_tolerance
        if max_iterations is None:
            max_iterations = self.default_max_iterations

        # Initialize FABRIK with hot or cold start
        if use_hot_start and self.has_previous_solution:
            s_initial = self.last_s_points.copy()
            j_initial = self.last_j_points.copy()
        else:
            s_initial, j_initial = FabrikInitialization.cold_start(self.num_segments)

        # Calculate initial joint distances
        joint_distances = FabrikInitialization.calculate_joint_distances(j_initial)

        # Run FABRIK iteration loop
        s_current = s_initial.copy()
        j_current = j_initial.copy()
        iterations_used = 0
        final_error = np.linalg.norm(j_current[-1] - target)

        for iteration in range(max_iterations):
            # Check convergence
            current_error = np.linalg.norm(j_current[-1] - target)
            if current_error <= tolerance:
                iterations_used = iteration
                final_error = current_error
                break

            # FABRIK iteration with approach point
            j_current = FabrikIteration.iterate_once(
                j_current,
                target,
                j_initial,
                joint_distances,
                iteration,
                use_relaxation=True,
                approach_point=approach_point
            )

            # Recalculate S-points and segment lengths
            s_current = FabrikIteration.recalculate_s_points_from_j_points(j_current)
            joint_distances = FabrikIteration.calculate_new_segment_lengths(s_current)

            iterations_used = iteration + 1
            final_error = current_error

        # Store for next hot start
        self.last_s_points = s_current.copy()
        self.last_j_points = j_current.copy()
        self.has_previous_solution = True

        # Calculate motor positions and joint angles from S-points
        motor_positions, joint_angles = self._calculate_motors_from_s_points(s_current)

        # Determine convergence
        converged = final_error <= tolerance

        return {
            'converged': converged,
            's_points': s_current,
            'j_points': j_current,
            'iterations': iterations_used,
            'final_error': final_error,
            'motor_positions': motor_positions,
            'joint_angles': joint_angles
        }

    def _calculate_motors_from_s_points(self, s_points: np.ndarray) -> Tuple[list, list]:
        """
        Calculate motor positions and joint angles from S-points.

        Args:
            s_points: Array of S-point positions (num_segments+1, 3) in meters

        Returns:
            Tuple of (motor_positions, joint_angles):
                - motor_positions: List of motor z positions [z_A1, z_B1, z_C1, z_A2, ...] in meters
                - joint_angles: List of joint values [roll1, pitch1, prism1, roll2, ...]
                               in radians for revolute joints, meters for prismatic
        """
        # Import here to avoid circular dependency
        from .fabrik_kinematics import calculate_motors_for_segments

        motor_positions = []
        joint_angles = []

        # Convert S-points to mm and extract segment positions (skip S0)
        segment_positions_mm = [s_points[i] * 1000 for i in range(1, len(s_points))]

        # Use hierarchical transformation for multi-segment kinematics
        with warnings.catch_warnings():
            warnings.simplefilter("ignore")  # Suppress motor clipping warnings

            levels = calculate_motors_for_segments(segment_positions_mm)

            # Extract results for each level/segment
            for level_data in levels:
                # Motor positions (convert from mm to meters)
                motor_positions.extend([
                    level_data['z_A'] / 1000.0,
                    level_data['z_B'] / 1000.0,
                    level_data['z_C'] / 1000.0
                ])

                # Joint angles: roll, pitch, prismatic (convert degrees to radians)
                import math
                joint_angles.extend([
                    math.radians(level_data['roll_joint']),      # convert to radians
                    math.radians(level_data['pitch_joint']),     # convert to radians
                    level_data['prismatic_joint'] / 1000.0  # convert mm to meters
                ])

        return motor_positions, joint_angles

    def reset_hot_start(self):
        """Reset hot start state (force next solve to use cold start)."""
        self.last_s_points = None
        self.last_j_points = None
        self.has_previous_solution = False

    def get_segment_spacing(self) -> float:
        """Get nominal spacing between segments in meters."""
        return FabrikInitialization.get_segment_spacing()
