#!/usr/bin/env python3
"""
FABRIK Iteration Module

Implements backward and forward passes for FABRIK IK solver.
"""

import numpy as np

from .fabrik_constraints import FabrikConeConstraint
from .fabrik_kinematics import calculate_kinematics
from robot_config import motion as motion_config


class FabrikIteration:
    """FABRIK iteration with backward and forward passes."""

    # Default cone constraint settings (derived from REVOLUTE_LIMIT)
    FULL_CONE_HALF_ANGLE_RAD = motion_config.FABRIK_CONE_HALF_ANGLE  # 2×30° = 60° (full constraint)
    INITIAL_CONE_HALF_ANGLE_RAD = motion_config.FABRIK_CONE_HALF_ANGLE / 2.0  # 30° (half of full, tight start)
    RELAXATION_ITERATIONS = 20  # Iterations to reach full cone

    @staticmethod
    def calculate_cone_angle_for_iteration(iteration: int,
                                           relaxation_iterations: int = 20,
                                           initial_angle: float = None,
                                           full_angle: float = None) -> float:
        """
        Calculate cone half-angle for current iteration with linear relaxation.

        Args:
            iteration: Current iteration number (0-based)
            relaxation_iterations: Number of iterations to reach full cone (default 20)
            initial_angle: Starting cone half-angle in radians (default π/6 = 30°)
            full_angle: Final cone half-angle in radians (default π/3 = 60°)

        Returns:
            Cone half-angle in radians for this iteration
        """
        if initial_angle is None:
            initial_angle = FabrikIteration.INITIAL_CONE_HALF_ANGLE_RAD
        if full_angle is None:
            full_angle = FabrikIteration.FULL_CONE_HALF_ANGLE_RAD

        if iteration >= relaxation_iterations:
            return full_angle

        # Linear interpolation from initial to full
        progress = iteration / relaxation_iterations
        cone_angle = initial_angle + (full_angle - initial_angle) * progress
        return cone_angle

    @staticmethod
    def calculate_approach_cone_angle(target: np.ndarray,
                                      approach_point: np.ndarray,
                                      min_distance: float = 0.005,
                                      max_distance: float = 0.095) -> tuple:
        """
        Calculate cone angle for J7 based on approach point position.

        Three cases:
        1. Distance < 5mm: NO constraint (disable)
        2. Distance 5-95mm: CONE constraint (linear interpolation)
        3. Distance >= 95mm: LINE constraint (straight line)

        Args:
            target: Target position (cone apex)
            approach_point: Approach point position (defines cone opening direction)
            min_distance: Distance below which constraint is disabled (default 5mm = 0.005m)
            max_distance: Distance above which it becomes line constraint (default 95mm = 0.095m)

        Returns:
            tuple: (cone_angle_rad, constraint_type)
                   constraint_type: 'none', 'cone', or 'line'
        """
        distance = np.linalg.norm(approach_point - target)

        # Case 1: No constraint (< 5mm)
        if distance < min_distance:
            return None, 'none'

        # Case 3: Line constraint (>= 95mm)
        if distance >= max_distance:
            return np.radians(0.01), 'line'  # Nearly 0° for numerical stability

        # Case 2: Cone constraint (5mm - 95mm)
        # Linear interpolation: 5mm → 90°, 95mm → 0°
        # angle = 90° * (1 - (d - 5) / (95 - 5))
        normalized_distance = (distance - min_distance) / (max_distance - min_distance)
        cone_half_angle_deg = 90.0 * (1.0 - normalized_distance)
        return np.radians(cone_half_angle_deg), 'cone'

    @staticmethod
    def backward_pass(joints: np.ndarray,
                      target: np.ndarray,
                      joint_distances: np.ndarray,
                      cone_half_angle_rad: float = np.pi / 3.0,
                      approach_point: np.ndarray = None) -> np.ndarray:
        """
        Perform backward pass: pull joints toward target from end-effector to base.

        Args:
            joints: Current joint positions, shape (num_joints, 3)
            target: Target end-effector position, shape (3,)
            joint_distances: Distances between consecutive joints, shape (num_joints-1,)
            cone_half_angle_rad: Cone constraint half-angle (default 60°)
            approach_point: Optional approach point position for J7 approach direction control

        Returns:
            b_joints: Joint positions after backward pass, shape (num_joints, 3)
        """
        num_joints = len(joints)
        b_joints = joints.copy()

        # Step 1: Move end-effector to target
        b_joints[-1] = target.copy()

        # Step 2: Loop backward from second-to-last joint to base
        for i in range(num_joints - 2, -1, -1):
            if i == num_joints - 2:
                # First joint back (e.g., J7 in 8-segment robot)
                # SPECIAL: Apply approach point cone constraint if provided
                if approach_point is not None:
                    approach_cone_angle, constraint_type = FabrikIteration.calculate_approach_cone_angle(
                        target, approach_point
                    )

                    if constraint_type == 'none':
                        # Case 1: No constraint (< 5mm)
                        direction = joints[i] - b_joints[i + 1]
                        direction_norm = direction / np.linalg.norm(direction)
                        b_joints[i] = b_joints[i + 1] + direction_norm * joint_distances[i]

                    elif constraint_type == 'line':
                        # Case 3: Line constraint (>= 95mm) - must be exactly on line
                        cone_apex = b_joints[i + 1]  # Target position
                        line_direction = approach_point - target  # Direction toward approach point
                        line_direction_norm = line_direction / np.linalg.norm(line_direction)
                        b_joints[i] = cone_apex + line_direction_norm * joint_distances[i]

                    else:  # constraint_type == 'cone'
                        # Case 2: Cone constraint (5-95mm)
                        cone_apex = b_joints[i + 1]  # Target position
                        cone_axis = approach_point - target  # Points toward approach point

                        # Desired direction from apex toward original joint position
                        desired_direction = joints[i] - cone_apex

                        # Apply cone constraint
                        corrected_direction = FabrikConeConstraint.project_onto_cone(
                            desired_direction, cone_axis, approach_cone_angle
                        )

                        # Place joint at fixed distance from apex
                        corrected_norm = corrected_direction / np.linalg.norm(corrected_direction)
                        b_joints[i] = cone_apex + corrected_norm * joint_distances[i]
                else:
                    # No approach point - no constraint (original behavior)
                    direction = joints[i] - b_joints[i + 1]
                    direction_norm = direction / np.linalg.norm(direction)
                    b_joints[i] = b_joints[i + 1] + direction_norm * joint_distances[i]

            else:
                # Middle joints (e.g., J6, J5, ..., J0)
                # APPLY CONE CONSTRAINT
                cone_apex = b_joints[i + 1]
                cone_axis = b_joints[i + 1] - b_joints[i + 2]

                # Desired direction from apex toward original joint position
                desired_direction = joints[i] - cone_apex

                # Apply cone constraint
                corrected_direction = FabrikConeConstraint.project_onto_cone(
                    desired_direction, cone_axis, cone_half_angle_rad
                )

                # Place joint at fixed distance from apex
                corrected_norm = corrected_direction / np.linalg.norm(corrected_direction)
                b_joints[i] = cone_apex + corrected_norm * joint_distances[i]

        return b_joints

    @staticmethod
    def iterate_once(joints: np.ndarray,
                     target: np.ndarray,
                     original_joints: np.ndarray,
                     joint_distances: np.ndarray,
                     iteration_number: int = 0,
                     use_relaxation: bool = True,
                     approach_point: np.ndarray = None) -> np.ndarray:
        """
        Perform one complete FABRIK iteration (backward + forward) with optional cone relaxation.

        Args:
            joints: Current joint positions
            target: Target position
            original_joints: Original joint positions (for forward pass base fixing)
            joint_distances: Fixed distances between joints
            iteration_number: Current iteration number (0-based)
            use_relaxation: If True, use progressive cone relaxation
            approach_point: Optional approach point for approach direction control

        Returns:
            Updated joint positions after one iteration
        """
        if use_relaxation:
            cone_angle = FabrikIteration.calculate_cone_angle_for_iteration(iteration_number)
        else:
            cone_angle = FabrikIteration.FULL_CONE_HALF_ANGLE_RAD

        # Backward pass (with optional approach point)
        j_backward = FabrikIteration.backward_pass(joints, target, joint_distances, cone_angle, approach_point)

        # Forward pass
        j_forward = FabrikIteration.forward_pass(j_backward, original_joints, joint_distances, cone_angle)

        return j_forward

    @staticmethod
    def recalculate_s_points_from_j_points(j_points: np.ndarray) -> np.ndarray:
        """
        Recalculate S-points from J-points after forward iteration.

        This updates S-points based on actual joint positions to reflect
        the changing segment lengths in the physical robot.

        Pattern:
        - S0 = J0 (always at origin)
        - S1: dist = ||J1 - J0||, direction = normalize(J2 - J1), S1 = J1 + direction * dist
        - S[i] (i >= 2): dist = ||J[i] - S[i-1]||, direction = normalize(J[i+1] - J[i]), S[i] = J[i] + direction * dist
        - S[last] = J[last] (end-effector)

        Args:
            j_points: Joint positions, shape (num_joints, 3)

        Returns:
            s_points: Recalculated S-points, shape (num_segments+1, 3)
                     For 10 J-points (J0-J9): returns 9 S-points (S0-S8)
        """
        num_joints = len(j_points)
        num_segments = num_joints - 2  # 10 J-points → 8 segments → 9 S-points
        s_points = np.zeros((num_segments + 1, 3), dtype=np.float64)

        # S0 = J0 (always at origin)
        s_points[0] = j_points[0].copy()

        # S1: Special case - S1 = J1 + normalize(J2 - J1) * ||J0 - J1||
        dist_J0_J1 = np.linalg.norm(j_points[1] - j_points[0])
        direction_J1_J2 = j_points[2] - j_points[1]
        direction_J1_J2_norm = direction_J1_J2 / np.linalg.norm(direction_J1_J2)
        s_points[1] = j_points[1] + direction_J1_J2_norm * dist_J0_J1

        # S2 to S[num_segments-1]: General pattern
        for i in range(2, num_segments):
            dist = np.linalg.norm(j_points[i] - s_points[i - 1])
            direction = j_points[i + 1] - j_points[i]
            direction_norm = direction / np.linalg.norm(direction)
            s_points[i] = j_points[i] + direction_norm * dist

        # Last S-point = last J-point (end-effector)
        s_points[num_segments] = j_points[-1].copy()

        return s_points

    @staticmethod
    def calculate_new_segment_lengths(s_points: np.ndarray) -> np.ndarray:
        """
        Calculate new segment lengths from S-points using robot geometry.

        Process:
        1. For each S-point, calculate prismatic_joint using segment_to_motors
        2. Create temporary straight vertical S-points with these lengths
        3. Convert to temporary J-points using calculate_j_points_from_s_points()
        4. Calculate J-to-J distances from temporary J-points
        5. Return these distances as new segment lengths

        Args:
            s_points: Recalculated S-points from forward pass, shape (num_segments+1, 3)

        Returns:
            joint_distances: New J-to-J distances for next backward pass, shape (num_joints-1,)
        """
        from .fabrik_initialization import FabrikInitialization

        num_segments = len(s_points) - 1
        segment_lengths = np.zeros(num_segments, dtype=np.float64)

        # Step 1 & 2: Calculate prismatic joint for each S-point
        # Skip S0 (baseplate at origin) - start from S1
        # S-points are in meters, segment_to_motors expects mm
        import warnings
        import os
        debug_first = os.environ.get('DEBUG_FABRIK') == '1'
        for i in range(1, num_segments + 1):  # S1 to S8
            s_point_mm = s_points[i] * 1000  # Convert to mm
            with warnings.catch_warnings():
                warnings.simplefilter("ignore")  # Suppress clipping warnings
                result = calculate_kinematics(s_point_mm)
            # prismatic_joint is in mm, keep in mm for segment lengths
            segment_lengths[i - 1] = result['prismatic_joint']  # Store in mm

            if debug_first and i <= 2:
                print(f"\n    [calculate_new_segment_lengths] S{i}:")
                print(f"      Input s_point (m): {s_points[i]}")
                print(f"      Input s_point_mm: {s_point_mm}")
                print(f"      Result prismatic_joint: {result['prismatic_joint']:.4f} mm")
                print(f"      Result fermat_point: {result['fermat_point']}")

        # Step 3: Create temporary straight vertical S-points (in mm)
        # segment_lengths contains prismatic_joint offsets, add to nominal TOTAL_SEGMENT_HEIGHT
        from robot_config import physical as phys_config

        NOMINAL_SEGMENT_HEIGHT_MM = phys_config.TOTAL_SEGMENT_HEIGHT * 1000  # 146mm

        temp_s_points = np.zeros((num_segments + 1, 3), dtype=np.float64)
        temp_s_points[0] = np.array([0.0, 0.0, 0.0])  # S0 at origin

        cumulative_height = 0.0
        for i in range(1, num_segments + 1):
            # Total segment height = nominal + 2x prismatic offset (two prismatic joints per segment)
            total_segment_height = NOMINAL_SEGMENT_HEIGHT_MM + 2 * segment_lengths[i - 1]
            cumulative_height += total_segment_height
            temp_s_points[i] = np.array([0.0, 0.0, cumulative_height])  # in mm

        # Step 4: Convert temporary S-points to J-points (in mm)
        temp_j_points = FabrikInitialization.calculate_j_points_from_s_points(temp_s_points)

        # Step 5: Calculate J-to-J distances (temp_j_points in mm, distances in mm, convert to meters)
        num_joints = len(temp_j_points)
        joint_distances = np.zeros(num_joints - 1, dtype=np.float64)
        for i in range(num_joints - 1):
            distance_mm = np.linalg.norm(temp_j_points[i + 1] - temp_j_points[i])
            joint_distances[i] = distance_mm / 1000.0  # Convert mm to meters

        # Discard temp_s_points and temp_j_points (only needed for distance calculation)
        return joint_distances

    @staticmethod
    def forward_pass(backward_joints: np.ndarray,
                     original_joints: np.ndarray,
                     joint_distances: np.ndarray,
                     cone_half_angle_rad: float = np.pi / 3.0) -> np.ndarray:
        """
        Perform forward pass: push joints from base toward target.

        Args:
            backward_joints: Joint positions after backward pass, shape (num_joints, 3)
            original_joints: Original joint positions before iteration, shape (num_joints, 3)
            joint_distances: Distances between consecutive joints, shape (num_joints-1,)
            cone_half_angle_rad: Cone constraint half-angle (default 60°)

        Returns:
            f_joints: Joint positions after forward pass, shape (num_joints, 3)
        """
        num_joints = len(backward_joints)
        f_joints = backward_joints.copy()

        # Step 1: Fix base position
        f_joints[0] = original_joints[0].copy()  # Always (0, 0, 0)

        # Step 2: Loop forward from J1 to end-effector
        for i in range(1, num_joints):
            if i == 1:
                # First joint from base (J1)
                # SPECIAL CASE: J1 must stay on Z-axis (above J0)
                # J0 is at origin (0, 0, 0), J1 is at (0, 0, distance)
                f_joints[i] = np.array([0.0, 0.0, joint_distances[i - 1]])

            else:
                # Middle and last joints (J2, J3, ..., J8)
                # APPLY CONE CONSTRAINT
                cone_apex = f_joints[i - 1]
                cone_axis = f_joints[i - 1] - f_joints[i - 2]

                # Desired direction from apex toward backward pass position
                desired_direction = backward_joints[i] - cone_apex

                # Apply cone constraint
                corrected_direction = FabrikConeConstraint.project_onto_cone(
                    desired_direction, cone_axis, cone_half_angle_rad
                )

                # Place joint at fixed distance from apex
                corrected_norm = corrected_direction / np.linalg.norm(corrected_direction)
                f_joints[i] = cone_apex + corrected_norm * joint_distances[i - 1]

        return f_joints
