#!/usr/bin/env python3
"""
FABRIK Kinematics Module

Converts segment positions to motor positions and joint angles.
Implements Fermat point calculation, motor positioning, and joint state extraction.

All calculations preserve exact C++ kinematics implementation.
"""

import numpy as np
from typing import List, Dict
import math
import warnings

from robot_config import physical as phys_config

# Convert to mm for internal calculations
ROBOT_RADIUS = phys_config.ACTUATOR_RADIUS * 1000
WORKING_HEIGHT = phys_config.BASE_HEIGHT * 1000
MOTOR_LIMIT = phys_config.PRISMATIC_RANGE * 1000
SEGMENT_HEIGHT = phys_config.SEGMENT_OFFSET * 1000

# Motor base angles
BASE_A_ANGLE = math.pi / 2.0
BASE_B_ANGLE = -math.pi / 6.0
BASE_C_ANGLE = -5.0 * math.pi / 6.0


def get_base_position_A():
    """Motor A base position."""
    return np.array([0.0, ROBOT_RADIUS, 0.0])


def get_base_position_B():
    """Motor B base position."""
    return np.array([
        ROBOT_RADIUS * math.cos(BASE_B_ANGLE),
        ROBOT_RADIUS * math.sin(BASE_B_ANGLE),
        0.0
    ])


def get_base_position_C():
    """Motor C base position."""
    return np.array([
        ROBOT_RADIUS * math.cos(BASE_C_ANGLE),
        ROBOT_RADIUS * math.sin(BASE_C_ANGLE),
        0.0
    ])


def calculate_z_intersection(base_x: float, base_y: float, normal: np.ndarray) -> float:
    """
    Calculate Z intersection of vertical line with plane.

    Args:
        base_x: X coordinate of vertical line
        base_y: Y coordinate of vertical line
        normal: Plane normal vector

    Returns:
        Z intersection coordinate
    """
    if abs(normal[2]) < 1e-10:
        return 0.0

    return -(normal[0] * base_x + normal[1] * base_y) / normal[2]


def calculate_fermat_point(direction: np.ndarray) -> Dict:
    """
    Calculate Fermat point and motor Z positions from direction vector.

    Args:
        direction: Segment direction vector

    Returns:
        Dictionary containing:
            - z_A, z_B, z_C: Motor Z positions (mm)
            - fermat_point: Fermat point position
            - A_point, B_point, C_point: 3D motor positions
    """
    # Normalize direction
    normal = direction / np.linalg.norm(direction)

    # Get base positions
    base_A = get_base_position_A()
    base_B = get_base_position_B()
    base_C = get_base_position_C()

    # Calculate Z intersections
    z_A = calculate_z_intersection(base_A[0], base_A[1], normal)
    z_B = calculate_z_intersection(base_B[0], base_B[1], normal)
    z_C = calculate_z_intersection(base_C[0], base_C[1], normal)

    # Center around zero
    avg = (z_A + z_B + z_C) / 3.0
    z_A_centered = z_A - avg
    z_B_centered = z_B - avg
    z_C_centered = z_C - avg

    # Apply motor limits
    min_val = min(z_A_centered, z_B_centered, z_C_centered)
    max_val = max(z_A_centered, z_B_centered, z_C_centered)

    constraint_offset = 0.0
    if min_val < -MOTOR_LIMIT:
        constraint_offset = -MOTOR_LIMIT - min_val
    elif max_val > MOTOR_LIMIT:
        constraint_offset = MOTOR_LIMIT - max_val

    z_A = z_A_centered + constraint_offset
    z_B = z_B_centered + constraint_offset
    z_C = z_C_centered + constraint_offset

    # Clip if still out of bounds
    clipped = False
    if z_A < -MOTOR_LIMIT or z_A > MOTOR_LIMIT:
        z_A = np.clip(z_A, -MOTOR_LIMIT, MOTOR_LIMIT)
        clipped = True
    if z_B < -MOTOR_LIMIT or z_B > MOTOR_LIMIT:
        z_B = np.clip(z_B, -MOTOR_LIMIT, MOTOR_LIMIT)
        clipped = True
    if z_C < -MOTOR_LIMIT or z_C > MOTOR_LIMIT:
        z_C = np.clip(z_C, -MOTOR_LIMIT, MOTOR_LIMIT)
        clipped = True

    if clipped:
        warnings.warn(
            f"Motor positions clipped to ±{MOTOR_LIMIT:.2f}mm. "
            f"Segment position may be unreachable."
        )

    # Create 3D motor positions
    A_point = np.array([base_A[0], base_A[1], z_A])
    B_point = np.array([base_B[0], base_B[1], z_B])
    C_point = np.array([base_C[0], base_C[1], z_C])

    # Calculate side lengths
    AB = B_point - A_point
    BC = C_point - B_point
    CA = A_point - C_point

    side_a = np.linalg.norm(BC)
    side_b = np.linalg.norm(CA)
    side_c = np.linalg.norm(AB)

    # Calculate angles
    neg_CA = -CA
    neg_AB = -AB
    neg_BC = -BC

    alpha = math.acos(np.clip(np.dot(neg_CA, AB) / (np.linalg.norm(CA) * np.linalg.norm(AB)), -1.0, 1.0))
    beta = math.acos(np.clip(np.dot(neg_AB, BC) / (np.linalg.norm(AB) * np.linalg.norm(BC)), -1.0, 1.0))
    gamma = math.acos(np.clip(np.dot(neg_BC, CA) / (np.linalg.norm(BC) * np.linalg.norm(CA)), -1.0, 1.0))

    # Calculate lambda values
    epsilon = 1e-10
    sin_alpha = math.sin(alpha + math.pi / 3)
    sin_beta = math.sin(beta + math.pi / 3)
    sin_gamma = math.sin(gamma + math.pi / 3)

    lambda_A = side_a / max(sin_alpha, epsilon)
    lambda_B = side_b / max(sin_beta, epsilon)
    lambda_C = side_c / max(sin_gamma, epsilon)

    # Calculate Fermat point (weighted centroid)
    total_lambda = lambda_A + lambda_B + lambda_C
    fermat_x = (lambda_A * A_point[0] + lambda_B * B_point[0] + lambda_C * C_point[0]) / total_lambda
    fermat_y = (lambda_A * A_point[1] + lambda_B * B_point[1] + lambda_C * C_point[1]) / total_lambda
    fermat_z = (lambda_A * A_point[2] + lambda_B * B_point[2] + lambda_C * C_point[2]) / total_lambda

    fermat_point = np.array([fermat_x, fermat_y, fermat_z])

    return {
        'z_A': z_A,
        'z_B': z_B,
        'z_C': z_C,
        'fermat_point': fermat_point,
        'A_point': A_point,
        'B_point': B_point,
        'C_point': C_point
    }


def calculate_prismatic_joint(fermat_point: np.ndarray) -> float:
    """
    Calculate prismatic joint extension from Fermat point.

    Args:
        fermat_point: Fermat point position

    Returns:
        Prismatic joint value (mm)
    """
    return 2.0 * fermat_point[2]


def calculate_roll_joint(direction_vector: np.ndarray) -> float:
    """
    Calculate roll joint angle (rotation around X-axis).

    Args:
        direction_vector: Segment direction vector

    Returns:
        Roll angle (radians)
    """
    normalized = direction_vector / np.linalg.norm(direction_vector)
    return -math.atan2(normalized[1], normalized[2])


def calculate_pitch_joint(direction_vector: np.ndarray) -> float:
    """
    Calculate pitch joint angle (rotation around Y-axis).

    Args:
        direction_vector: Segment direction vector

    Returns:
        Pitch angle (radians)
    """
    normalized = direction_vector / np.linalg.norm(direction_vector)
    return math.atan2(normalized[0], normalized[2])


def calculate_kinematics(segment_position: np.ndarray) -> Dict:
    """
    Calculate complete kinematics for a segment position.

    Args:
        segment_position: 3D segment position (mm)

    Returns:
        Dictionary containing:
            - z_A, z_B, z_C: Motor Z positions (mm)
            - prismatic_joint: Prismatic extension (mm)
            - roll_joint: Roll angle (radians)
            - pitch_joint: Pitch angle (radians)
            - fermat_point: Fermat point position
            - A_point, B_point, C_point: 3D motor positions
    """
    # Use segment position as direction vector
    direction_vector = segment_position / np.linalg.norm(segment_position)

    # Calculate Fermat point and motor positions
    fermat_result = calculate_fermat_point(direction_vector)

    # Calculate joint states
    prismatic = calculate_prismatic_joint(fermat_result['fermat_point'])
    roll = calculate_roll_joint(direction_vector)
    pitch = calculate_pitch_joint(direction_vector)

    return {
        'z_A': fermat_result['z_A'],
        'z_B': fermat_result['z_B'],
        'z_C': fermat_result['z_C'],
        'prismatic_joint': prismatic,
        'roll_joint': roll,
        'pitch_joint': pitch,
        'fermat_point': fermat_result['fermat_point'],
        'A_point': fermat_result['A_point'],
        'B_point': fermat_result['B_point'],
        'C_point': fermat_result['C_point']
    }


def calculate_plane_normal(A_point: np.ndarray, B_point: np.ndarray, C_point: np.ndarray) -> np.ndarray:
    """
    Calculate normal vector to ABC plane.

    Args:
        A_point, B_point, C_point: 3D motor positions

    Returns:
        Normalized plane normal vector
    """
    AB = B_point - A_point
    AC = C_point - A_point
    normal = np.cross(AC, AB)
    return normal / np.linalg.norm(normal)


def create_UVW_frame(fermat_point: np.ndarray, A_point: np.ndarray,
                     B_point: np.ndarray, C_point: np.ndarray) -> Dict:
    """
    Create UVW coordinate frame at Fermat point.

    Args:
        fermat_point: Origin of UVW frame
        A_point, B_point, C_point: 3D motor positions

    Returns:
        Dictionary containing:
            - origin: Fermat point
            - u_axis, v_axis, w_axis: Coordinate axes
    """
    # V-axis: Fermat → A
    v_axis = (A_point - fermat_point)
    v_axis = v_axis / np.linalg.norm(v_axis)

    # W-axis: Normal to ABC plane
    w_axis = calculate_plane_normal(A_point, B_point, C_point)
    if w_axis[2] < 0:
        w_axis = -w_axis

    # U-axis: V × W
    u_axis = np.cross(v_axis, w_axis)
    u_axis = u_axis / np.linalg.norm(u_axis)

    return {
        'origin': fermat_point,
        'u_axis': u_axis,
        'v_axis': v_axis,
        'w_axis': w_axis
    }


def create_uvw_to_xyz_rotation_matrix(u_axis: np.ndarray, v_axis: np.ndarray,
                                      w_axis: np.ndarray) -> np.ndarray:
    """
    Create rotation matrix from UVW to XYZ frame.

    Args:
        u_axis, v_axis, w_axis: UVW frame axes

    Returns:
        3x3 rotation matrix
    """
    # Align axes
    aligned_u = u_axis.copy()
    aligned_v = v_axis.copy()
    aligned_w = w_axis.copy()

    if u_axis[0] < 0:
        aligned_u = -u_axis
    if v_axis[1] < 0:
        aligned_v = -v_axis
    if w_axis[2] < 0:
        aligned_w = -w_axis

    return np.vstack([aligned_u, aligned_v, aligned_w])


def transform_segment_to_local_frame(segment_position: np.ndarray,
                                     base_position: np.ndarray,
                                     rotation_matrix: np.ndarray) -> np.ndarray:
    """
    Transform segment position into local coordinate frame.

    Args:
        segment_position: Global segment position
        base_position: Global base position
        rotation_matrix: Rotation from UVW to XYZ

    Returns:
        Transformed position in local frame
    """
    translated_pos = segment_position - base_position
    transformed_pos = rotation_matrix @ translated_pos
    return transformed_pos


def mirror_across_XY(uvw_frame: Dict) -> Dict:
    """
    Step 2: Mirror UVW frame across XY plane to get IJK frame.

    Args:
        uvw_frame: Dict with 'origin', 'u_axis', 'v_axis', 'w_axis'

    Returns:
        ijk_frame: Dict with mirrored coordinates
    """
    # Mirror across XY plane: (x,y,z) → (x,y,-z)
    mirrored_origin = np.array([
        uvw_frame['origin'][0],
        uvw_frame['origin'][1],
        -uvw_frame['origin'][2]
    ])

    mirrored_u = np.array([
        uvw_frame['u_axis'][0],
        uvw_frame['u_axis'][1],
        -uvw_frame['u_axis'][2]
    ])

    mirrored_v = np.array([
        uvw_frame['v_axis'][0],
        uvw_frame['v_axis'][1],
        -uvw_frame['v_axis'][2]
    ])

    mirrored_w = np.array([
        uvw_frame['w_axis'][0],
        uvw_frame['w_axis'][1],
        -uvw_frame['w_axis'][2]
    ])

    # Invert W to make K point upward
    inverted_k = -mirrored_w

    return {
        'origin': mirrored_origin,
        'u_axis': mirrored_u,
        'v_axis': mirrored_v,
        'w_axis': inverted_k
    }


def align_with_origin(uvw_frame: Dict, ijk_frame: Dict) -> Dict:
    """
    Step 3: Align IJK with XYZ origin to get U'V'W' frame.

    Args:
        uvw_frame: Original UVW frame
        ijk_frame: Mirrored IJK frame

    Returns:
        aligned_frame: U'V'W' frame aligned with origin
    """
    # Translation: move IJK origin to (0,0,0)
    translation = np.array([0.0, 0.0, 0.0]) - ijk_frame['origin']

    # Build IJK matrix as column vectors
    ijk_matrix = np.column_stack([
        ijk_frame['u_axis'],
        ijk_frame['v_axis'],
        ijk_frame['w_axis']
    ])

    # Rotation matrix R = [I J K]^(-1)
    R = np.linalg.inv(ijk_matrix)

    # Apply translation to UVW origin
    new_uvw_origin = uvw_frame['origin'] + translation

    # Apply rotation R to UVW axes
    new_u_axis = R @ uvw_frame['u_axis']
    new_v_axis = R @ uvw_frame['v_axis']
    new_w_axis = R @ uvw_frame['w_axis']

    return {
        'origin': new_uvw_origin,
        'u_axis': new_u_axis,
        'v_axis': new_v_axis,
        'w_axis': new_w_axis
    }


def translate_to_position(aligned_frame: Dict, position: np.ndarray) -> Dict:
    """
    Step 4: Translate aligned frame to target position to get U''V''W''.

    Args:
        aligned_frame: U'V'W' frame
        position: Target position (S-point)

    Returns:
        final_frame: U''V''W'' frame at target position
    """
    return {
        'origin': position,
        'u_axis': aligned_frame['u_axis'],
        'v_axis': aligned_frame['v_axis'],
        'w_axis': aligned_frame['w_axis']
    }


def calculate_full_orientation(s_point_mm: np.ndarray) -> Dict:
    """
    Calculate full orientation transformation for a segment.
    Uses direction from origin (for segments in their parent's local frame).

    Steps:
    1. Create UVW frame at Fermat point (using S direction from origin)
    2. Mirror across XY → IJK
    3. Align with origin → U'V'W'
    4. Translate to S-point → U''V''W''

    Args:
        s_point_mm: Segment position [x, y, z] in mm

    Returns:
        Dict with final U''V''W'' frame and fermat result
    """
    s_point = np.array(s_point_mm)

    # Direction vector (from origin in current frame)
    direction = s_point / np.linalg.norm(s_point)

    # Calculate Fermat point and motor positions
    fermat_result = calculate_fermat_point(direction)

    # Step 1: Create UVW frame at Fermat point
    uvw_frame = create_UVW_frame(
        fermat_result['fermat_point'],
        fermat_result['A_point'],
        fermat_result['B_point'],
        fermat_result['C_point']
    )

    # Step 2: Mirror across XY → IJK
    ijk_frame = mirror_across_XY(uvw_frame)

    # Step 3: Align with origin → U'V'W'
    aligned_frame = align_with_origin(uvw_frame, ijk_frame)

    # Step 4: Translate to S-point → U''V''W''
    final_frame = translate_to_position(aligned_frame, s_point)

    return {
        'final_frame': final_frame,
        'fermat_result': fermat_result
    }


def calculate_motors_for_segments(segment_positions: List[np.ndarray]) -> List[Dict]:
    """
    Calculate motor positions for all segments using hierarchical transformation.

    Args:
        segment_positions: List of 3D segment positions (mm)

    Returns:
        List of dictionaries containing motor data for each segment level
    """
    levels = []
    current_input_positions = [np.array(pos) for pos in segment_positions]
    level_index = 0

    while len(current_input_positions) > 0:
        level_data = {'level': level_index}

        # Base segment for this level
        base_pos_for_this_level = current_input_positions[0]
        level_data['base_segment_position'] = base_pos_for_this_level.tolist()

        # Calculate full orientation (4-step transformation: UVW → IJK → U'V'W' → U''V''W'')
        orientation = calculate_full_orientation(base_pos_for_this_level)
        final_frame = orientation['final_frame']
        fermat_result = orientation['fermat_result']

        # Extract motor positions from fermat result
        level_data['z_A'] = fermat_result['z_A']
        level_data['z_B'] = fermat_result['z_B']
        level_data['z_C'] = fermat_result['z_C']

        # Calculate joint angles from segment direction
        direction_vector = base_pos_for_this_level / np.linalg.norm(base_pos_for_this_level)
        level_data['prismatic_joint'] = calculate_prismatic_joint(fermat_result['fermat_point'])
        level_data['roll_joint'] = math.degrees(calculate_roll_joint(direction_vector))
        level_data['pitch_joint'] = math.degrees(calculate_pitch_joint(direction_vector))

        # Store final U''V''W'' frame
        level_data['final_frame'] = {
            'origin': final_frame['origin'].tolist(),
            'u_axis': final_frame['u_axis'].tolist(),
            'v_axis': final_frame['v_axis'].tolist(),
            'w_axis': final_frame['w_axis'].tolist()
        }

        # Transform remaining segments for next level
        next_level_input_positions = []
        if len(current_input_positions) > 1:
            # Create rotation matrix from U''V''W'' to XYZ
            rotation_matrix = np.linalg.inv(np.column_stack([
                final_frame['u_axis'],
                final_frame['v_axis'],
                final_frame['w_axis']
            ]))

            transformed_segments = []
            for i in range(1, len(current_input_positions)):
                segment_to_transform = current_input_positions[i]
                transformed_pos = transform_segment_to_local_frame(
                    segment_to_transform,
                    base_pos_for_this_level,
                    rotation_matrix
                )
                transformed_segments.append(transformed_pos.tolist())
                next_level_input_positions.append(transformed_pos)

            level_data['transformed_segments'] = transformed_segments
        else:
            level_data['transformed_segments'] = []

        levels.append(level_data)
        current_input_positions = next_level_input_positions
        level_index += 1

    return levels


def print_motor_results(levels: List[Dict]):
    """Print motor calculation results in readable format."""
    print("\n" + "=" * 70)
    print("MOTOR CALCULATION RESULTS")
    print("=" * 70)

    for level_data in levels:
        level = level_data['level']
        print(f"\nLEVEL {level} (Segment {level + 1}):")
        print(f"  Base position: {level_data['base_segment_position']}")
        print(f"  Motor A Z: {level_data['z_A']:.3f} mm")
        print(f"  Motor B Z: {level_data['z_B']:.3f} mm")
        print(f"  Motor C Z: {level_data['z_C']:.3f} mm")
        print(f"  Prismatic: {level_data['prismatic_joint']:.3f} mm")
        print(f"  Roll:      {level_data['roll_joint']:.2f}°")
        print(f"  Pitch:     {level_data['pitch_joint']:.2f}°")

        if len(level_data['transformed_segments']) > 0:
            print(f"  Transformed segments:")
            for i, seg in enumerate(level_data['transformed_segments']):
                print(f"    Segment {level + i + 2}: {seg}")


if __name__ == "__main__":
    # Example usage
    segment_positions = [
        [0, 0, 100],
        [10, 5, 180],
        [15, 8, 250]
    ]

    print("Input segment positions (global frame):")
    for i, pos in enumerate(segment_positions):
        print(f"  Segment {i+1}: {pos}")

    motor_results = calculate_motors_for_segments(segment_positions)
    print_motor_results(motor_results)

    print("\n" + "=" * 70)
    print("SINGLE SEGMENT EXAMPLE")
    print("=" * 70)

    single_segment = np.array([10, 5, 100])
    print(f"\nInput: {single_segment}")

    result = calculate_kinematics(single_segment)
    print(f"\nOutput:")
    print(f"  Motor A Z: {result['z_A']:.3f} mm")
    print(f"  Motor B Z: {result['z_B']:.3f} mm")
    print(f"  Motor C Z: {result['z_C']:.3f} mm")
    print(f"  Prismatic: {result['prismatic_joint']:.3f} mm")
    print(f"  Roll:      {math.degrees(result['roll_joint']):.2f}°")
    print(f"  Pitch:     {math.degrees(result['pitch_joint']):.2f}°")
