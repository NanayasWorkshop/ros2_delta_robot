#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from fabrik_ik_solver.msg import MotorCommand
from sensor_msgs.msg import JointState
import numpy as np
import math

from robot_config import physical as phys_config
from robot_config import motion as motion_config
from robot_config import system as sys_config


class MotorToJointConverterNode(Node):
    def __init__(self):
        super().__init__('motor_to_joint_converter_node')

        # Subscribe to smoothed motor commands
        self.motor_sub = self.create_subscription(
            MotorCommand,
            sys_config.TOPIC_SMOOTHED_MOTOR_COMMANDS,
            self.motor_command_callback,
            sys_config.QUEUE_SIZE_DEFAULT
        )

        # Publish joint states
        self.joint_pub = self.create_publisher(
            JointState,
            sys_config.TOPIC_JOINT_STATES,
            sys_config.QUEUE_SIZE_DEFAULT
        )

        self.get_logger().info('Motor to Joint Converter started')
        self.get_logger().info(f'  Segments: {phys_config.NUM_SEGMENTS}')
        self.get_logger().info(f'  Motors per segment: 3')
        self.get_logger().info(f'  Total motors: {phys_config.NUM_MOTORS}')

    def motor_command_callback(self, msg: MotorCommand):
        """Convert motor positions to joint angles and publish JointState"""
        # Motor positions are in the joint_angles field (24 values)
        motor_positions = msg.motor_positions

        if len(motor_positions) != phys_config.NUM_MOTORS:
            self.get_logger().error(f'Expected {phys_config.NUM_MOTORS} motor positions, got {len(motor_positions)}')
            return

        # Convert to joint states
        joint_state = JointState()
        joint_state.header.stamp = self.get_clock().now().to_msg()
        joint_state.name = []
        joint_state.position = []

        # Process each segment (3 motors → 3 joints per segment)
        for seg_idx in range(phys_config.NUM_SEGMENTS):
            # Extract motor positions for this segment
            motor_start = seg_idx * 3
            z_A = motor_positions[motor_start + 0]
            z_B = motor_positions[motor_start + 1]
            z_C = motor_positions[motor_start + 2]

            # Convert motors → joints using copied kinematics formulas
            joints = self.motors_to_joints(z_A, z_B, z_C)

            # Debug: Log first segment
            if seg_idx == 0:
                self.get_logger().info(f'Seg1 motors: A={z_A*1000:.2f}mm B={z_B*1000:.2f}mm C={z_C*1000:.2f}mm')
                self.get_logger().info(f'Seg1 joints: roll={math.degrees(joints["roll"]):.2f}° pitch={math.degrees(joints["pitch"]):.2f}° prismatic={joints["prismatic"]*1000:.2f}mm')

            # Add to JointState message
            seg_name = f'seg{seg_idx + 1}'
            joint_state.name.extend([
                f'{seg_name}_joint1',  # roll (revolute X)
                f'{seg_name}_joint2',  # pitch (revolute Y)
                f'{seg_name}_joint3',  # prismatic (linear Z)
            ])
            joint_state.position.extend([
                joints['roll'],
                joints['pitch'],
                joints['prismatic']
            ])

        # Publish joint state
        self.joint_pub.publish(joint_state)

    def motors_to_joints(self, z_A: float, z_B: float, z_C: float) -> dict:
        """
        Convert motor Z positions to joint angles.

        The direction vector is the normal of the plane through A, B, C.

        Args:
            z_A, z_B, z_C: Motor Z positions (m)

        Returns:
            Dictionary with roll, pitch, prismatic joint values
        """
        # Get base positions (in meters)
        base_A = self.get_base_position_A()
        base_B = self.get_base_position_B()
        base_C = self.get_base_position_C()

        # Create 3D motor positions
        A_point = np.array([base_A[0], base_A[1], z_A])
        B_point = np.array([base_B[0], base_B[1], z_B])
        C_point = np.array([base_C[0], base_C[1], z_C])

        # Calculate direction vector as the normal of the plane through A, B, C
        # Using cross product: (C-A) × (B-A) to get upward-pointing normal
        AB = B_point - A_point
        AC = C_point - A_point
        direction = np.cross(AC, AB)  # Flipped order for +Z direction
        direction = direction / np.linalg.norm(direction)  # Normalize

        # Calculate Fermat point (weighted centroid)
        fermat_point = self.calculate_fermat_point(A_point, B_point, C_point)

        # Calculate prismatic joint (extension along Z)
        prismatic = motion_config.MOTOR_TO_JOINT_PRISMATIC_MULTIPLIER * fermat_point[2]

        # Calculate roll and pitch from direction vector
        roll = -math.atan2(direction[1], direction[2])
        pitch = math.atan2(direction[0], direction[2])

        return {
            'roll': roll,
            'pitch': pitch,
            'prismatic': prismatic
        }

    def calculate_fermat_point(self, A_point: np.ndarray, B_point: np.ndarray,
                                C_point: np.ndarray) -> np.ndarray:
        """
        Calculate Fermat point from three motor positions.

        Args:
            A_point, B_point, C_point: 3D motor positions

        Returns:
            Fermat point position
        """
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
        epsilon = motion_config.MOTOR_TO_JOINT_DIVISION_EPSILON
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

        return np.array([fermat_x, fermat_y, fermat_z])

    def get_base_position_A(self) -> tuple:
        """Get base position for motor A (in meters) - at 90° (0, r)"""
        r = phys_config.ACTUATOR_RADIUS
        return (0.0, r, 0.0)

    def get_base_position_B(self) -> tuple:
        """Get base position for motor B (in meters) - at 330° (-30°)"""
        r = phys_config.ACTUATOR_RADIUS
        angle = phys_config.MOTOR_B_ANGLE
        return (r * math.cos(angle), r * math.sin(angle), 0.0)

    def get_base_position_C(self) -> tuple:
        """Get base position for motor C (in meters) - at 210° (-150°)"""
        r = phys_config.ACTUATOR_RADIUS
        angle = phys_config.MOTOR_C_ANGLE
        return (r * math.cos(angle), r * math.sin(angle), 0.0)


def main(args=None):
    rclpy.init(args=args)
    node = MotorToJointConverterNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
