#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from fabrik_ik_solver.msg import MotorCommand
import sys
import numpy as np

# Add robot_constants to path
sys.path.append('/home/yuuki/ROS2')
import robot_constants as rc

# Import Ruckig
try:
    from ruckig import Ruckig, InputParameter, OutputParameter, Result, Synchronization, ControlInterface
except ImportError:
    raise ImportError("Ruckig library not found. Install with: sudo apt install ros-humble-ruckig")


class MotorTrajectorySmootherNode(Node):
    def __init__(self):
        super().__init__('motor_trajectory_smoother_node')

        # Declare parameters
        self.declare_parameter('publish_rate', 100.0)  # Hz

        # Get parameters
        self.publish_rate = self.get_parameter('publish_rate').value

        # Initialize Ruckig for 24 motors
        self.ruckig = Ruckig(rc.NUM_MOTORS, rc.RUCKIG_DELTA_TIME)
        self.input_param = InputParameter(rc.NUM_MOTORS)
        self.output_param = OutputParameter(rc.NUM_MOTORS)

        # Configure Ruckig parameters from robot_constants
        self._configure_ruckig_parameters()

        # Waypoint buffer: list of MotorCommand messages
        self.waypoint_buffer = []
        self.buffer_lock = False  # Simple lock for buffer access

        # Flag to track if we're actively moving
        self.is_moving = False

        # Flag to track if we've initialized Ruckig state
        self.ruckig_initialized = False

        # Subscriber to motor commands from FABRIK
        self.motor_sub = self.create_subscription(
            MotorCommand,
            '/motor_commands',
            self.motor_command_callback,
            10
        )

        # Publisher for smoothed motor commands
        self.smoothed_pub = self.create_publisher(
            MotorCommand,
            '/smoothed_motor_commands',
            10
        )

        # Timer for publishing smooth trajectory at fixed rate
        self.publish_timer = self.create_timer(
            1.0 / self.publish_rate,
            self.publish_smooth_trajectory
        )

        self.get_logger().info('Motor Trajectory Smoother started')
        self.get_logger().info(f'  Motors: {rc.NUM_MOTORS}')
        self.get_logger().info(f'  Publish rate: {self.publish_rate} Hz')
        self.get_logger().info(f'  Ruckig delta_time: {rc.RUCKIG_DELTA_TIME}s')
        self.get_logger().info(f'  Max velocity: {rc.MOTOR_MAX_VELOCITY} m/s')
        self.get_logger().info(f'  Max acceleration: {rc.MOTOR_MAX_ACCELERATION} m/s²')
        self.get_logger().info(f'  Max jerk: {rc.MOTOR_MAX_JERK} m/s³')

    def _configure_ruckig_parameters(self):
        """Configure Ruckig input parameters from robot_constants"""
        # Set kinematic limits for all motors (same for all 24)
        for i in range(rc.NUM_MOTORS):
            self.input_param.max_velocity[i] = rc.MOTOR_MAX_VELOCITY
            self.input_param.max_acceleration[i] = rc.MOTOR_MAX_ACCELERATION
            self.input_param.max_jerk[i] = rc.MOTOR_MAX_JERK

        # Set synchronization mode
        if rc.RUCKIG_SYNCHRONIZATION == 'time':
            self.input_param.synchronization = Synchronization.Time
        elif rc.RUCKIG_SYNCHRONIZATION == 'phase':
            self.input_param.synchronization = Synchronization.Phase
        elif rc.RUCKIG_SYNCHRONIZATION == 'none':
            self.input_param.synchronization = Synchronization.No
        else:
            self.input_param.synchronization = Synchronization.Time

        # Set control interface
        if rc.RUCKIG_CONTROL_INTERFACE == 'position':
            self.input_param.control_interface = ControlInterface.Position
        elif rc.RUCKIG_CONTROL_INTERFACE == 'velocity':
            self.input_param.control_interface = ControlInterface.Velocity
        else:
            self.input_param.control_interface = ControlInterface.Position

    def motor_command_callback(self, msg: MotorCommand):
        """Receive new motor command waypoint from FABRIK"""
        # Add to buffer (simple list append)
        self.waypoint_buffer.append(msg)

        self.get_logger().info(
            f'New waypoint added | Buffer size: {len(self.waypoint_buffer)}'
        )

    def publish_smooth_trajectory(self):
        """Publish smooth motor trajectory at fixed rate using Ruckig"""
        # If no waypoints, do nothing
        if not self.waypoint_buffer:
            if self.is_moving:
                self.get_logger().info('No more waypoints - trajectory complete')
                self.is_moving = False
            return

        # Get next target waypoint (first in buffer)
        target_waypoint = self.waypoint_buffer[0]
        is_final_waypoint = (len(self.waypoint_buffer) == 1)

        # Initialize Ruckig state with first waypoint if not yet initialized
        if not self.ruckig_initialized:
            for i in range(rc.NUM_MOTORS):
                self.input_param.current_position[i] = target_waypoint.motor_positions[i]
                self.input_param.current_velocity[i] = 0.0
                self.input_param.current_acceleration[i] = 0.0
            self.ruckig_initialized = True
            self.get_logger().info('Ruckig initialized with first waypoint')

        # Set target position from waypoint
        for i in range(rc.NUM_MOTORS):
            self.input_param.target_position[i] = target_waypoint.motor_positions[i]

        # Check if target is same as current (no movement needed) - skip this waypoint
        if self.ruckig_initialized:
            all_same = True
            for i in range(rc.NUM_MOTORS):
                if abs(self.input_param.current_position[i] - self.input_param.target_position[i]) > 1e-6:
                    all_same = False
                    break
            if all_same:
                self.waypoint_buffer.pop(0)
                self.get_logger().info('Skipped waypoint (no movement needed)')
                return

        # Set target velocity based on whether this is final waypoint
        if is_final_waypoint:
            # Final waypoint: stop completely
            target_vel = rc.RUCKIG_FINAL_TARGET_VELOCITY
        else:
            # Intermediate waypoint: pass through at max velocity
            target_vel = rc.MOTOR_MAX_VELOCITY

        for i in range(rc.NUM_MOTORS):
            self.input_param.target_velocity[i] = target_vel
            self.input_param.target_acceleration[i] = rc.RUCKIG_TARGET_ACCELERATION
            # Re-set kinematic limits (pass_to_input can reset these)
            self.input_param.max_velocity[i] = rc.MOTOR_MAX_VELOCITY
            self.input_param.max_acceleration[i] = rc.MOTOR_MAX_ACCELERATION
            self.input_param.max_jerk[i] = rc.MOTOR_MAX_JERK

        # Update Ruckig trajectory
        result = self.ruckig.update(self.input_param, self.output_param)

        if result == Result.Error or result == Result.ErrorInvalidInput:
            self.get_logger().error(f'Ruckig error: {result}')
            return

        # Publish smoothed motor command
        smoothed_msg = MotorCommand()
        smoothed_msg.timestamp = self.get_clock().now().to_msg()
        smoothed_msg.motor_positions = list(self.output_param.new_position)
        smoothed_msg.joint_angles = target_waypoint.joint_angles  # Pass through
        smoothed_msg.converged = target_waypoint.converged
        smoothed_msg.iterations = target_waypoint.iterations
        smoothed_msg.final_error = target_waypoint.final_error

        self.smoothed_pub.publish(smoothed_msg)
        self.is_moving = True

        # Pass output to input for next cycle
        self.output_param.pass_to_input(self.input_param)

        # If we reached the target waypoint, remove it from buffer
        if result == Result.Finished:
            self.waypoint_buffer.pop(0)
            self.get_logger().info(
                f'Waypoint reached | Remaining: {len(self.waypoint_buffer)}'
            )


def main(args=None):
    rclpy.init(args=args)
    node = MotorTrajectorySmootherNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
