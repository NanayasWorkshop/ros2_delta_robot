#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from fabrik_ik_solver.msg import MotorCommand
from std_msgs.msg import Float64MultiArray, Int32
import numpy as np

from robot_config import physical as phys_config
from robot_config import motion as motion_config
from robot_config import system as sys_config

# Import Ruckig
try:
    from ruckig import Ruckig, InputParameter, OutputParameter, Result, Synchronization, ControlInterface
except ImportError:
    raise ImportError("Ruckig library not found. Install with: sudo apt install ros-humble-ruckig")


class MotorTrajectorySmootherNode(Node):
    def __init__(self):
        super().__init__('motor_trajectory_smoother_node')

        # Declare parameters
        self.declare_parameter('publish_rate', motion_config.RUCKIG_PUBLISH_RATE)

        # Get parameters
        self.publish_rate = self.get_parameter('publish_rate').value

        # Initialize Ruckig for 24 motors
        self.ruckig = Ruckig(phys_config.NUM_MOTORS, motion_config.RUCKIG_DELTA_TIME)
        self.input_param = InputParameter(phys_config.NUM_MOTORS)
        self.output_param = OutputParameter(phys_config.NUM_MOTORS)

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
            sys_config.TOPIC_MOTOR_COMMANDS,
            self.motor_command_callback,
            sys_config.QUEUE_SIZE_DEFAULT
        )

        # Publisher for smoothed motor commands
        self.smoothed_pub = self.create_publisher(
            MotorCommand,
            sys_config.TOPIC_SMOOTHED_MOTOR_COMMANDS,
            sys_config.QUEUE_SIZE_DEFAULT
        )

        # Publishers for PlotJuggler visualization
        self.viz_current_pos_pub = self.create_publisher(Float64MultiArray, sys_config.TOPIC_RUCKIG_CURRENT_POSITION, sys_config.QUEUE_SIZE_DEFAULT)
        self.viz_current_vel_pub = self.create_publisher(Float64MultiArray, sys_config.TOPIC_RUCKIG_CURRENT_VELOCITY, sys_config.QUEUE_SIZE_DEFAULT)
        self.viz_current_acc_pub = self.create_publisher(Float64MultiArray, sys_config.TOPIC_RUCKIG_CURRENT_ACCELERATION, sys_config.QUEUE_SIZE_DEFAULT)
        self.viz_target_pos_pub = self.create_publisher(Float64MultiArray, sys_config.TOPIC_RUCKIG_TARGET_POSITION, sys_config.QUEUE_SIZE_DEFAULT)
        self.viz_buffer_size_pub = self.create_publisher(Int32, sys_config.TOPIC_RUCKIG_BUFFER_SIZE, sys_config.QUEUE_SIZE_DEFAULT)

        # Timer for publishing smooth trajectory at fixed rate
        self.publish_timer = self.create_timer(
            1.0 / self.publish_rate,
            self.publish_smooth_trajectory
        )

        self.get_logger().info('Motor Trajectory Smoother started')
        self.get_logger().info(f'  Motors: {phys_config.NUM_MOTORS}')
        self.get_logger().info(f'  Publish rate: {self.publish_rate} Hz')
        self.get_logger().info(f'  Ruckig delta_time: {motion_config.RUCKIG_DELTA_TIME}s')
        self.get_logger().info(f'  Max velocity: {motion_config.MOTOR_MAX_VELOCITY} m/s')
        self.get_logger().info(f'  Max acceleration: {motion_config.MOTOR_MAX_ACCELERATION} m/s²')
        self.get_logger().info(f'  Max jerk: {motion_config.MOTOR_MAX_JERK} m/s³')

    def _configure_ruckig_parameters(self):
        """Configure Ruckig input parameters from robot_constants"""
        # Set kinematic limits for all motors (same for all 24) - assign whole list!
        self.input_param.max_velocity = [motion_config.MOTOR_MAX_VELOCITY] * phys_config.NUM_MOTORS
        self.input_param.max_acceleration = [motion_config.MOTOR_MAX_ACCELERATION] * phys_config.NUM_MOTORS
        self.input_param.max_jerk = [motion_config.MOTOR_MAX_JERK] * phys_config.NUM_MOTORS

        # Set synchronization mode
        if motion_config.RUCKIG_SYNCHRONIZATION == 'time':
            self.input_param.synchronization = Synchronization.Time
        elif motion_config.RUCKIG_SYNCHRONIZATION == 'phase':
            self.input_param.synchronization = Synchronization.Phase
        elif motion_config.RUCKIG_SYNCHRONIZATION == 'none':
            self.input_param.synchronization = Synchronization.No
        else:
            self.input_param.synchronization = Synchronization.Time

        # Set control interface
        if motion_config.RUCKIG_CONTROL_INTERFACE == 'position':
            self.input_param.control_interface = ControlInterface.Position
        elif motion_config.RUCKIG_CONTROL_INTERFACE == 'velocity':
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
        """Publish smooth motor trajectory at fixed rate using Ruckig with look-ahead"""
        # If no waypoints, do nothing
        if not self.waypoint_buffer:
            if self.is_moving:
                self.get_logger().info('No more waypoints - trajectory complete')
                self.is_moving = False
            return

        # Initialize Ruckig state with first waypoint if not yet initialized
        if not self.ruckig_initialized:
            first_waypoint = self.waypoint_buffer[0]
            # Initialize current state - assign whole lists!
            self.input_param.current_position = list(first_waypoint.motor_positions)
            self.input_param.current_velocity = [motion_config.RUCKIG_INITIAL_VELOCITY] * phys_config.NUM_MOTORS
            self.input_param.current_acceleration = [motion_config.RUCKIG_INITIAL_ACCELERATION] * phys_config.NUM_MOTORS
            self.ruckig_initialized = True
            self.waypoint_buffer.pop(0)  # Remove initialization waypoint
            self.get_logger().info(f'Ruckig initialized at position: [{self.input_param.current_position[0]:.6f}, {self.input_param.current_position[1]:.6f}, {self.input_param.current_position[2]:.6f}]')
            return  # Wait for next waypoint

        # BATCH MODE TEST: Wait until we have enough waypoints before starting
        MIN_BUFFER_SIZE_TO_START = 20  # Wait for 20 waypoints (~4 seconds at 5 Hz)

        if len(self.waypoint_buffer) < MIN_BUFFER_SIZE_TO_START and not self.is_moving:
            # Still collecting waypoints, wait...
            self.get_logger().info(
                f'Collecting waypoints... {len(self.waypoint_buffer)}/{MIN_BUFFER_SIZE_TO_START}',
                throttle_duration_sec=1.0
            )
            return

        if not self.is_moving and len(self.waypoint_buffer) >= MIN_BUFFER_SIZE_TO_START:
            self.get_logger().info(f'✓ Buffer ready with {len(self.waypoint_buffer)} waypoints - starting trajectory!')

        # Ruckig waypoint logic:
        # - intermediate_positions = waypoints to CRUISE THROUGH (no stopping)
        # - target_position = FINAL destination (where target_velocity applies)
        #
        # Strategy: Use look-ahead to decide if we should cruise or stop
        max_lookahead = min(motion_config.RUCKIG_MAX_LOOKAHEAD_WAYPOINTS, len(self.waypoint_buffer))

        if max_lookahead == 1:
            # Only 1 waypoint left → this is the final target, stop here
            self.input_param.target_position = list(self.waypoint_buffer[0].motor_positions)
            self.input_param.intermediate_positions = []
            target_vel = motion_config.RUCKIG_FINAL_TARGET_VELOCITY  # Stop
            target_acc = motion_config.RUCKIG_TARGET_ACCELERATION
            target_waypoint = self.waypoint_buffer[0]
        else:
            # Multiple waypoints → cruise through buffer[0...N-2], target is buffer[N-1]
            # Set intermediate positions (cruise through these)
            self.input_param.intermediate_positions = []
            for i in range(0, max_lookahead - 1):
                self.input_param.intermediate_positions.append(
                    list(self.waypoint_buffer[i].motor_positions)
                )

            # Set target position (final point in lookahead)
            target_idx = max_lookahead - 1
            self.input_param.target_position = list(self.waypoint_buffer[target_idx].motor_positions)
            target_waypoint = self.waypoint_buffer[target_idx]

            # Keep moving - don't stop at target since more waypoints may come
            target_vel = 0.0  # Let Ruckig optimize
            target_acc = 0.0

        # Assign target velocity and acceleration - whole lists!
        self.input_param.target_velocity = [target_vel] * phys_config.NUM_MOTORS
        self.input_param.target_acceleration = [target_acc] * phys_config.NUM_MOTORS

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

        # Publish visualization data for PlotJuggler
        self._publish_visualization_data()

        # Pass output to input for next cycle
        self.output_param.pass_to_input(self.input_param)

        # If we reached the target, remove processed waypoints from buffer
        if result == Result.Finished:
            # We processed: all intermediate waypoints + the target
            # That's (lookahead - 1) + 1 = lookahead waypoints total
            waypoints_to_remove = max_lookahead
            for _ in range(waypoints_to_remove):
                if self.waypoint_buffer:
                    self.waypoint_buffer.pop(0)

            self.get_logger().info(
                f'Reached target | Removed {waypoints_to_remove} waypoints | '
                f'Remaining: {len(self.waypoint_buffer)}'
            )

    def _publish_visualization_data(self):
        """Publish Ruckig state data for PlotJuggler visualization"""
        # Current position
        pos_msg = Float64MultiArray()
        pos_msg.data = list(self.output_param.new_position)
        self.viz_current_pos_pub.publish(pos_msg)

        # Current velocity
        vel_msg = Float64MultiArray()
        vel_msg.data = list(self.output_param.new_velocity)
        self.viz_current_vel_pub.publish(vel_msg)

        # Current acceleration
        acc_msg = Float64MultiArray()
        acc_msg.data = list(self.output_param.new_acceleration)
        self.viz_current_acc_pub.publish(acc_msg)

        # Target position
        target_msg = Float64MultiArray()
        target_msg.data = list(self.input_param.target_position)
        self.viz_target_pos_pub.publish(target_msg)

        # Buffer size
        buffer_msg = Int32()
        buffer_msg.data = len(self.waypoint_buffer)
        self.viz_buffer_size_pub.publish(buffer_msg)


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
