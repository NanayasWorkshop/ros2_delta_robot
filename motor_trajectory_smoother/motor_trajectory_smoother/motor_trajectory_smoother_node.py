#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from fabrik_ik_solver.msg import MotorCommand
from std_msgs.msg import Float64MultiArray, Int32
import numpy as np
from enum import Enum

from robot_config import physical as phys_config
from robot_config import motion as motion_config
from robot_config import system as sys_config

# Import Ruckig
try:
    from ruckig import Ruckig, InputParameter, OutputParameter, Result, Synchronization, ControlInterface
except ImportError:
    raise ImportError("Ruckig library not found. Install with: sudo apt install ros-humble-ruckig")


class SmootherState(Enum):
    """State machine for batch trajectory processing"""
    RECORDING = 1   # Recording motor commands from FABRIK
    PROCESSING = 2  # Analyzing recorded data for peaks/valleys
    EXECUTING = 3   # Executing smooth trajectory with Ruckig


class MotorTrajectorySmootherNode(Node):
    def __init__(self):
        super().__init__('motor_trajectory_smoother_node')

        # Declare parameters
        self.declare_parameter('publish_rate', motion_config.RUCKIG_PUBLISH_RATE)
        self.declare_parameter('rest_timeout', 1.0)  # Seconds to wait for rest detection

        # Get parameters
        self.publish_rate = self.get_parameter('publish_rate').value
        self.rest_timeout = self.get_parameter('rest_timeout').value

        # Initialize Ruckig for 24 motors
        self.ruckig = Ruckig(phys_config.NUM_MOTORS, motion_config.RUCKIG_DELTA_TIME)
        self.input_param = InputParameter(phys_config.NUM_MOTORS)
        self.output_param = OutputParameter(phys_config.NUM_MOTORS)

        # Configure Ruckig parameters
        self._configure_ruckig_parameters()

        # State machine
        self.state = SmootherState.RECORDING

        # Recording buffer for batch processing
        self.recorded_commands = []  # List of MotorCommand messages
        self.last_command_time = None

        # Execution buffer (compressed waypoints from peak/valley detection)
        self.execution_buffer = []

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

        # Timer for rest detection and trajectory execution
        self.timer = self.create_timer(
            1.0 / self.publish_rate,
            self.timer_callback
        )

        self.get_logger().info('Motor Trajectory Smoother started (BATCH MODE)')
        self.get_logger().info(f'  Motors: {phys_config.NUM_MOTORS}')
        self.get_logger().info(f'  Publish rate: {self.publish_rate} Hz')
        self.get_logger().info(f'  Ruckig delta_time: {motion_config.RUCKIG_DELTA_TIME}s')
        self.get_logger().info(f'  Rest timeout: {self.rest_timeout}s')
        self.get_logger().info(f'  Max velocity: {motion_config.MOTOR_MAX_VELOCITY} m/s')
        self.get_logger().info(f'  Max acceleration: {motion_config.MOTOR_MAX_ACCELERATION} m/s²')
        self.get_logger().info(f'  Max jerk: {motion_config.MOTOR_MAX_JERK} m/s³')

    def _configure_ruckig_parameters(self):
        """Configure Ruckig input parameters"""
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
        if self.state == SmootherState.RECORDING:
            # Add to recording buffer
            self.recorded_commands.append(msg)
            self.last_command_time = self.get_clock().now()

            self.get_logger().info(
                f'[RECORDING] Buffered waypoint | Total: {len(self.recorded_commands)}',
                throttle_duration_sec=0.5
            )
        else:
            # Ignore new commands during processing or execution
            self.get_logger().info(
                f'[{self.state.name}] Ignoring new waypoint',
                throttle_duration_sec=1.0
            )

    def timer_callback(self):
        """Main state machine - handles rest detection and trajectory execution"""

        if self.state == SmootherState.RECORDING:
            self._handle_recording_state()
        elif self.state == SmootherState.PROCESSING:
            self._handle_processing_state()
        elif self.state == SmootherState.EXECUTING:
            self._handle_executing_state()

    def _handle_recording_state(self):
        """Check for rest timeout and trigger batch processing"""
        # Need at least one command to check timeout
        if not self.recorded_commands or self.last_command_time is None:
            return

        # Check if we've been at rest for rest_timeout seconds
        time_since_last = (self.get_clock().now() - self.last_command_time).nanoseconds / 1e9

        if time_since_last >= self.rest_timeout:
            # Marker came to rest - trigger batch processing
            self.get_logger().info(
                f'[RECORDING] ✓ Rest detected ({time_since_last:.2f}s) | '
                f'Recorded {len(self.recorded_commands)} waypoints'
            )
            self.state = SmootherState.PROCESSING

    def _handle_processing_state(self):
        """Analyze recorded data and extract keyframes (peaks/valleys)"""
        if len(self.recorded_commands) < 2:
            self.get_logger().warn('[PROCESSING] Not enough waypoints, returning to RECORDING')
            self.state = SmootherState.RECORDING
            self.recorded_commands.clear()
            return

        self.get_logger().info(f'[PROCESSING] Analyzing {len(self.recorded_commands)} waypoints...')

        # Detect peaks and valleys
        keyframe_indices = self._detect_keyframes()

        # Extract keyframe commands
        self.execution_buffer = [self.recorded_commands[i] for i in keyframe_indices]

        self.get_logger().info(
            f'[PROCESSING] ✓ Found {len(keyframe_indices)} keyframes | '
            f'Compression: {len(self.recorded_commands)} → {len(self.execution_buffer)} '
            f'({100*(1-len(self.execution_buffer)/len(self.recorded_commands)):.1f}% reduction)'
        )

        # Clear recording buffer
        self.recorded_commands.clear()

        # Transition to execution
        self.state = SmootherState.EXECUTING
        self.ruckig_initialized = False  # Reset for new trajectory

    def _detect_keyframes(self):
        """
        Detect peaks and valleys in motor position trajectories.
        Returns list of keyframe indices.
        """
        keyframes = set()

        # Always include first and last
        keyframes.add(0)
        keyframes.add(len(self.recorded_commands) - 1)

        # For each motor, find peaks and valleys
        for motor_idx in range(phys_config.NUM_MOTORS):
            # Extract time series for this motor
            trajectory = [cmd.motor_positions[motor_idx] for cmd in self.recorded_commands]

            # Find local maxima and minima
            for i in range(1, len(trajectory) - 1):
                prev_val = trajectory[i-1]
                curr_val = trajectory[i]
                next_val = trajectory[i+1]

                # Peak: curr > prev AND curr > next
                if curr_val > prev_val and curr_val > next_val:
                    keyframes.add(i)

                # Valley: curr < prev AND curr < next
                elif curr_val < prev_val and curr_val < next_val:
                    keyframes.add(i)

        return sorted(list(keyframes))

    def _handle_executing_state(self):
        """Execute smooth trajectory using Ruckig with compressed waypoints"""
        # If no waypoints, return to recording
        if not self.execution_buffer:
            self.get_logger().info('[EXECUTING] ✓ Trajectory complete, returning to RECORDING')
            self.state = SmootherState.RECORDING
            self.last_command_time = None
            return

        # Initialize Ruckig state with first waypoint if not yet initialized
        if not self.ruckig_initialized:
            first_waypoint = self.execution_buffer[0]
            # Initialize current state - assign whole lists!
            self.input_param.current_position = list(first_waypoint.motor_positions)
            self.input_param.current_velocity = [motion_config.RUCKIG_INITIAL_VELOCITY] * phys_config.NUM_MOTORS
            self.input_param.current_acceleration = [motion_config.RUCKIG_INITIAL_ACCELERATION] * phys_config.NUM_MOTORS
            self.ruckig_initialized = True
            self.execution_buffer.pop(0)  # Remove initialization waypoint
            self.get_logger().info(f'[EXECUTING] Ruckig initialized | Remaining waypoints: {len(self.execution_buffer)}')
            return  # Wait for next cycle

        # Ruckig waypoint logic with look-ahead
        max_lookahead = min(motion_config.RUCKIG_MAX_LOOKAHEAD_WAYPOINTS, len(self.execution_buffer))

        if max_lookahead == 1:
            # Only 1 waypoint left → this is the final target, stop here
            self.input_param.target_position = list(self.execution_buffer[0].motor_positions)
            self.input_param.intermediate_positions = []
            target_vel = motion_config.RUCKIG_FINAL_TARGET_VELOCITY  # Stop
            target_acc = motion_config.RUCKIG_TARGET_ACCELERATION
            target_waypoint = self.execution_buffer[0]
        else:
            # Multiple waypoints → cruise through buffer[0...N-2], target is buffer[N-1]
            self.input_param.intermediate_positions = []
            for i in range(0, max_lookahead - 1):
                self.input_param.intermediate_positions.append(
                    list(self.execution_buffer[i].motor_positions)
                )

            # Set target position (final point in lookahead)
            target_idx = max_lookahead - 1
            self.input_param.target_position = list(self.execution_buffer[target_idx].motor_positions)
            target_waypoint = self.execution_buffer[target_idx]

            # Keep moving - don't stop at target since more waypoints may come
            target_vel = 0.0  # Let Ruckig optimize
            target_acc = 0.0

        # Assign target velocity and acceleration - whole lists!
        self.input_param.target_velocity = [target_vel] * phys_config.NUM_MOTORS
        self.input_param.target_acceleration = [target_acc] * phys_config.NUM_MOTORS

        # Update Ruckig trajectory
        result = self.ruckig.update(self.input_param, self.output_param)

        if result == Result.Error or result == Result.ErrorInvalidInput:
            self.get_logger().error(f'[EXECUTING] Ruckig error: {result}')
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

        # Publish visualization data for PlotJuggler
        self._publish_visualization_data()

        # Pass output to input for next cycle
        self.output_param.pass_to_input(self.input_param)

        # If we reached the target, remove processed waypoints from buffer
        if result == Result.Finished:
            waypoints_to_remove = max_lookahead
            for _ in range(waypoints_to_remove):
                if self.execution_buffer:
                    self.execution_buffer.pop(0)

            self.get_logger().info(
                f'[EXECUTING] Reached target | Removed {waypoints_to_remove} waypoints | '
                f'Remaining: {len(self.execution_buffer)}'
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

        # Buffer size (execution buffer during execution, recorded during recording)
        buffer_msg = Int32()
        if self.state == SmootherState.EXECUTING:
            buffer_msg.data = len(self.execution_buffer)
        else:
            buffer_msg.data = len(self.recorded_commands)
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
