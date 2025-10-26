#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import Int32
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker, MarkerArray
from trajectory_tracker.msg import Trajectory
from fabrik_ik_solver.msg import MotorCommand, FabrikVisualization
import numpy as np
import threading

from fabrik import FabrikSolver
from robot_config import physical as phys_config
from robot_config import motion as motion_config
from robot_config import visualization as viz_config
from robot_config import system as sys_config


class FabrikIKSolverNode(Node):
    def __init__(self):
        super().__init__('fabrik_ik_solver_node')

        # Declare parameters
        self.declare_parameter('num_segments', phys_config.NUM_SEGMENTS)
        self.declare_parameter('tolerance', motion_config.FABRIK_TOLERANCE)
        self.declare_parameter('max_iterations', motion_config.FABRIK_MAX_ITERATIONS)
        self.declare_parameter('enable_visualization', True)
        self.declare_parameter('use_hot_start', True)

        # Get parameters
        num_segments = self.get_parameter('num_segments').value
        tolerance = self.get_parameter('tolerance').value
        max_iterations = self.get_parameter('max_iterations').value
        self.enable_viz = self.get_parameter('enable_visualization').value
        self.use_hot_start = self.get_parameter('use_hot_start').value

        # Initialize FABRIK solver
        self.fabrik = FabrikSolver(
            num_segments=num_segments,
            default_tolerance=tolerance,
            default_max_iterations=max_iterations
        )

        # Trajectory buffer management with thread safety
        self.processing_lock = threading.Lock()
        self.processing_point = False
        self.current_target_frame = None
        self.current_approach_frame = None

        # Track last approach point to detect changes
        self.last_approach_point = None

        # Motor command history
        self.motor_command_history = []

        # Create reentrant callback group for parallel processing
        self.callback_group = ReentrantCallbackGroup()

        # Subscribers with callback group
        self.trajectory_sub = self.create_subscription(
            Trajectory,
            sys_config.TOPIC_TRAJECTORY,
            self.trajectory_callback,
            sys_config.QUEUE_SIZE_DEFAULT,
            callback_group=self.callback_group
        )

        # Publishers
        self.motor_pub = self.create_publisher(
            MotorCommand,
            sys_config.TOPIC_MOTOR_COMMANDS,
            sys_config.QUEUE_SIZE_DEFAULT
        )
        self.viz_pub = self.create_publisher(
            FabrikVisualization,
            sys_config.TOPIC_FABRIK_VISUALIZATION,
            sys_config.QUEUE_SIZE_DEFAULT
        )
        self.marker_pub = self.create_publisher(
            MarkerArray,
            sys_config.TOPIC_FABRIK_MARKERS,
            sys_config.QUEUE_SIZE_DEFAULT
        )
        self.clear_oldest_pub = self.create_publisher(
            Int32,
            sys_config.TOPIC_CLEAR_OLDEST,
            sys_config.QUEUE_SIZE_DEFAULT
        )
        # Publisher for motor positions visualization (PlotJuggler)
        from std_msgs.msg import Float64MultiArray
        self.fabrik_motor_pos_pub = self.create_publisher(
            Float64MultiArray,
            sys_config.TOPIC_FABRIK_MOTOR_POSITIONS,
            sys_config.QUEUE_SIZE_DEFAULT
        )

        self.get_logger().info(f'FABRIK IK Solver started')
        self.get_logger().info(f'  Segments: {num_segments}')
        self.get_logger().info(f'  Tolerance: {tolerance}m')
        self.get_logger().info(f'  Max iterations: {max_iterations}')
        self.get_logger().info(f'  Visualization: {self.enable_viz}')
        self.get_logger().info(f'  Hot start: {self.use_hot_start}')

    def trajectory_callback(self, msg: Trajectory):
        """Process trajectory points one by one"""

        # Thread-safe check if already processing
        with self.processing_lock:
            if self.processing_point or msg.total_count == 0:
                return
            # Mark as processing immediately to prevent other threads
            self.processing_point = True

        try:
            # Get the oldest point (first in the list)
            oldest_point = msg.points[0]

            # Check if we have both target and direction frames
            # We need to find a target point and optionally a direction point
            target_point = None
            approach_point = None

            # Look for target and direction frames in the trajectory
            for point in msg.points:
                if point.frame_name == sys_config.FRAME_TARGET:
                    target_point = point
                elif point.frame_name == sys_config.FRAME_DIRECTION:
                    approach_point = point

            # Only process if we have a target
            if target_point is None:
                with self.processing_lock:
                    self.processing_point = False
                return

            # Extract positions
            target = np.array([
                target_point.pose.position.x,
                target_point.pose.position.y,
                target_point.pose.position.z
            ])

            approach = None
            if approach_point is not None:
                approach = np.array([
                    approach_point.pose.position.x,
                    approach_point.pose.position.y,
                    approach_point.pose.position.z
                ])

            # Check if approach point changed - add tiny offset to invalidate hot start convergence
            approach_changed = False
            if self.last_approach_point is not None and approach is not None:
                approach_changed = np.linalg.norm(approach - self.last_approach_point) > motion_config.FABRIK_APPROACH_CHANGE_THRESHOLD
                if approach_changed:
                    # Add 0.1mm offset to target to force FABRIK to iterate
                    # This ensures approach constraint is applied even when target hasn't moved
                    target = target + np.array([1e-4, 0, 0])
                    self.get_logger().info(f'  Approach changed! Added 0.1mm offset to target')

            # Store current approach point for next comparison
            self.last_approach_point = approach.copy() if approach is not None else None

            # Log hot start status
            hot_start_active = self.fabrik.has_previous_solution and self.use_hot_start
            self.get_logger().info(
                f'Solving IK for target: ({target[0]:.3f}, {target[1]:.3f}, {target[2]:.3f})'
            )
            self.get_logger().info(
                f'  Hot start: {hot_start_active}, Approach changed: {approach_changed}'
            )

            # Solve IK
            result = self.fabrik.solve(
                target=target,
                approach_point=approach,
                use_hot_start=self.use_hot_start
            )

            # Log result
            if result['converged']:
                self.get_logger().info(
                    f'  ✓ Converged in {result["iterations"]} iterations, error: {result["final_error"]*1000:.3f}mm'
                )
            else:
                self.get_logger().warn(
                    f'  ✗ Did not converge after {result["iterations"]} iterations, error: {result["final_error"]*1000:.3f}mm'
                )

            # Publish motor commands
            self.publish_motor_commands(result, target_point.timestamp)

            # Publish visualization if enabled
            if self.enable_viz:
                self.publish_visualization(result, target, approach, target_point.timestamp)

            # Store in history
            self.motor_command_history.append(result['motor_positions'])

            # Delete the processed point(s) from trajectory
            # Delete both target and direction if both exist
            num_to_delete = 2 if approach_point is not None else 1
            self.clear_trajectory_point(num_to_delete)

        finally:
            # Always mark as done processing, even if there was an error
            with self.processing_lock:
                self.processing_point = False

    def publish_motor_commands(self, result, timestamp):
        """Publish motor command message"""
        msg = MotorCommand()
        msg.timestamp = timestamp
        msg.converged = bool(result['converged'])  # Convert to Python bool
        msg.iterations = int(result['iterations'])  # Convert to Python int
        msg.final_error = float(result['final_error'])  # Convert to Python float
        msg.motor_positions = [float(x) for x in result['motor_positions']]  # Convert to Python floats
        msg.joint_angles = [float(x) for x in result['joint_angles']]  # Convert to Python floats

        self.motor_pub.publish(msg)

        # Publish motor positions for visualization
        from std_msgs.msg import Float64MultiArray
        motor_viz_msg = Float64MultiArray()
        motor_viz_msg.data = [float(x) for x in result['motor_positions']]
        self.fabrik_motor_pos_pub.publish(motor_viz_msg)

        # Log motor positions summary
        self.get_logger().info(
            f'  Motor positions: {len(result["motor_positions"])} values (8 segments × 3 motors)'
        )

    def publish_visualization(self, result, target, approach, timestamp):
        """Publish FABRIK visualization data"""
        # Publish raw data message
        viz_msg = FabrikVisualization()
        viz_msg.timestamp = timestamp

        # Convert numpy arrays to Point messages
        for s_point in result['s_points']:
            p = Point()
            p.x, p.y, p.z = float(s_point[0]), float(s_point[1]), float(s_point[2])
            viz_msg.s_points.append(p)

        for j_point in result['j_points']:
            p = Point()
            p.x, p.y, p.z = float(j_point[0]), float(j_point[1]), float(j_point[2])
            viz_msg.j_points.append(p)

        # Target
        viz_msg.target.x, viz_msg.target.y, viz_msg.target.z = float(target[0]), float(target[1]), float(target[2])

        # Approach point (if exists)
        if approach is not None:
            viz_msg.has_approach_point = True
            viz_msg.approach_point.x = float(approach[0])
            viz_msg.approach_point.y = float(approach[1])
            viz_msg.approach_point.z = float(approach[2])
        else:
            viz_msg.has_approach_point = False

        self.viz_pub.publish(viz_msg)

        # Publish MarkerArray for RViz visualization
        self.publish_markers(result['s_points'], result['j_points'])

    def publish_markers(self, s_points, j_points):
        """Publish visualization markers for RViz"""
        marker_array = MarkerArray()

        # S-points as blue spheres
        for i, s_point in enumerate(s_points):
            marker = Marker()
            marker.header.frame_id = sys_config.FRAME_WORLD
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = 's_points'
            marker.id = i
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose.position.x = float(s_point[0])
            marker.pose.position.y = float(s_point[1])
            marker.pose.position.z = float(s_point[2])
            marker.pose.orientation.w = 1.0
            marker.scale.x = viz_config.FABRIK_S_POINT_SIZE
            marker.scale.y = viz_config.FABRIK_S_POINT_SIZE
            marker.scale.z = viz_config.FABRIK_S_POINT_SIZE
            marker.color.r = 0.0
            marker.color.g = 0.0
            marker.color.b = 1.0  # Blue
            marker.color.a = 0.8
            marker_array.markers.append(marker)

        # J-points as green spheres
        for i, j_point in enumerate(j_points):
            marker = Marker()
            marker.header.frame_id = sys_config.FRAME_WORLD
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = 'j_points'
            marker.id = i
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose.position.x = float(j_point[0])
            marker.pose.position.y = float(j_point[1])
            marker.pose.position.z = float(j_point[2])
            marker.pose.orientation.w = 1.0
            marker.scale.x = viz_config.FABRIK_J_POINT_SIZE
            marker.scale.y = viz_config.FABRIK_J_POINT_SIZE
            marker.scale.z = viz_config.FABRIK_J_POINT_SIZE
            marker.color.r = 0.0
            marker.color.g = 1.0  # Green
            marker.color.b = 0.0
            marker.color.a = 0.8
            marker_array.markers.append(marker)

        # Line strip connecting S-points (blue)
        s_line_marker = Marker()
        s_line_marker.header.frame_id = sys_config.FRAME_WORLD
        s_line_marker.header.stamp = self.get_clock().now().to_msg()
        s_line_marker.ns = 's_chain'
        s_line_marker.id = 0
        s_line_marker.type = Marker.LINE_STRIP
        s_line_marker.action = Marker.ADD
        s_line_marker.scale.x = 0.005  # 5mm line width
        s_line_marker.color.r = 0.0
        s_line_marker.color.g = 0.0
        s_line_marker.color.b = 1.0  # Blue
        s_line_marker.color.a = 0.6

        for s_point in s_points:
            p = Point()
            p.x = float(s_point[0])
            p.y = float(s_point[1])
            p.z = float(s_point[2])
            s_line_marker.points.append(p)

        marker_array.markers.append(s_line_marker)

        # Line strip connecting J-points (green)
        j_line_marker = Marker()
        j_line_marker.header.frame_id = sys_config.FRAME_WORLD
        j_line_marker.header.stamp = self.get_clock().now().to_msg()
        j_line_marker.ns = 'j_chain'
        j_line_marker.id = 0
        j_line_marker.type = Marker.LINE_STRIP
        j_line_marker.action = Marker.ADD
        j_line_marker.scale.x = 0.005  # 5mm line width
        j_line_marker.color.r = 0.0
        j_line_marker.color.g = 1.0  # Green
        j_line_marker.color.b = 0.0
        j_line_marker.color.a = 0.6

        for j_point in j_points:
            p = Point()
            p.x = float(j_point[0])
            p.y = float(j_point[1])
            p.z = float(j_point[2])
            j_line_marker.points.append(p)

        marker_array.markers.append(j_line_marker)

        self.marker_pub.publish(marker_array)

    def clear_trajectory_point(self, count=1):
        """Request deletion of oldest point(s) from trajectory tracker"""
        msg = Int32()
        msg.data = count
        self.clear_oldest_pub.publish(msg)
        self.get_logger().info(f'  Deleted {count} point(s) from trajectory')


def main(args=None):
    rclpy.init(args=args)
    node = FabrikIKSolverNode()

    # Use MultiThreadedExecutor for parallel callback processing
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
