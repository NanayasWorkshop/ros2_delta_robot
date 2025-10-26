#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener, LookupException, ConnectivityException, ExtrapolationException
from geometry_msgs.msg import Pose
from std_msgs.msg import Empty, Int32
from trajectory_tracker.msg import TrajectoryPoint, Trajectory


class TrajectoryTrackerNode(Node):
    def __init__(self):
        super().__init__('trajectory_tracker_node')

        # Declare parameters
        self.declare_parameter('tracked_frames', ['target', 'direction'])
        self.declare_parameter('reference_frame', 'world')
        self.declare_parameter('sample_rate', 5.0)

        # Get parameters
        self.tracked_frames = self.get_parameter('tracked_frames').value
        self.reference_frame = self.get_parameter('reference_frame').value
        sample_rate = self.get_parameter('sample_rate').value

        # TF setup
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Trajectory storage (infinite growth)
        self.trajectory_points = []

        # Track last known positions to detect changes
        self.last_positions = {}  # {frame_name: (x, y, z)}

        # Publishers
        self.trajectory_pub = self.create_publisher(
            Trajectory,
            '/trajectory',
            10
        )
        self.status_pub = self.create_publisher(
            Int32,
            '/trajectory_status',
            10
        )

        # Subscribers for control
        self.clear_sub = self.create_subscription(
            Empty,
            '/clear_trajectory',
            self.clear_callback,
            10
        )
        self.clear_oldest_sub = self.create_subscription(
            Int32,
            '/clear_oldest',
            self.clear_oldest_callback,
            10
        )

        # Timer for periodic sampling
        timer_period = 1.0 / sample_rate  # Convert Hz to seconds
        self.timer = self.create_timer(timer_period, self.sample_callback)

        self.get_logger().info(f'Trajectory Tracker started')
        self.get_logger().info(f'  Tracking frames: {self.tracked_frames}')
        self.get_logger().info(f'  Reference frame: {self.reference_frame}')
        self.get_logger().info(f'  Sample rate: {sample_rate} Hz')

    def sample_callback(self):
        """Sample TF transforms at fixed rate and add to trajectory only if position changed"""
        timestamp = self.get_clock().now().to_msg()
        any_changed = False

        for frame_name in self.tracked_frames:
            try:
                # Lookup transform
                transform = self.tf_buffer.lookup_transform(
                    self.reference_frame,
                    frame_name,
                    rclpy.time.Time()
                )

                # Get current position
                current_pos = (
                    transform.transform.translation.x,
                    transform.transform.translation.y,
                    transform.transform.translation.z
                )

                # Check if position changed
                if frame_name in self.last_positions:
                    last_pos = self.last_positions[frame_name]
                    # Compare with small tolerance for floating point
                    if (abs(current_pos[0] - last_pos[0]) < 1e-6 and
                        abs(current_pos[1] - last_pos[1]) < 1e-6 and
                        abs(current_pos[2] - last_pos[2]) < 1e-6):
                        # Position hasn't changed, skip recording
                        continue

                # Position changed or first time seeing this frame
                self.last_positions[frame_name] = current_pos
                any_changed = True

                # Create trajectory point
                point = TrajectoryPoint()
                point.timestamp = timestamp
                point.frame_name = frame_name

                # Convert transform to pose
                point.pose = Pose()
                point.pose.position.x = current_pos[0]
                point.pose.position.y = current_pos[1]
                point.pose.position.z = current_pos[2]
                point.pose.orientation = transform.transform.rotation

                # Add to trajectory
                self.trajectory_points.append(point)

            except (LookupException, ConnectivityException, ExtrapolationException) as e:
                self.get_logger().warning(f'TF lookup failed for {frame_name}: {e}', throttle_duration_sec=1.0)

        # Publish trajectory
        self.publish_trajectory()

        # Publish status
        status_msg = Int32()
        status_msg.data = len(self.trajectory_points)
        self.status_pub.publish(status_msg)

    def publish_trajectory(self):
        """Publish the entire trajectory"""
        msg = Trajectory()
        msg.header_stamp = self.get_clock().now().to_msg()
        msg.points = self.trajectory_points
        msg.total_count = len(self.trajectory_points)
        self.trajectory_pub.publish(msg)

    def clear_callback(self, msg):
        """Clear all trajectory points"""
        count = len(self.trajectory_points)
        self.trajectory_points.clear()
        self.get_logger().info(f'Cleared {count} trajectory points')
        self.publish_trajectory()  # Publish empty trajectory

    def clear_oldest_callback(self, msg):
        """Clear N oldest points"""
        n = msg.data
        if n > 0:
            removed = min(n, len(self.trajectory_points))
            self.trajectory_points = self.trajectory_points[removed:]
            self.get_logger().info(f'Removed {removed} oldest trajectory points')
            self.publish_trajectory()


def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryTrackerNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
