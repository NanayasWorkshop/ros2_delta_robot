#!/usr/bin/env python3
"""
Circular Motion Generator
Broadcasts TF transforms to move the 'target' frame in a circular path.
Useful for testing continuous motion and Ruckig trajectory smoothing.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
import math


class CircularMotionGenerator(Node):
    def __init__(self):
        super().__init__('circular_motion_generator')

        # Parameters
        self.declare_parameter('radius', 0.15)  # Circle radius in meters
        self.declare_parameter('center_x', 0.15)  # Center X position
        self.declare_parameter('center_y', 0.0)   # Center Y position
        self.declare_parameter('center_z', 0.6)   # Center Z position
        self.declare_parameter('period', 10.0)    # Time for one full circle (seconds)
        self.declare_parameter('publish_rate', 50.0)  # Hz

        # Get parameters
        self.radius = self.get_parameter('radius').value
        self.center_x = self.get_parameter('center_x').value
        self.center_y = self.get_parameter('center_y').value
        self.center_z = self.get_parameter('center_z').value
        self.period = self.get_parameter('period').value
        self.publish_rate = self.get_parameter('publish_rate').value

        # TF broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)

        # Timer for publishing
        self.timer = self.create_timer(1.0 / self.publish_rate, self.publish_circular_motion)

        # Start time
        self.start_time = self.get_clock().now()

        self.get_logger().info('Circular Motion Generator started')
        self.get_logger().info(f'  Center: ({self.center_x:.3f}, {self.center_y:.3f}, {self.center_z:.3f})')
        self.get_logger().info(f'  Radius: {self.radius:.3f}m')
        self.get_logger().info(f'  Period: {self.period:.1f}s')
        self.get_logger().info(f'  Publish rate: {self.publish_rate}Hz')
        self.get_logger().info('  Target frame will move in XY plane')

    def publish_circular_motion(self):
        """Publish TF transform for circular motion"""
        # Calculate time elapsed
        current_time = self.get_clock().now()
        elapsed_sec = (current_time - self.start_time).nanoseconds / 1e9

        # Calculate angle (radians) - one full rotation every 'period' seconds
        angle = 2.0 * math.pi * (elapsed_sec / self.period)

        # Calculate position on circle (in XY plane)
        x = self.center_x + self.radius * math.cos(angle)
        y = self.center_y + self.radius * math.sin(angle)
        z = self.center_z

        # Create TF message
        t = TransformStamped()
        t.header.stamp = current_time.to_msg()
        t.header.frame_id = 'world'
        t.child_frame_id = 'target'

        # Set position
        t.transform.translation.x = x
        t.transform.translation.y = y
        t.transform.translation.z = z

        # Set rotation (identity - no rotation)
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = 0.0
        t.transform.rotation.w = 1.0

        # Broadcast transform
        self.tf_broadcaster.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    node = CircularMotionGenerator()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
