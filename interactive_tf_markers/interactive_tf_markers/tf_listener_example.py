#!/usr/bin/env python3

"""
Example node showing how other nodes can access the Target and Direction positions
through TF lookups (no custom subscribers needed!)
"""

import rclpy
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException
import math


class TFListenerExample(Node):
    def __init__(self):
        super().__init__('tf_listener_example')

        # Create TF buffer and listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Create timer to periodically check transforms
        self.timer = self.create_timer(1.0, self.timer_callback)

        self.get_logger().info('TF Listener Example started!')
        self.get_logger().info('This node demonstrates how to access Target and Direction positions')

    def timer_callback(self):
        """Periodically lookup and display the Target and Direction positions"""

        try:
            # Lookup Target position relative to world frame
            target_transform = self.tf_buffer.lookup_transform(
                'world',  # target frame
                'target',  # source frame
                rclpy.time.Time()  # get latest available
            )

            # Lookup Direction position relative to world frame
            direction_transform = self.tf_buffer.lookup_transform(
                'world',
                'direction',
                rclpy.time.Time()
            )

            # Extract positions
            target_pos = target_transform.transform.translation
            direction_pos = direction_transform.transform.translation

            # Calculate distance between them
            dx = target_pos.x - direction_pos.x
            dy = target_pos.y - direction_pos.y
            dz = target_pos.z - direction_pos.z
            distance = math.sqrt(dx*dx + dy*dy + dz*dz)

            self.get_logger().info('=' * 70)
            self.get_logger().info(
                f'Target:    pos=({target_pos.x:.2f}, {target_pos.y:.2f}, {target_pos.z:.2f})'
            )
            self.get_logger().info(
                f'Direction: pos=({direction_pos.x:.2f}, {direction_pos.y:.2f}, {direction_pos.z:.2f})'
            )
            self.get_logger().info(f'Distance between markers: {distance:.2f} meters')

            # You can also lookup transform FROM target TO direction
            relative_transform = self.tf_buffer.lookup_transform(
                'target',  # direction position relative to target
                'direction',
                rclpy.time.Time()
            )
            rel_pos = relative_transform.transform.translation
            self.get_logger().info(
                f'Direction relative to Target: ({rel_pos.x:.2f}, {rel_pos.y:.2f}, {rel_pos.z:.2f})'
            )

        except (LookupException, ConnectivityException, ExtrapolationException) as e:
            self.get_logger().warning(f'Could not lookup transform: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = TFListenerExample()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
