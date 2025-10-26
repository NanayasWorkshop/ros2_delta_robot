#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from interactive_markers.interactive_marker_server import InteractiveMarkerServer
from visualization_msgs.msg import InteractiveMarker, InteractiveMarkerControl, Marker
from geometry_msgs.msg import Point, Pose, Quaternion, TransformStamped
from tf2_ros import TransformBroadcaster
import math


class InteractiveMarkerTFNode(Node):
    def __init__(self):
        super().__init__('interactive_marker_tf_node')

        # Create TF broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)

        # Store marker poses for continuous TF broadcasting
        self.marker_poses = {
            'target': Pose(),
            'direction': Pose()
        }

        # Create interactive marker server
        self.server = InteractiveMarkerServer(self, 'interactive_markers')

        # Create the two markers (positions in meters: mm/1000)
        self.create_marker('target', [0.150, 0.0, 0.600], [1.0, 1.0, 0.0], 0.06)  # Yellow marker, 60mm at 150,0,600mm
        self.create_marker('direction', [0.0, 0.0, 0.600], [0.0, 1.0, 1.0], 0.04)  # Cyan marker, 40mm at 0,0,600mm

        # Apply changes
        self.server.applyChanges()

        # Create timer to continuously broadcast TF (10Hz)
        self.create_timer(0.1, self.broadcast_tf_timer)

        self.get_logger().info('Interactive Marker TF Node started!')
        self.get_logger().info('Use the "Interact" tool in RViz to move Target and Direction markers')
        self.get_logger().info('TF frames: "target" and "direction" are being broadcast')

    def create_marker(self, name, position, color, size):
        """Create an interactive marker with 6DOF controls"""

        # Create the interactive marker
        int_marker = InteractiveMarker()
        int_marker.header.frame_id = 'world'
        int_marker.name = name
        int_marker.description = name.capitalize()
        int_marker.scale = 0.15  # Make controls smaller (default is 1.0)

        # Set initial position
        int_marker.pose.position.x = position[0]
        int_marker.pose.position.y = position[1]
        int_marker.pose.position.z = position[2]
        int_marker.pose.orientation.w = 1.0

        # Store initial pose
        self.marker_poses[name] = Pose()
        self.marker_poses[name].position.x = position[0]
        self.marker_poses[name].position.y = position[1]
        self.marker_poses[name].position.z = position[2]
        self.marker_poses[name].orientation.w = 1.0

        # Create the marker visual (sphere)
        marker = Marker()
        marker.type = Marker.SPHERE
        marker.scale.x = size
        marker.scale.y = size
        marker.scale.z = size
        marker.color.r = color[0]
        marker.color.g = color[1]
        marker.color.b = color[2]
        marker.color.a = 0.8

        # Create a control to hold the marker
        marker_control = InteractiveMarkerControl()
        marker_control.always_visible = True
        marker_control.markers.append(marker)
        marker_control.interaction_mode = InteractiveMarkerControl.MOVE_ROTATE_3D
        int_marker.controls.append(marker_control)

        # Add 6DOF controls (3 arrows for translation, 3 rings for rotation)

        # X axis control (red arrow)
        control = InteractiveMarkerControl()
        control.orientation.w = 1.0
        control.orientation.x = 1.0
        control.orientation.y = 0.0
        control.orientation.z = 0.0
        control.name = 'move_x'
        control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
        int_marker.controls.append(control)

        # Rotation around X
        control = InteractiveMarkerControl()
        control.orientation.w = 1.0
        control.orientation.x = 1.0
        control.orientation.y = 0.0
        control.orientation.z = 0.0
        control.name = 'rotate_x'
        control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
        int_marker.controls.append(control)

        # Y axis control (green arrow)
        control = InteractiveMarkerControl()
        control.orientation.w = 1.0
        control.orientation.x = 0.0
        control.orientation.y = 1.0
        control.orientation.z = 0.0
        control.name = 'move_y'
        control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
        int_marker.controls.append(control)

        # Rotation around Y
        control = InteractiveMarkerControl()
        control.orientation.w = 1.0
        control.orientation.x = 0.0
        control.orientation.y = 1.0
        control.orientation.z = 0.0
        control.name = 'rotate_y'
        control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
        int_marker.controls.append(control)

        # Z axis control (blue arrow)
        control = InteractiveMarkerControl()
        control.orientation.w = 1.0
        control.orientation.x = 0.0
        control.orientation.y = 0.0
        control.orientation.z = 1.0
        control.name = 'move_z'
        control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
        int_marker.controls.append(control)

        # Rotation around Z
        control = InteractiveMarkerControl()
        control.orientation.w = 1.0
        control.orientation.x = 0.0
        control.orientation.y = 0.0
        control.orientation.z = 1.0
        control.name = 'rotate_z'
        control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
        int_marker.controls.append(control)

        # Add the marker to server and set callback
        self.server.insert(int_marker, feedback_callback=self.marker_feedback)

        # Broadcast initial TF
        self.broadcast_tf(name, int_marker.pose)

    def marker_feedback(self, feedback):
        """Callback when marker is moved - stores pose for TF broadcasting"""
        marker_name = feedback.marker_name
        pose = feedback.pose

        # Store updated pose (timer will broadcast it)
        self.marker_poses[marker_name] = pose

        # Log position for debugging
        self.get_logger().info(
            f'{marker_name}: pos=({pose.position.x:.2f}, {pose.position.y:.2f}, {pose.position.z:.2f}) '
            f'ori=({pose.orientation.x:.2f}, {pose.orientation.y:.2f}, {pose.orientation.z:.2f}, {pose.orientation.w:.2f})'
        )

        # Update the marker (required to keep it synchronized)
        self.server.applyChanges()

    def broadcast_tf(self, frame_name, pose):
        """Broadcast TF transform for the marker"""
        t = TransformStamped()

        # Set timestamp and frame info
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'world'
        t.child_frame_id = frame_name

        # Set translation
        t.transform.translation.x = pose.position.x
        t.transform.translation.y = pose.position.y
        t.transform.translation.z = pose.position.z

        # Set rotation
        t.transform.rotation.x = pose.orientation.x
        t.transform.rotation.y = pose.orientation.y
        t.transform.rotation.z = pose.orientation.z
        t.transform.rotation.w = pose.orientation.w

        # Broadcast the transform
        self.tf_broadcaster.sendTransform(t)

    def broadcast_tf_timer(self):
        """Timer callback to continuously broadcast TF for all markers"""
        for frame_name, pose in self.marker_poses.items():
            self.broadcast_tf(frame_name, pose)


def main(args=None):
    rclpy.init(args=args)
    node = InteractiveMarkerTFNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
