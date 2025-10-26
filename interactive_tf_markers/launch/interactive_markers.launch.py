from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        # Static transform for world frame (optional, creates a fixed reference)
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='world_frame',
            arguments=['0', '0', '0', '0', '0', '0', 'map', 'world']
        ),

        # Interactive marker node
        Node(
            package='interactive_tf_markers',
            executable='interactive_marker_tf_node',
            name='interactive_marker_tf_node',
            output='screen'
        ),
    ])
