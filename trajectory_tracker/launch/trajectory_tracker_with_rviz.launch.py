from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """
    Launch trajectory tracker with RViz (for consistency).
    Note: trajectory_tracker doesn't require RViz visualization,
    but this file exists for architectural consistency.
    """
    return LaunchDescription([
        Node(
            package='trajectory_tracker',
            executable='trajectory_tracker_node',
            name='trajectory_tracker_node',
            output='screen',
            parameters=[{
                'tracked_frames': ['target', 'direction'],
                'reference_frame': 'world',
                'sample_rate': 5.0
            }]
        ),
    ])
