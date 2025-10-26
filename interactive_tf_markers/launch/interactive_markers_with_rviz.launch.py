from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # Get the path to the RViz config file
    pkg_dir = get_package_share_directory('interactive_tf_markers')
    rviz_config_file = os.path.join(pkg_dir, 'rviz', 'interactive_markers.rviz')

    return LaunchDescription([
        # Static transform for world frame (creates a fixed reference)
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

        # RViz2 with config
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_file],
            output='screen'
        ),
    ])
