#!/usr/bin/env python3
"""
Main launch file for the complete system.
Launches interactive markers with RViz and trajectory tracker.
"""

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution


def generate_launch_description():
    # Find package paths
    interactive_markers_pkg = FindPackageShare('interactive_tf_markers')
    trajectory_tracker_pkg = FindPackageShare('trajectory_tracker')

    # Include interactive markers with RViz
    interactive_markers_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                interactive_markers_pkg,
                'launch',
                'interactive_markers_with_rviz.launch.py'
            ])
        ])
    )

    # Include trajectory tracker
    trajectory_tracker_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                trajectory_tracker_pkg,
                'launch',
                'trajectory_tracker.launch.py'
            ])
        ])
    )

    return LaunchDescription([
        interactive_markers_launch,
        trajectory_tracker_launch,
    ])
