#!/usr/bin/env python3
"""
Main launch file for the complete system.
Launches interactive markers, trajectory tracker, FABRIK IK solver, and RViz with main config.
"""

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution
import os


def generate_launch_description():
    # Find package paths
    interactive_markers_pkg = FindPackageShare('interactive_tf_markers')
    trajectory_tracker_pkg = FindPackageShare('trajectory_tracker')
    fabrik_ik_solver_pkg = FindPackageShare('fabrik_ik_solver')

    # Get main RViz config path
    main_rviz_config = os.path.join(os.path.dirname(__file__), 'rviz', 'main.rviz')

    # Include interactive markers (WITHOUT RViz - we launch our own)
    interactive_markers_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                interactive_markers_pkg,
                'launch',
                'interactive_markers.launch.py'  # Changed to version without RViz
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

    # Include FABRIK IK solver
    fabrik_ik_solver_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                fabrik_ik_solver_pkg,
                'launch',
                'fabrik_ik_solver.launch.py'
            ])
        ])
    )

    # Launch RViz with main config
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', main_rviz_config],
        output='screen'
    )

    return LaunchDescription([
        interactive_markers_launch,
        trajectory_tracker_launch,
        fabrik_ik_solver_launch,
        rviz_node,
    ])
