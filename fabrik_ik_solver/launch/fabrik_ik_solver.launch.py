#!/usr/bin/env python3
"""
Launch file for FABRIK IK Solver
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    # Import robot_config to get dynamic NUM_SEGMENTS
    from robot_config import physical as phys_config
    from robot_config import motion as motion_config

    return LaunchDescription([
        Node(
            package='fabrik_ik_solver',
            executable='fabrik_ik_solver_node',
            name='fabrik_ik_solver_node',
            output='screen',
            parameters=[{
                'num_segments': phys_config.NUM_SEGMENTS,
                'tolerance': motion_config.FABRIK_TOLERANCE,
                'max_iterations': motion_config.FABRIK_MAX_ITERATIONS,
                'enable_visualization': motion_config.FABRIK_ENABLE_VISUALIZATION,
                'use_hot_start': motion_config.FABRIK_USE_HOT_START
            }]
        )
    ])
