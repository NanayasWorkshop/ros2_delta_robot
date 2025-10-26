#!/usr/bin/env python3
"""
Launch file for FABRIK IK Solver
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='fabrik_ik_solver',
            executable='fabrik_ik_solver_node',
            name='fabrik_ik_solver_node',
            output='screen',
            parameters=[{
                'num_segments': 8,
                'tolerance': 0.001,  # 1mm
                'max_iterations': 50,
                'enable_visualization': True,
                'use_hot_start': True
            }]
        )
    ])
