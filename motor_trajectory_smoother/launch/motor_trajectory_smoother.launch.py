#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='motor_trajectory_smoother',
            executable='motor_trajectory_smoother_node',
            name='motor_trajectory_smoother_node',
            output='screen',
            parameters=[{
                'publish_rate': 100.0,  # Hz - smooth trajectory publishing rate
            }]
        ),
    ])
