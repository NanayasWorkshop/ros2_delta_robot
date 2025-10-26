from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='motor_to_joint_converter',
            executable='motor_to_joint_converter_node',
            name='motor_to_joint_converter_node',
            output='screen',
        ),
    ])
