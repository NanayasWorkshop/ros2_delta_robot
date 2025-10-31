from launch import LaunchDescription
from launch_ros.actions import Node
import os

def generate_launch_description():
    urdf_file = os.path.join(os.getcwd(), 'modular_robot_follower.urdf')
    rviz_config = os.path.join(os.getcwd(), 'robot_view.rviz')
    
    with open(urdf_file, 'r') as f:
        robot_desc = f.read()
    
    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_desc}]
        ),
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            output='screen'
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            output='screen',
            arguments=['-d', rviz_config]
        )
    ])
