from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            output='screen'
        ),
        Node(
            package='joy_to_wrench',
            executable='joy_to_wrench_node',
            name='joy_to_wrench_node',
            output='screen'
        ),
    ])
