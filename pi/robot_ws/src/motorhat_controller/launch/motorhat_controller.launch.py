from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='motorhat_controller',
            executable='motor_node',
            name='motorhat_controller',
            output='screen',
            # parameters=[{ 'something': value }]
        )
    ])

