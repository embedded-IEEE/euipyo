from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='rc_ackermann_joy',
            executable='joy_to_ackermann',
            name='joy_to_ackermann',
            parameters=[{
                'joy_topic': '/joy',
                'cmd_topic': '/rc_car/cmd_vel',
                'speed_axis': 1,
                'steer_axis': 2,
                'max_speed': 1.0,
                'max_steer': 1.0,
                'speed_deadzone': 0.05,
                'steer_deadzone': 0.05,
                'invert_speed': False,
                'invert_steer': False,
            }],
            output='screen',
        ),
    ])
