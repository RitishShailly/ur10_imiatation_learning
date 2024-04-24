from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='ur10_learning',
            executable='ur10_teleop_joystick',
            output='screen'),
    ])