from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='ur10_teleop',
            executable='ur10_teleop_keyboard',
            output='screen'),
    ])