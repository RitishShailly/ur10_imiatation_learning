from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    # Path to your ur10_interface.launch.py file
    ur10_launch_file = os.path.join(get_package_share_directory('ur10_ros2_moveit2'), 'launch', 'ur10_interface.launch.py')

    # Include the existing UR10 launch file
    ur10_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(ur10_launch_file),
    )

    spawn_objects = ExecuteProcess(
        cmd = ['ros2', 'run', 'ur10_colab_gazebo', 'add_simple_env_objects'], output = 'screen')

    return LaunchDescription([       
        ur10_launch,  
        spawn_objects      
    ])
