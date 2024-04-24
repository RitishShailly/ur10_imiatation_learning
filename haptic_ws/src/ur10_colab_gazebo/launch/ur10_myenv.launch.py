from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, TimerAction
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

    gazebo_model_share_dir = get_package_share_directory('ur10_colab_gazebo')
    table_path = os.path.join(gazebo_model_share_dir, 'models', 'gazebo_models', 'table', 'model.sdf')
    cabinet_path = os.path.join(gazebo_model_share_dir, 'models', 'gazebo_models', 'cabinet', 'model.sdf')
    ball_path = os.path.join(gazebo_model_share_dir, 'models', 'gazebo_models', 'cricket_ball', 'model.sdf')
    bowl_path = os.path.join(gazebo_model_share_dir, 'models', 'gazebo_models', 'bowl', 'model.sdf')

    # Timer for delaying spawns
    spawn_delay = 1.5  # Seconds

    # Example: Spawn a table in Gazebo at position x=10, y=0, z=0
    spawn_table1 = TimerAction(
        period=spawn_delay,
        actions=[ExecuteProcess(
            cmd=['ros2', 'run', 'gazebo_ros', 'spawn_entity.py',
                 '-entity', 'table1', 
                 '-file', table_path,
                 '-x', '-0.4', '-y', '1.05', '-z', '0.0', '-Y', '1.57'],
            output='screen'
        )]
    )

    spawn_table2 = TimerAction(
        period=spawn_delay,
        actions=[ExecuteProcess(
            cmd=['ros2', 'run', 'gazebo_ros', 'spawn_entity.py',
                 '-entity', 'table2', 
                 '-file', table_path,
                 '-x', '0.4', '-y', '1.05', '-z', '0.0', '-Y', '1.57'],
            output='screen'
        )]
    )

    spawn_cabinet = TimerAction(
        period=spawn_delay,
        actions=[ExecuteProcess(
            cmd=['ros2', 'run', 'gazebo_ros', 'spawn_entity.py',
                 '-entity', 'cabinet', 
                 '-file', cabinet_path,
                 '-x', '0.3', '-y', '1.25', '-z', '1.02', '-Y', '1.57'],
            output='screen'
        )]
    )

    spawn_ball = TimerAction(
        period=spawn_delay,
        actions=[ExecuteProcess(
            cmd=['ros2', 'run', 'gazebo_ros', 'spawn_entity.py',
                 '-entity', 'ball', 
                 '-file', ball_path,
                 '-x', '0.3', '-y', '1.15', '-z', '1.6'],
            output='screen'
        )]
    )

    spawn_bowl = TimerAction(
        period=spawn_delay,
        actions=[ExecuteProcess(
            cmd=['ros2', 'run', 'gazebo_ros', 'spawn_entity.py',
                 '-entity', 'bowl', 
                 '-file', bowl_path,
                 '-x', '-0.3', '-y', '0.8', '-z', '1.4'],
            output='screen'
        )]
    )

    return LaunchDescription([
        spawn_ball,
        spawn_bowl,
        ur10_launch, 
        spawn_table1,
        spawn_table2,
        spawn_cabinet,       
    ])
