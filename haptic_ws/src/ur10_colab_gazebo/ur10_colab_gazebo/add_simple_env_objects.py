#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from gazebo_msgs.srv import SpawnEntity
from geometry_msgs.msg import Pose
from ament_index_python.packages import get_package_share_directory
import os
import time

class ModelSpawner(Node):
    def __init__(self):
        super().__init__('model_spawner')
        self.client = self.create_client(SpawnEntity, '/spawn_entity')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('/spawn_entity service not available, waiting again...')
    
    def spawn_model(self, name, model_path, pose):
        request = SpawnEntity.Request()
        request.name = name
        request.xml = open(model_path, 'r').read()
        request.robot_namespace = ""
        request.initial_pose = pose
        request.reference_frame = "world"
        
        future = self.client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        if future.result() is not None:
            self.get_logger().info(f'Model {name} spawned successfully.')
        else:
            self.get_logger().error('Failed to spawn model %r' % future.exception())

def main(args=None):
    rclpy.init(args=args)
    model_spawner = ModelSpawner()

    #Define model names:
    table1_name = "table1"
    table2_name = "table2"
    coke_name = "coke"
    ball_name = "ball"
    bowl_name = "bowl"

    # Define model paths
    gazebo_model_share_dir = get_package_share_directory('ur10_colab_gazebo')
    table_path = os.path.join(gazebo_model_share_dir, 'models', 'gazebo_models', 'table', 'model.sdf')
    coke_path = os.path.join(gazebo_model_share_dir, 'models', 'gazebo_models', 'coke_can', 'model.sdf')
    ball_path = os.path.join(gazebo_model_share_dir, 'models', 'gazebo_models', 'cricket_ball', 'model.sdf')
    bowl_path = os.path.join(gazebo_model_share_dir, 'models', 'gazebo_models', 'bowl', 'model.sdf')
    
    # Table1 pose
    table1_pose = Pose()
    table1_pose.position.x = 0.0
    table1_pose.position.y = 0.75
    table1_pose.position.z = 0.05 
    table1_pose.orientation.x = 0.0
    table1_pose.orientation.y = 0.0
    table1_pose.orientation.z = 0.0
    table1_pose.orientation.w = 0.0

    # Table2 pose
    table2_pose = Pose()
    table2_pose.position.x = 0.4
    table2_pose.position.y = 1.05
    table2_pose.position.z = 0.05  
    table2_pose.orientation.x = 0.0
    table2_pose.orientation.y = 0.0
    table2_pose.orientation.z = 0.7071067811865475
    table2_pose.orientation.w = 0.7071067811865475

    # Coke pose
    coke_pose = Pose()
    coke_pose.position.x = -0.3
    coke_pose.position.y = 1.00
    coke_pose.position.z = 1.07 
    coke_pose.orientation.x = 0.0
    coke_pose.orientation.y = 0.0
    coke_pose.orientation.z = 0.0
    coke_pose.orientation.w = 0.0

    # Ball pose
    ball_pose = Pose()
    ball_pose.position.x = 0.3
    ball_pose.position.y = 1.00
    ball_pose.position.z = 1.07 
    ball_pose.orientation.x = 0.0
    ball_pose.orientation.y = 0.0
    ball_pose.orientation.z = 0.0
    ball_pose.orientation.w = 0.0

    # Bowl pose
    bowl_pose = Pose()
    bowl_pose.position.x = -0.3
    bowl_pose.position.y = 0.5
    bowl_pose.position.z = 1.07
    bowl_pose.orientation.x = 0.0
    bowl_pose.orientation.y = 0.0
    bowl_pose.orientation.z = 0.0
    bowl_pose.orientation.w = 0.0
    
    # Spawn the model
    model_spawner.spawn_model(table1_name, table_path, table1_pose)
    time.sleep(1)
    # model_spawner.spawn_model(table2_name, table_path, table2_pose)
    # time.sleep(1)
    model_spawner.spawn_model(coke_name, coke_path, coke_pose)
    time.sleep(1)
    model_spawner.spawn_model(ball_name, ball_path, ball_pose)
    time.sleep(1)
    model_spawner.spawn_model(bowl_name, bowl_path, bowl_pose)
    
    model_spawner.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
