#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from gazebo_msgs.srv import DeleteEntity

class ModelDeleter(Node):
    def __init__(self):
        super().__init__('model_deleter')
        self.client = self.create_client(DeleteEntity, '/delete_entity')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('/delete_entity service not available, waiting again...')
    
    def delete_model(self, name):
        request = DeleteEntity.Request()
        request.name = name
        
        future = self.client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        if future.result() is not None:
            if future.result().success:
                self.get_logger().info(f'Model {name} removed successfully.')
            else:
                self.get_logger().error(f'Failed to remove model {name}: {future.result().status_message}')
        else:
            self.get_logger().error('Service call failed %r' % future.exception())

def main(args=None):
    rclpy.init(args=args)
    model_deleter = ModelDeleter()
    
    # Specify the model name to delete
    table1_name = "table1"
    table2_name = "table2"
    coke_name = "coke"
    ball_name = "ball"
    bowl_name = "bowl"
    
    # Delete the model
    model_deleter.delete_model(table1_name)
    # model_deleter.delete_model(table2_name)
    model_deleter.delete_model(coke_name)
    model_deleter.delete_model(ball_name)
    model_deleter.delete_model(bowl_name)
    
    model_deleter.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
