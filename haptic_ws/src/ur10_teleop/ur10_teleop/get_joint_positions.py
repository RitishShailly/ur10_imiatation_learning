import rclpy
from rclpy.node import Node
from control_msgs.msg import JointTrajectoryControllerState  # Adjust the message type if needed
import time


class JointStateSubscriber(Node):
    def __init__(self):
        super().__init__('joint_state_subscriber')
        self.joint_positions = []  # Initialize an empty list to store positions
        self.subscription = self.create_subscription(
            JointTrajectoryControllerState,
            '/ur10_controller/state',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.joint_positions = msg.actual.positions  # Store the latest positions
        print(self.joint_positions)
        # You may need to adjust the path to positions depending on your message structure

    def get_latest_joint_positions(self):
        return self.joint_positions

def main(args=None):
    rclpy.init(args=args)
    joint_state_subscriber = JointStateSubscriber()

    try:
        timeout = 10  # Timeout in seconds
        start_time = time.time()
        while rclpy.ok():
            rclpy.spin_once(joint_state_subscriber, timeout_sec=1)  # Adjust timeout as needed
            positions = joint_state_subscriber.get_latest_joint_positions()
            if positions:  # Check if positions list is not empty
                print("Latest Joint Positions:", positions)
                break  # Exit the loop if positions have been retrieved
            if (time.time() - start_time) > timeout:
                print("Timeout: No message received on /ur10_controller/state.")
                break
    finally:
        # Cleanup
        joint_state_subscriber.destroy_node()
        rclpy.shutdown()

