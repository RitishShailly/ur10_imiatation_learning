import pickle 
import os
import time
import numpy as np
import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import sys


class PlayTrajectory(Node):
    def __init__(self, data):
        super().__init__('play_trajectory_node')
        self.timestep = 1
        self.publisher = self.create_publisher(JointTrajectory, '/ur10_controller/joint_trajectory', 10)
        self.trajectory_data = data
        self.joint_names = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
        self.play_trajectory()

    def play_trajectory(self):
        try:
            initial_state = np.array(self.trajectory_data[0])
            print("Wait until the robot reaches initial position")
            self.goToJointState(initial_state, time = 5)
            time.sleep(5)
            print("Playing Trajectory")
            for i in range(1, len(self.trajectory_data)):
                self.publish_joint_trajectory(np.array(self.trajectory_data[i]))
                time.sleep(self.timestep)
            print("Trajectory finished")
        except Exception as e:
            self.get_logger().error('An error occurred: {}'.format(e))
        finally:
            raise KeyboardInterrupt 

    def publish_joint_trajectory(self, joint_positions):
        msg = JointTrajectory()
        msg.joint_names = self.joint_names
        point = JointTrajectoryPoint()
        point.positions = joint_positions.tolist()
        point.time_from_start.sec = 1  # Adjust as needed
        msg.points.append(point)
        self.publisher.publish(msg)
        # self.get_logger().info('Publishing joint trajectory')

    def goToJointState(self, joint_positions, time):
        msg = JointTrajectory()
        msg.joint_names = self.joint_names
        point = JointTrajectoryPoint()
        point.positions = joint_positions.tolist()
        point.time_from_start.sec = time
        msg.points.append(point)
        self.publisher.publish(msg)
        self.get_logger().info('Going to initial position of trajectory')

def main(args = None):
    # Extract data from pickle file
    pickle_file_name = input("[*] Enter the pickle file name (hint: 'demo1', 'demo2', etc): ")
    folder_name = input("[*] Enter the type of environment (hint: 'env1','env2', etc.): ")
    if pickle_file_name == "":
        pickle_file_name = 'demo0'
    if folder_name=="":
        folder_name = 'env0'

    filename = '~/ros_projects/haptic_ws/src/ur10_learning/data/demonstrations/'+ folder_name +'/'+ pickle_file_name +'.pkl'
    filename = os.path.expanduser(filename)
    print("Filename is: ", filename)
    with open(filename, 'rb') as file:
        # Load the list of lists from the file
        trajectory_data = np.array(pickle.load(file))
        print("Total waypoints: ", len(trajectory_data))

    # Run PlayTrajectory ROS2 Node
    rclpy.init(args=args)
    play_trajectory_node = PlayTrajectory(trajectory_data)
    try:
        rclpy.spin(play_trajectory_node)
    except KeyboardInterrupt:
        pass
    finally:
        play_trajectory_node.destroy_node()
        rclpy.shutdown()
        sys.exit()

if __name__ == '__main__':
    main()
    