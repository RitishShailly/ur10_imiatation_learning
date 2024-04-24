import rclpy
from rclpy.node import Node
from control_msgs.msg import JointTrajectoryControllerState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import pinocchio as pin
from pinocchio.robot_wrapper import RobotWrapper
import numpy as np
import os
from ament_index_python.packages import get_package_share_directory
import pygame
import time
import pickle
import sys

class JoystickControl(object):
    def __init__(self):
        pygame.init()
        pygame.joystick.init()
        joystick_count = pygame.joystick.get_count()
        if joystick_count == 0:
            raise IOError("No joystick detected")
        else:
            self.gamepad = pygame.joystick.Joystick(0)
            self.gamepad.init()
            self.toggle = False
            self.deadband = 0.1
            self.timeband = 0.5
            self.lastpress = time.time()
            print(f"Initialized Joystick : {self.gamepad.get_name()}")

    def getInput(self):
        pygame.event.get()
        curr_time = time.time()
        dx = self.gamepad.get_axis(0)
        dy = -self.gamepad.get_axis(1)
        dz = -self.gamepad.get_axis(4)
        d_roll = -(self.gamepad.get_axis(2)+1)/2 + (self.gamepad.get_axis(5)+1)/2

        if abs(dx) < self.deadband:
            dx = 0.0
        if abs(dy) < self.deadband:
            dy = 0.0
        if abs(dz) < self.deadband:
            dz = 0.0
        if abs(d_roll) < self.deadband:
            d_roll = 0.0

        LB = self.gamepad.get_button(4)
        RB = self.gamepad.get_button(5)
        d_pitch = 0.0
        if LB:
            d_pitch = -1.0
        elif RB:
            d_pitch = 1.0
        d_yaw = self.gamepad.get_hat(0)[0]        

        A = self.gamepad.get_button(0)
        B = self.gamepad.get_button(1)
        # X = self.gamepad.get_button(2)
        # Y = self.gamepad.get_button(3)        
        BACK = self.gamepad.get_button(6)
        START = self.gamepad.get_button(7)
        # LOGITECH = self.gamepad.get_button(8)
        return [dx, dy, dz], [d_roll, d_pitch, d_yaw], A, B, BACK, START

class UR10(Node):
    def __init__(self, urdf_path, home_state, folderpath, filename):
        super().__init__('record_demo')
        self.urdf_path = urdf_path
        # self.joystick = joystick
        self.publisher = self.create_publisher(JointTrajectory, '/ur10_controller/joint_trajectory', 10)
        self.subscription = self.create_subscription(
            JointTrajectoryControllerState,
            '/ur10_controller/state',
            self.joint_state_callback,
            10)
        
        # Load the robot model
        if not os.path.exists(urdf_path):
            raise ValueError("URDF file not found.")
        self.robot = RobotWrapper.BuildFromURDF(urdf_path)
        
        # Initialize joint positions and names
        self.joint_positions = np.zeros(self.robot.model.nq)
        self.joint_names = [name for name in self.robot.model.names if name != "universe"]
        self.tool_rotation = np.eye(3)    
        self.home = home_state

        # Initialize variables for recording data
        self.record = False
        self.data = []
        self.end_effector_data = []
        self.folderpath = folderpath
        self.filename = filename
        self.last_time = time.time()
    
    def initialize_joystick(self):
        # Joystick Listener:
        self.joystick = JoystickControl()    
        self.create_timer(0.5, self.handle_joystick_input) 
    
    def joint_state_callback(self, msg):
        self.joint_positions = np.array(msg.actual.positions)
    
    def compute_jacobian_and_inverse(self, joint_states):
        self.robot.forwardKinematics(joint_states)
        frame_id = self.robot.model.getFrameId("tool0")
        jacobian = self.robot.computeFrameJacobian(joint_states, frame_id)
        pseudo_inverse_jacobian = np.linalg.pinv(jacobian)
        return jacobian, pseudo_inverse_jacobian
    
    def compute_tool_rot(self):
        model = pin.buildModelFromUrdf(self.urdf_path)
        data = model.createData()
        q = self.joint_positions
        pin.forwardKinematics(model,data,q)
        frame_id = model.frames[-1].parent
        frame_name = model.frames[-1].name
        end_eff_pose = data.oMi[frame_id]
        self.tool_rotation =  end_eff_pose.rotation
        self.tool_translation = end_eff_pose.translation
        # self.print_states()

    def print_states(self):
        print("End effector position: ", self.tool_translation)
        print("Joint states: ", self.joint_positions)

    def handle_joystick_input(self):
        # Define velocity increments for x and y directions
        lin_velocity_increment = 0.5 
        ang_velocity_increment = 2.0 
        
        # Initialize the end effector velocity vector (6D: 3 linear and 3 angular velocities)
        end_effector_velocity = np.zeros(6)
        BACK_pressed = 0
        try: 
            du, dtheta, A_pressed, B_pressed, BACK_pressed, START_pressed = self.joystick.getInput()
            end_effector_velocity[0:3] = lin_velocity_increment * np.asarray(du)
            end_effector_velocity[3:] = ang_velocity_increment * np.asarray(dtheta)
            # print(end_effector_velocity)
        except Exception as e:
            self.get_logger().error(f"Failed to process joystick input: {e}")

        # Compute tool rotation
        self.compute_tool_rot()
        end_effector_velocity[0:3] = np.dot(np.transpose(self.tool_rotation), end_effector_velocity[0:3])

        
        # Compute the Jacobian and its pseudo-inverse
        jacobian, pseudo_inverse_jacobian = self.compute_jacobian_and_inverse(self.joint_positions)
        
        # Compute the joint velocities by multiplying the pseudo-inverse of the Jacobian with the end effector velocity
        joint_velocities = np.matmul(pseudo_inverse_jacobian, end_effector_velocity)
        # print("End effector velocities: \n", np.matmul(jacobian,joint_velocities))

        time_step = 0.1  # Time step for updating positions
        self.joint_positions += joint_velocities * time_step        
        
        # Publish the updated joint positions as a trajectory
        self.publish_joint_trajectory(self.joint_positions)

        # Print States:
        self.print_states()

        # Go to HOME position if BACK pressed
        if (BACK_pressed):
            self.get_logger().info('BACK button pressed. Going to HOME position...')
            time.sleep(1)
            self.goToJointState(self.home, 5)
            time.sleep(6)
            # self.publish_joint_trajectory(self.joint_positions)
            self.get_logger().info('Reached HOME!')
            self.joint_positions = self.home

        # Stop the program when START button is pressed:
        if (START_pressed):
            self.get_logger().info('START button pressed. Initiating KeyboardInterrupt...')
            raise KeyboardInterrupt 

        # For data recording:     
        self.curr_time = time.time()   
        if (not self.record and A_pressed):
            print('A button pressed! \n[*] Recording')
            self.last_time = time.time()
            self.record = True
        
        if self.record:
            # self.print_states()
            print("Recording")

        if self.record and self.curr_time - self.last_time > time_step:
            self.data.append(list(self.joint_positions))
            self.end_effector_data.append(self.tool_translation)
            # print("Recorded current position")
            self.last_time = self.curr_time

        if self.record and B_pressed:
            self.record = False
            if(self.save_data(self.data, self.end_effector_data, self.folderpath, self.filename)):
                print("B button pressed!\n[*] Saved Recording")

    def save_data(self, trajectory_data, end_effector_data, folderpath, filename):
        # store trajectory data in list
        data = trajectory_data
        print("Initial data size: ",len(data))
        # Make action pairs for each trajectory point
        a =[]
        # print(data[1]-data[0])
        for i in range(len(data)-1):
            a.append(list(np.array(data[i+1])- np.array(data[i]))) 
        a.append([0.,0.,0.,0.,0.,0.])
        # Remove null data is action of each state < threshold
        null_threshold_array = np.array([5e-3]*6)
        null_index_list = []
        num_states = 6
        for i in range(len(a)-1):
            shallRemove = [False]*6
            for j in range(num_states):
                if abs(a[i][j]) < abs(null_threshold_array[j]):
                    shallRemove[j] = True
            if shallRemove == [True]*6:
                null_index_list.append(i+1)        
        for i in range(len(null_index_list)):
            pop_index = null_index_list[len(null_index_list)-i-1]
            data.pop(pop_index)
            end_effector_data.pop(pop_index)
        # Save data:
        pickle.dump(data, open(folderpath+ 'joints/' + filename + '.pkl', "wb"))
        pickle.dump(list(end_effector_data), open(folderpath+ 'end_eff/' + filename + '.pkl', "wb"))
        print("Final data size: ",len(data))
        return True

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
        self.get_logger().info('Publishing joint trajectory')
        # asyncio.sleep(time)

def main(args=None):
    rclpy.init(args=args)
    # joystick = JoystickControl()
    package_path = get_package_share_directory('ur10_learning')
    urdf_path = os.path.join(package_path, 'ur10_description.urdf')
    
    # Take user input for saving data path
    filename = input("[*] Enter the pickle file name (hint: 'demo1', 'demo2', etc): ")
    segment_name = input("[*] Enter the type of segment (hint: 'segment1','segment2', 'segment3'): ")
    if filename == "":
        filename = 'demo0'
    if segment_name=="":
        segment_name = 'segment0'

    folderpath = '~/ros_projects/haptic_ws/src/ur10_learning/data/demonstrations/segments/'+ segment_name +'/'
    folderpath = os.path.expanduser(folderpath)
    print("Folderpath is: ", folderpath)
    print("Filename is: ", filename)

    # Initialize ROS2 node
    home = np.array([-1.82079426, -1.98607819, -2.37173143, -0.37213856,  1.59708262,  2.55759712])
    ptA = np.array([-1.7098215,  -2.45771547, -1.37379185, -0.89669361,  1.59892469,  2.66856049])
    ptB = np.array([-1.11940628, -2.4539005,  -1.39806373, -0.86297165,  1.60338097,  3.25906777])

    record_demo_node = UR10(urdf_path, ptB, folderpath, filename)
    
    print("Going to Home position!")
    record_demo_node.goToJointState(ptB, 5)
    time.sleep(6)
    record_demo_node.initialize_joystick()
    try:
        rclpy.spin(record_demo_node)
    except KeyboardInterrupt:
        pass
    finally:
        record_demo_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
