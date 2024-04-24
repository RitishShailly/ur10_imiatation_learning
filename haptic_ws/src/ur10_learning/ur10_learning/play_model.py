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
import torch
import torch.nn as nn
from torch.utils.data import Dataset, DataLoader
import torch.optim as optim

class BC(nn.Module):
    def __init__(self, hidden_dim):
        super(BC, self).__init__()
        self.state_dim = 6        
        self.action_dim = 6
        self.linear1 = nn.Linear(self.state_dim, hidden_dim)
        self.linear2 = nn.Linear(hidden_dim, hidden_dim)
        self.linear3 = nn.Linear(hidden_dim, self.action_dim)
        self.loss_func = nn.MSELoss()

    def encoder(self, state):
        h1 = torch.tanh(self.linear1(state))
        h2 = torch.tanh(self.linear2(h1))
        return self.linear3(h2)

    def forward(self, x):
        state = x[:, :self.state_dim]
        a_target = x[:, self.action_dim:]
        a_predicted = self.encoder(state)
        loss = self.loss(a_predicted, a_target)
        return loss

    def loss(self, a_predicted, a_target):
        return self.loss_func(a_predicted, a_target)

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
    
class Model(object):
    def __init__(self, model_file):
        self.model = BC(32)
        model_dict = torch.load(model_file, map_location='cpu')
        self.model.load_state_dict(model_dict)
        self.model.eval

    def policy(self, state):
        s_tensor = torch.FloatTensor(state)
        action = self.model.encoder(s_tensor).detach().numpy()
        return action

class UR10(Node):
    def __init__(self, urdf_path, home_state, folderpath, filename):
        super().__init__('play_model_node')
        self.urdf_path = urdf_path
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
        # self.record = False
        self.data = []
        self.end_effector_data = []
        self.folderpath = folderpath
        self.filename = filename
        self.last_time = time.time()

        # Initialize model
        self.model = Model(self.folderpath + self.filename)
        self.run = False
    
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

    def print_states(self):
        print("End effector position: ", self.tool_translation)
        print("Joint states: ", self.joint_positions)

    def play_model(self):
        s = list(self.joint_positions)
        a = self.model.policy(s)
        action_scale = 0.6

        if np.linalg.norm(a) > action_scale:
            a = a / np.linalg.norm(a) * action_scale

        if self.run == True:
            time_step = 0.1  # Time step for updating positions
            s_new = self.joint_positions + a * time_step *10
            self.publish_joint_trajectory(s_new)
            time.sleep(time_step)


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
        time_step = 0.1  # Time step for updating positions

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
        if (not self.run and A_pressed):
            print('A button pressed! \n[*] Running Model')
            self.last_time = time.time()
            self.run = True
        
        if self.run:
            self.play_model()
            self.print_states()

        if self.run and self.curr_time - self.last_time > time_step:
            self.data.append(list(self.joint_positions))
            self.end_effector_data.append(self.tool_translation)
            self.last_time = self.curr_time

        if self.run and B_pressed:
            self.run = False
            print("B button pressed!\n[*] Stopping Model")
            pickle.dump(list(self.end_effector_data), open(self.folderpath + "/trajectories/"+ self.filename +"_traj.pkl", "wb"))



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
    filename = input("[*] Enter the model name (hint: 'model1', 'model2', etc): ")
    folder_name = input("[*] Enter the type of environment (hint: 'env1','env2', etc.): ")
    if filename == "":
        filename = 'model1'
    if folder_name=="":
        folder_name = 'env1'

    folderpath = '~/ros_projects/haptic_ws/src/ur10_learning/data/demonstrations/'+ folder_name +'/DAgger_model/'
    folderpath = os.path.expanduser(folderpath)
    print("Folderpath is: ", folderpath)
    print("Filename is: ", filename)

    # Initialize ROS2 node
    home = np.array([-1.82079426, -1.98607819, -2.37173143, -0.37213856,  1.59708262,  2.55759712])
    play_model_node = UR10(urdf_path, home, folderpath, filename)

    
    print("Going to Home position!")
    play_model_node.goToJointState(home, 5)
    time.sleep(6)
    play_model_node.initialize_joystick()
    try:
        rclpy.spin(play_model_node)
    except KeyboardInterrupt:
        pass
    finally:
        play_model_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
