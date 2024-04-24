import rclpy
from rclpy.node import Node
from control_msgs.msg import JointTrajectoryControllerState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from pynput.keyboard import Key, Listener
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
            # self.action = None
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

class UR10Teleop(Node):
    def __init__(self, urdf_path, filename = "data/recording.pkl"):
        super().__init__('ur10_teleop_joystick')
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
        self.home = np.array([-1.2376821676837366, -1.098849121724264, -2.025407139454977, -1.5564325491534632, 1.5806676149368286, np.pi])

        # Initialize variables for recording data
        self.record = False
        self.data = []
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
        # print(f"\n\n\nFrame name: {frame_name}, \nPose: \n{end_eff_pose}\n")  
        # for frame in model.frames:
        #     print(f"Frame name: {frame.name}, ID: {model.getFrameId(frame.name)}")
        # for i, frame in enumerate(model.frames):
        #     pose = data.oMi[frame.parent]
        #     print(f"Frame: {frame.name}, Pose: \n{pose}\n")        


    def handle_joystick_input(self):
        # Define velocity increments for x and y directions
        lin_velocity_increment = 0.9 
        ang_velocity_increment = 2.0 
        
        # Initialize the end effector velocity vector (6D: 3 linear and 3 angular velocities)
        
        end_effector_velocity = np.zeros(6)
        BACK_pressed = 0
        try: 
            du, dtheta, A_pressed, B_pressed, BACK_pressed, START_pressed = self.joystick.getInput()
            end_effector_velocity[0:3] = lin_velocity_increment * np.asarray(du)
            end_effector_velocity[3:] = ang_velocity_increment * np.asarray(dtheta)
            # print(end_effector_velocity)
            if (BACK_pressed):
                self.goToJointState(self.home, 5)
                time.sleep(10)
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

        # Go to HOME position if BACK pressed
        if (BACK_pressed):
            self.get_logger().info('BACK button pressed. Going to HOME position...')
            time.sleep(1)
            self.joint_positions = self.home
            self.goToJointState(self.home, 5)
            # time.sleep(6)
            self.get_logger().info('Reached HOME!')

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

        if self.record and self.curr_time - self.last_time > time_step:
            self.data.append(list(self.joint_positions))
            last_time = self.curr_time

        if self.record and B_pressed:
            self.record = False
            pickle.dump(self.data, open(self.filename, "wb"))
            print("B button pressed!\n[*] Saved Recording")
        


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

def main(args=None):
    rclpy.init(args=args)
    # joystick = JoystickControl()
    package_path = get_package_share_directory('ur10_teleop')
    urdf_path = os.path.join(package_path, 'ur10_description.urdf')
    filename = '~/ros_projects/haptic_ws/src/ur10_teleop/data/recording.pkl'
    filename = os.path.expanduser(filename)
    print("Filename is: ", filename)
    teleop_node = UR10Teleop(urdf_path, filename)
    home = np.array([-1.2376821676837366, -1.098849121724264, -2.025407139454977, -1.5564325491534632, 1.5806676149368286, np.pi])
    teleop_node.goToJointState(home, 5)
    time.sleep(6)
    teleop_node.initialize_joystick()
    try:
        rclpy.spin(teleop_node)
    except KeyboardInterrupt:
        pass
    finally:
        teleop_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
