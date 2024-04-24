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

class UR10Teleop(Node):
    def __init__(self, urdf_path):
        super().__init__('ur10_teleop_keyboard')
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
        
        # Keyboard listener
        listener = Listener(on_press=self.on_press)
        listener.start()
    
    def joint_state_callback(self, msg):
        self.joint_positions = np.array(msg.actual.positions)
    
    def compute_jacobian_and_inverse(self, joint_states):
        self.robot.forwardKinematics(joint_states)
        frame_id = self.robot.model.getFrameId("tool0")
        jacobian = self.robot.frameJacobian(joint_states, frame_id)
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
        print(f"\n\n\nFrame name: {frame_name}, \nPose: \n{end_eff_pose}\n")  
        # for frame in model.frames:
        #     print(f"Frame name: {frame.name}, ID: {model.getFrameId(frame.name)}")
        # for i, frame in enumerate(model.frames):
        #     pose = data.oMi[frame.parent]
        #     print(f"Frame: {frame.name}, Pose: \n{pose}\n")        


    def on_press(self, key):
        # Define velocity increments for x and y directions
        velocity_increment = 0.9  # Adjust as needed for your application
        
        # Initialize the end effector velocity vector (6D: linear and angular velocities)
        # We're only interested in linear velocities here, so angular velocities are set to zero.
        end_effector_velocity = np.zeros(6)
        
        # Update the velocity based on the key pressed
        if key == Key.right:
            end_effector_velocity[0] += velocity_increment  # Increase x velocity
        elif key == Key.left:
            end_effector_velocity[0] -= velocity_increment  # Decrease x velocity
        elif key == Key.up:
            end_effector_velocity[1] += velocity_increment  # Increase y velocity
        elif key == Key.down:
            end_effector_velocity[1] -= velocity_increment  # Decrease y velocity

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


    def publish_joint_trajectory(self, joint_positions):
        msg = JointTrajectory()
        msg.joint_names = self.joint_names
        point = JointTrajectoryPoint()
        point.positions = joint_positions.tolist()
        point.time_from_start.sec = 1  # Adjust as needed
        msg.points.append(point)
        self.publisher.publish(msg)
        self.get_logger().info('Publishing joint trajectory')

    def goToJointState(self, joint_positions, time):
        msg = JointTrajectory()
        msg.joint_names = self.joint_names
        point = JointTrajectoryPoint()
        point.positions = joint_positions.tolist()
        point.time_from_start.sec = time  # Adjust as needed
        msg.points.append(point)
        self.publisher.publish(msg)
        self.get_logger().info('Publishing joint trajectory')

def main(args=None):
    rclpy.init(args=args)
    package_path = get_package_share_directory('ur10_teleop')
    urdf_path = os.path.join(package_path, 'ur10_description.urdf')
    teleop_node = UR10Teleop(urdf_path)
    home = np.array([-1.2376821676837366, -1.098849121724264, -2.025407139454977, -1.5564325491534632, 1.5806676149368286, np.pi])
    # home = np.array([-0.08800792381718203, -0.932866785035019, 1.7335201915270666, -2.3408058241734517, -1.531266118330854, 1.601639085746454])
    teleop_node.goToJointState(home, 5)
    try:
        rclpy.spin(teleop_node)
    finally:
        teleop_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
