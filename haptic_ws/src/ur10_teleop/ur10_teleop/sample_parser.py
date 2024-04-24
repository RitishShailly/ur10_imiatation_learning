# import pinocchio as pin
# from pinocchio.utils import *
# import numpy as np

# model = pin.buildModelFromUrdf('/home/rshailly/Documents/ur10_description.urdf')
# data = model.createData()

# # Specify joint configuration
# q = pin.neutral(model)

# # Compute Jacobian at the end-effector
# frame_id = model.getFrameId('tool0')  # Adjust as needed
# J = pin.computeFrameJacobian(model, data, q, frame_id)

# # Compute pseudo-inverse of Jacobian
# pseudo_inverse_J = np.linalg.pinv(J)

# print(J)
# print("\n\n\n\-----------------------------\n\n\n\n\n")
# print(pseudo_inverse_J)



import pinocchio as pin
from pinocchio.robot_wrapper import RobotWrapper
import numpy as np
import os

class RobotArm:
    def __init__(self, urdf_path):
        # Ensure the URDF file is found at the specified path
        if not os.path.exists(urdf_path):
            raise ValueError("URDF file not found at specified path.")

        # Load the robot model using RobotWrapper
        self.robot = RobotWrapper.BuildFromURDF(urdf_path)
        self.model = self.robot.model
        self.data = self.robot.data

    def compute_jacobian(self, joint_states, frame_name="tool0"):
        """
        Computes the Jacobian for the given joint states at the specified frame.
        :param joint_states: A list or numpy array of joint positions.
        :param frame_name: Name of the frame to compute the Jacobian for.
        :return: The Jacobian matrix for the specified frame.
        """
        # Make sure the joint state vector is the correct size
        if len(joint_states) != self.robot.model.nq:
            raise ValueError("Joint state vector does not match the model's number of joints.")
        
        # Update the robot's internal state
        self.robot.forwardKinematics(joint_states)

        # Compute the Jacobian
        frame_id = self.robot.model.getFrameId(frame_name)
        jacobian = self.robot.frameJacobian(joint_states, frame_id)
        return jacobian

    def pseudo_inverse(self, matrix, tolerance=1e-4):
        """
        Computes the pseudo-inverse of a matrix.
        :param matrix: The matrix to pseudo-invert.
        :param tolerance: Tolerance value for singular values.
        :return: The pseudo-inverse of the matrix.
        """
        u, s, vh = np.linalg.svd(matrix, full_matrices=False)
        s_inv = np.array([0 if abs(x) < tolerance else 1/x for x in s])
        return np.dot(vh.T, np.dot(np.diag(s_inv), u.T))


# Example usage
if __name__ == "__main__":
    # print(pin.Model())
    urdf_path = '/home/rshailly/Documents/ur10_description.urdf'
    robot_arm = RobotArm(urdf_path)
    
    # Example joint state; replace with actual joint states as needed
    
    joint_states = np.array([3.080001785198407, -2.3760839802651876,  -1.135113218857268, -1.214480224517489, 1.390300467660687, 1.3903042907211018] )
    q_dot = 0.01*joint_states
    x_dot = np.array([-0.11674441,  0.39035987,  0.27991097, -0.20627489,  0.37872736, -0.2487736])
    jacobian = robot_arm.compute_jacobian(joint_states)
    print("Jacobian:\n", jacobian)
    
    pseudo_inverse_jacobian = robot_arm.pseudo_inverse(jacobian)
    print("Pseudo-inverse of the Jacobian:\n", pseudo_inverse_jacobian)
    print("\n\n")
    print("End effector velocity:", np.matmul(jacobian,0.1*joint_states))
    print("\n\n")
    print("Joint Velocities:", np.matmul(pseudo_inverse_jacobian,x_dot))