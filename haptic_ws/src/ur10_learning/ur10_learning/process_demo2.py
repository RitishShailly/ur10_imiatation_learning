import pickle
import numpy as np
import argparse
import os
import pinocchio as pin
from pinocchio.robot_wrapper import RobotWrapper
import os

urdf_path = '~/ros_projects/haptic_ws/src/ur10_learning/urdf/ur10_description.urdf'
urdf_path = os.path.expanduser(urdf_path)
folderpath = '~/ros_projects/haptic_ws/src/ur10_learning/data/demonstrations/env1/joint_states/'
folder = os.path.expanduser(folderpath)

# hyperparameters
noise = 0.01        
n_upsamples = 10
n_lookahead = 1

files = ['demo1.pkl','demo2.pkl','demo3.pkl','demo4.pkl','demo5.pkl']

def getEndEffectorTranslation(joint_positions):
    model = pin.buildModelFromUrdf(urdf_path)
    data = model.createData()
    q = np.array(joint_positions)
    pin.forwardKinematics(model,data,q)
    frame_id = model.frames[-1].parent
    frame_name = model.frames[-1].name
    end_eff_pose = data.oMi[frame_id]
    tool_rotation =  end_eff_pose.rotation
    tool_translation = end_eff_pose.translation
    return list(tool_translation)

p1 = [0.3,1.0,1.36]
p2 = [-0.3,1.0,1.36]
p3 = [-0.3, 0.5,1.36]
sa_pairs = []
for filename in files:
    traj = pickle.load(open(folder + filename, 'rb'))
    print("[*] Loading file: ", folder + "/" + filename)
    print("[*] Number of data points: ", len(traj))
    traj = np.asarray(traj)
    for idx in range(len(traj) - n_lookahead):  
        s = traj[idx]
        s_next = traj[idx + n_lookahead]       
        for _ in range(n_upsamples):
            s = np.copy(s) + np.random.normal(0, noise, 6)
            a = s_next - s
            sa_pairs.append(s.tolist() + getEndEffectorTranslation(s)+ p1 + p2 + p3 + a.tolist())

save_folderpath = '~/ros_projects/haptic_ws/src/ur10_learning/data/demonstrations/env1/processed_DAgger_data/'
save_folder = os.path.expanduser(save_folderpath)
pickle.dump(sa_pairs, open(save_folder +"sa_pairs_with_positions.pkl", "wb"))
print("I have this many state-action pairs: ", len(sa_pairs))