import pickle
import numpy as np
import argparse
import os

folderpath = '~/ros_projects/haptic_ws/src/ur10_learning/data/demonstrations/segments'
folder = os.path.expanduser(folderpath)
segment_name = "segment3"

# hyperparameters
noise = 0.01        
n_upsamples = 10
n_lookahead = 1

files = ['demo1.pkl','demo2.pkl','demo3.pkl','demo4.pkl','demo5.pkl']

sa_pairs = []
for filename in files:
    traj = pickle.load(open(folder + "/"+segment_name+"/joints/" + filename, 'rb'))
    print("[*] Loading file: ", folder + "/" + filename)
    print("[*] Number of data points: ", len(traj))
    traj = np.asarray(traj)
    for idx in range(len(traj) - n_lookahead):  
        s = traj[idx]
        s_next = traj[idx + n_lookahead]       
        for _ in range(n_upsamples):
            s = np.copy(s) + np.random.normal(0, noise, 6)
            a = s_next - s
            sa_pairs.append(s.tolist() + a.tolist())

pickle.dump(sa_pairs, open(folder + "/models/"+segment_name+"/pdata/sa_pairs.pkl", "wb"))
print("I have this many state-action pairs: ", len(sa_pairs))
