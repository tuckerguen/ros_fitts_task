import dill
import sys
import warnings
import os
sys.path.append("../src")
base = "C:/School/thesis/ros_fitts_task/trials"
ts = "2021-12-29_16-49-08-519180"
traj_path = [os.path.join(base, f, f2) for f in os.listdir(base) for f2 in os.listdir(os.path.join(base, f)) if f2 == ts]
if traj_path:
    with open(os.path.join(traj_path[0], "trial.pkl"), 'rb') as f:
        trial = dill.load(f)
        trial.replay(speed=3)
else:
    warnings.warn(f"No trial for {ts}")

