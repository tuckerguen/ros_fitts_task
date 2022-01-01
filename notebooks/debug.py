import dill
import sys
import warnings
import os
sys.path.append("../src")
base = "/home/tucker/thesis/ros_workspace/src/fitts_task/trials"
ts = "2021-12-30_14-30-18-403086"
traj_path = [os.path.join(base, f, f2) for f in os.listdir(base) for f2 in os.listdir(os.path.join(base, f)) if f2 == ts]
if traj_path:
    with open(os.path.join(traj_path[0], "trial.pkl"), 'rb') as f:
        trial = dill.load(f)
        # trial.replay(speed=3)
        trial.plot()
else:
    warnings.warn(f"No trial for {ts}")

