# This is just to test and see if the rosbags work properly. They do!

import os
import numpy as np
import dill
import rosbag
import sys
sys.path.append("/home/tucker/thesis/ros_workspace/src/fitts_task/src")
from task import FittsTask
from util import Trial

if __name__ == "__main__":
    # CONFIG
    framerate = 60
    delay_secs = 0.5
    n_trials = 100
    calib_secs = 5
    WS_WIDTH = 0.5300869565118
    WS_HEIGHT = 0.298173902
    TGSIZE = 0.01
    base = "C:/School/thesis/ros_fitts_task/trials"
    ts = "2021-12-29_16-07-16-689049"
    traj_path = [os.path.join(base, f, f2) for f in os.listdir(base) for f2 in os.listdir(os.path.join(base, f)) if
                 f2 == ts][0]

    # Create the fitts task
    # NOTE: When using a viewer, create the FittsTask after performing
    #  calibration to avoid interference of the existing pygame screen
    task = FittsTask(workspace_lims=((0, WS_WIDTH), (0, WS_HEIGHT)),
                     target_size_lims=(TGSIZE, TGSIZE),
                     home_pos=(WS_WIDTH / 2, WS_HEIGHT / 2),
                     home_size=0.01,
                     steps_to_wait=int(delay_secs * framerate),
                     stationary_tolerance=0.005,
                     render=True,
                     render_kwargs=dict(display_size=(1920, 1080)))

    # TRIAL
    trial = Trial(framerate=framerate, delay_secs=delay_secs, n_trials=n_trials)

    bag = rosbag.Bag(os.path.join(traj_path, "rosbag.bag"))
    msg_gen = bag.read_messages("polaris_sensor/targets")

    T = np.load(os.path.join(traj_path, "T.npy"))

    def msg_func():
        p = next(msg_gen).message.poses[0].position
        pt = np.array([p.x, p.y, p.z, 1])
        if not np.any(np.isnan(pt)):
            pt = np.matmul(T, pt.T)[1:3]
            pt = pt[::-1]
            print(pt[0]*1920, pt[1]*1080)
            return pt
        else:
            return None


    trial.run(task, msg_func)



