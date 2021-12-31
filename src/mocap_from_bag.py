import os
import time
import rospkg
from shutil import copyfile
import sys
sys.path.append("/home/tucker/thesis/ros_workspace/src/fitts_task/src")
from datetime import datetime
from task import FittsTask, TimeDelayWrapper
from mocap import MocapTracker
from util import pygame_get_screenres, Trial, Timepoint, Trajectory

if __name__ == "__main__":
    # CONFIG
    username = "cathy"
    framerate = 60
    delay_secs = 0.5
    n_trials = 100
    calib_secs = 5
    WS_WIDTH = 0.5300869565118
    WS_HEIGHT = 0.298173902
    TGSIZE = 0.01

    td = 0.1
    delay_steps = int(60 * td)
    username = f"{username}_{td:0.2f}_{TGSIZE:.3f}"

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
    task = TimeDelayWrapper(task, delay_steps)

    # TRIAL
    trial = Trial(framerate=framerate, delay_secs=delay_secs, n_trials=n_trials)

    trial.run(task, mocap.get_mocap_pt)



