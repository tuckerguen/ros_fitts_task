import os
import time
import rospkg
from shutil import copyfile
from datetime import datetime
from task import FittsTask, TimeDelayWrapper
from mocap import MocapTracker
from util import pygame_get_screenres, Trial, Timepoint, Trajectory

if __name__ == "__main__":
    # CONFIG
    username = "tucker"
    framerate = 60
    delay_secs = 0.5
    n_trials = 2
    calib_secs = 5
    WS_WIDTH = 0.5300869565118
    WS_HEIGHT = 0.298173902

    # Trial tracking info
    rospack = rospkg.RosPack()
    timestamp = datetime.now().strftime("%Y-%m-%d_%H-%M-%S-%f")
    target_dir = os.path.join(rospack.get_path('fitts_task'), "trials", username, timestamp)
    if not os.path.exists(target_dir):
        os.makedirs(target_dir, exist_ok=True)

    t0 = time.process_time()

    # Create the mocap task
    mocap = MocapTracker(os.path.join(target_dir, "rosbag.bag"))
    # Run calibration
    calib_start_time = time.process_time() - t0
    mocap.calibrate(calib_secs)
    calib_end_time = time.process_time() - t0

    # Create the fitts task
    # NOTE: When using a viewer, create the FittsTask after performing
    #  calibration to avoid interference of the existing pygame screen
    task = FittsTask(workspace_lims=((0, WS_WIDTH), (0, WS_HEIGHT)),
                     target_size_lims=(0.01, 0.02),
                     home_pos=(WS_WIDTH / 2, WS_HEIGHT / 2),
                     home_size=0.01,
                     steps_to_wait=int(delay_secs * framerate),
                     stationary_tolerance=0.005,
                     render=True,
                     render_kwargs=dict(display_size=pygame_get_screenres(), fullscreen=True))
    task = TimeDelayWrapper(task, 0)

    # TRIAL
    trial = Trial(framerate=framerate, delay_secs=delay_secs, n_trials=1)
    task_start_time = time.process_time() - t0
    trial.run(task, mocap.get_mocap_pt)
    task_end_time = time.process_time() - t0

    # SAVE
    # rosbag
    # Mocap tracker
    mocap.save(os.path.join(target_dir, "mocap.pkl"))
    # Organized Trial and Trajectories data
    trial.save(os.path.join(target_dir, "trial.pkl"))
    # Calibration and task timestamps
    with open(os.path.join(target_dir, "timestamps.txt"), "w") as f:
        f.write("Script start")
        f.write(str(t0))
        f.write("Calibrate Start,End")
        f.write(f"{calib_start_time},{calib_end_time}")
        f.write("Task Start,End")
        f.write(f"{calib_start_time},{task_end_time}")

    # Relevant files
    pyfiles_path = os.path.join(target_dir, "pyfiles")
    os.makedirs(pyfiles_path, exist_ok=True)
    for f in os.listdir("../src"):
        dest = os.path.join(pyfiles_path, os.path.basename(__file__))
        copyfile(__file__, dest)


