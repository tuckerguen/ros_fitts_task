import os
from shutil import copyfile
import pygame
import pickle
import numpy as np
from task import FittsTask, TimeDelayWrapper
from util import pygame_get_screenres, Trial, Trajectory, Timepoint


def main():
    delay_secs = 0.5
    framerate = 60

    w, h = pygame_get_screenres()

    # Initialize the task
    task = FittsTask(workspace_lims=((0, w), (0, h)),
                     target_size_lims=(10, 30),
                     home_pos=(w//2, h//2),
                     home_size=15,
                     steps_to_wait=int(delay_secs * framerate),
                     render=True,
                     render_kwargs=dict(display_size=(w, h), fullscreen=True))

    time_delay = 0.0
    n_delay_steps = int(round(time_delay * framerate))
    task = TimeDelayWrapper(task, n_delay_steps)

    # Track the trial
    trial = Trial(framerate=framerate, delay_secs=delay_secs, n_trials=1)
    trial.run(task, lambda: np.array(pygame.mouse.get_pos()))
    trial.save("../trials/test1.pkl")

    # Relevant files
    pyfiles_path = os.path.join("../trials", "pyfiles")
    os.makedirs(pyfiles_path, exist_ok=True)
    for f in os.listdir("../src"):
        dest = os.path.join(pyfiles_path, os.path.basename(f))
        copyfile(__file__, dest)


if __name__ == "__main__":
    main()
