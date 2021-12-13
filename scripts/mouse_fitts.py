import pickle
import numpy as np
import pygame
from scripts.task import FittsTask, from_pickle


def main():
    clock = pygame.time.Clock()
    framerate = 60
    delay_secs = 0.5
    n_trials = 1
    n = 0

    # Initialize the task
    task = FittsTask(workspace_lims=((0, 2560), (0, 1440)),
                     target_size_lims=(10, 20),
                     home_pos=(10, 1440//2),
                     home_size=10,
                     steps_to_wait=int(delay_secs * framerate),
                     render=True,
                     render_kwargs=dict(display_size=(2560, 1440), fullscreen=True))

    # Run all trials
    while n < n_trials:
        p_pointer = np.array(pygame.mouse.get_pos())
        success, target_pos, target_size = task.step(p_pointer)
        if success:
            n += 1
        clock.tick(framerate)

    task.exit()


if __name__ == "__main__":
    main()
