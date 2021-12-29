import numpy as np
import pygame
from task import FittsTask, TimeDelayWrapper
from util import pygame_get_screenres


def main():
    clock = pygame.time.Clock()
    framerate = 60
    delay_secs = 0.5
    n_trials = 2
    n = 0
    w, h = pygame_get_screenres()

    # Initialize the task
    task = FittsTask(workspace_lims=((0, w), (0, h)),
                     target_size_lims=(10, 30),
                     home_pos=(w//2, h//2),
                     home_size=15,
                     steps_to_wait=int(delay_secs * framerate),
                     render=True,
                     render_kwargs=dict(display_size=(w, h), fullscreen=True))

    time_delay = 0.1
    n_delay_steps = int(round(time_delay * framerate))
    task = TimeDelayWrapper(task, n_delay_steps)

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
