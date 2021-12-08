from task import FittsTask
import numpy as np
import pygame


def main():
    clock = pygame.time.Clock()
    fr = 60
    delay_secs = 0.5
    dt = 1/fr
    t = 0

    task = FittsTask(workspace_lims=((0, 2560), (0, 1440)),
                     target_size_lims=(10, 20),
                     home_pos=(10, 1440//2),
                     home_size=10,
                     steps_to_wait=int(delay_secs * fr),
                     render=True)

    while t < 100:
        p_pointer = np.array(pygame.mouse.get_pos())
        task.step(p_pointer)
        t += dt
        clock.tick(fr)


if __name__ == "__main__":
    main()
