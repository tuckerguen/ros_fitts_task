from task import FittsTask
import numpy as np
import pygame

def is_pygame_running():
    running = True
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False
    return running


def main():
    clock = pygame.time.Clock()
    fr = 60
    delay_secs = 1
    dt = 1/fr
    t = 0

    task = FittsTask(workspace_lims=((0, 1920), (0, 1080)),
                     target_size_lims=(10, 20),
                     home_pos=(1080//2, 5),
                     home_size=5,
                     delay=delay_secs * fr,
                     n_trials=5,
                     render=True)
    while t < 100:
        p_pointer = np.array(pygame.mouse.get_pos())
        task.step(p_pointer)
        p_pointer += 1
        t += dt
        clock.tick(fr)


if __name__ == "__main__":
    main()
