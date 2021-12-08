import pygame
from typing import Tuple
from pygame import Color
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from task import FittsTask

def is_pygame_running():
    running = True
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False
    return running

class FittsTaskViewer:
    def __init__(self,
                 task: 'FittsTask',
                 pointer_size: int = 5,
                 pointer_color: Tuple[int] = (255, 255, 255),
                 home_color: Tuple[int] = (255, 0, 0),
                 target_color: Tuple[int] = (0, 0, 255),
                 bg_color: Tuple[int] = (0, 0, 0)):
        self.task = task
        if task.ndim == 3:
            raise NotImplementedError("3D rendering of fitts task is not supported")

        self.pointer_size = pointer_size
        self.pointer_color = Color(*pointer_color)
        self.home_color = Color(*home_color)
        self.target_color = Color(*target_color)
        self.bg_color = Color(*bg_color)
        self._pygame_init()

    def render(self):
        if not is_pygame_running():
            raise RuntimeError("Viewer failed. pygame display failed or was closed")

        color = self.home_color if self.task.is_home_state else self.target_color
        self.screen.fill(self.bg_color)
        pygame.mouse.set_visible(False)
        pygame.draw.circle(self.screen, color, (int(self.task.target_pos[0]), int(self.task.target_pos[1])),
                           self.task.target_size, 0)
        pygame.draw.circle(self.screen, self.pointer_color, (int(self.task.pointer_pts[-1][0]),
                                                             int(self.task.pointer_pts[-1][1])),
                           self.pointer_size, 0)
        pygame.display.flip()
        pygame.display.update()

    def _pygame_init(self):
        pygame.init()
        self.screen = pygame.display.set_mode((0, 0), pygame.FULLSCREEN)
        pygame.display.set_caption('Fitts task')
        pygame.font.init()
        self.pg_clock = pygame.time.Clock()

    def exit(self):
        pygame.quit()
