import pygame
from typing import Tuple
from pygame import Color
from typing import TYPE_CHECKING
if TYPE_CHECKING:
    from task import FittsTask


class FittsTaskViewer:
    def __init__(self,
                 task: 'FittsTask',
                 home_color: Tuple[int] = (255, 0, 0),
                 target_color: Tuple[int] = (0, 0, 255),
                 bg_color: Tuple[int] = (0, 0, 0)):
        self.task = task
        if task.ndim == 3:
            raise NotImplementedError("3D rendering of fitts task is not supported")

        self.home_color = Color(*home_color)
        self.target_color = Color(*target_color)
        self.bg_color = Color(*bg_color)
        self._pygame_init()

    def render(self):
        color = self.home_color if self.task.is_home_state else self.target_color
        self.screen.fill(self.bg_color)
        pygame.mouse.set_visible(False)
        pygame.draw.circle(self.screen, color, self.task.target_pos, self.task.target_size, 0)
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
