import pygame
import numpy as np
from typing import Tuple, Union, TYPE_CHECKING
from pygame import Color
from util import is_pygame_running

if TYPE_CHECKING:
    from task import FittsTask


class FittsTaskViewer:
    def __init__(self,
                 task: 'FittsTask',
                 display_size: Tuple[int, int] = (0, 0),
                 fullscreen: bool = True,
                 pointer_size: int = 5,
                 pointer_color: Union[Tuple[int], Color] = (255, 255, 255),
                 home_color: Union[Tuple[int], Color] = (255, 0, 0),
                 target_color: Union[Tuple[int], Color] = (0, 0, 255),
                 bg_color: Union[Tuple[int], Color] = (0, 0, 0)):
        """
        A viewer to render a fitts task
        :param task: The fitts task to render
        :param pointer_size: Size of the rendered pointer
        :param pointer_color: Color of the pointer
        :param home_color: Color of the home point
        :param target_color: Color of the target
        :param bg_color: Color of the background display
        """
        self.task = task
        if task.ndim == 3:
            raise NotImplementedError("3D rendering of fitts task is not supported")

        self.fullscreen = fullscreen
        self.pointer_size = pointer_size
        self.display_size = display_size
        self.pointer_color = self._handle_color(pointer_color)
        self.home_color = self._handle_color(home_color)
        self.target_color = self._handle_color(target_color)
        self.bg_color = self._handle_color(bg_color)
        self._pygame_init()

    @staticmethod
    def _handle_color(color: Union[Tuple[int], Color]):
        return Color(*color) if isinstance(color, tuple) else color

    def render(self):
        if not is_pygame_running():
            raise RuntimeError("Viewer failed. pygame display failed or was closed")

        self.screen.fill(self.bg_color)
        pygame.mouse.set_visible(False)

        # Draw the current circle
        color = self.home_color if self.task.is_home_state else self.target_color
        target_screen_pos = self._map_to_screen(self.task.target_pos)
        size_scl = min(self.display_size) / min(self.task.workspace_lims[:, 1])
        pygame.draw.circle(self.screen, color, target_screen_pos, self.task.target_size * size_scl, 0)

        # Draw the pointer
        if self.task.pointer_pts:
            pointer_screen_pos = self._map_to_screen(self.task.pointer_pts[-1])
            pygame.draw.circle(self.screen, self.pointer_color, pointer_screen_pos, self.pointer_size, 0)

        pygame.display.flip()
        pygame.display.update()

    def _map_to_screen(self, pt):
        if pt is not None:
            pt = pt - self.task.workspace_lims[:, 0]
            scl = np.array(self.display_size) / (self.task.workspace_lims[:, 1] - self.task.workspace_lims[:, 0])
            screen_pt = pt * scl
            return np.round(screen_pt, 0).astype(np.int32)
        return 0, 0

    def _pygame_init(self):
        pygame.init()
        if self.fullscreen:
            self.screen = pygame.display.set_mode((0, 0), pygame.FULLSCREEN)
        else:
            self.screen = pygame.display.set_mode(self.display_size)
        pygame.display.set_caption('Fitts task')
        pygame.font.init()
        self.pg_clock = pygame.time.Clock()

    def exit(self):
        pygame.quit()
