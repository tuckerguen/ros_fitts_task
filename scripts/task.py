#!/home/tucker/anaconda3/bin/python3
import sys
import numpy as np
from enum import Enum
from typing import Tuple
from viewer import FittsTaskViewer


class FittsTask:
    CircleState = Enum("CircleState", [("HOME", True), ("TARGET", False)])

    def __init__(self,
                 workspace_lims: Tuple[Tuple[int, ...], ...],
                 target_size_lims: Tuple[float, ...],
                 home_pos: Tuple[float, ...],
                 home_size: float,
                 delay: int,
                 n_trials: int = 20,
                 render: bool = True):

        assert 0 < len(workspace_lims) < 3, "Workspace should be 1D, 2D, or 3D"
        assert len(workspace_lims) == len(home_pos), "Home position must have same dimensionality as workspace"
        assert len(target_size_lims) == 2, "Target size limits must be a tuple, (min, max)"
        assert target_size_lims[0] < target_size_lims[1], "First target size must be smaller than second target size"

        self.workspace_lims = workspace_lims
        self.ndim = len(workspace_lims)
        self.n_trials = n_trials

        self.target_pos = (0,) * self.ndim
        self.target_size = 0
        self.is_home_state = FittsTask.CircleState.HOME
        self.home_size = home_size
        self.home_pos = home_pos
        self.target_size_lims = target_size_lims

        self.n_trials = n_trials
        self.delay = delay
        self.delay_steps = 0

        self.pointer_pts = []

        self.render = render
        if self.render:
            self.viewer = FittsTaskViewer(self)

    def step(self, p_pointer: np.ndarray) -> Tuple[bool, np.ndarray, float]:
        # Render circle every step
        if self.viewer:
            self.viewer.render()

        success = False
        # Once the pointer enters the circle
        if self._pointer_in_circle(p_pointer):
            # Wait inside the circle for delay steps
            if self.delay_steps > self.delay:
                # Waited long enough, flip state
                self.is_home_state = not self.is_home_state
                if self.is_home_state == FittsTask.CircleState.HOME:
                    # Set the circle to home
                    self.target_pos = self.home_pos
                    self.target_size = self.home_size
                else:
                    # Set the circle to a new random point
                    self.target_pos = np.random.uniform(self.workspace_lims, self.ndim)
                    self.target_size = np.random.uniform(self.target_size_lims, 1)
            else:
                self.delay_steps += 1

        return success, self.target_pos, self.target_size

    def exit(self):
        if self.viewer:
            self.viewer.exit()
        sys.exit(0)

    def _pointer_in_circle(self, p_pointer: np.ndarray):
        distance = np.linalg.norm(p_pointer - self.target_pos)
        return distance < self.target_size

