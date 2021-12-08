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
                 steps_to_wait: int,
                 stationary_tolerance: float = 2,
                 render: bool = True,
                 render_kwargs=None):
        """
        Simulates a 1D, 2D, or 3D fitts task where an agent must begin at a
        home location and move its point to the target location. Once the
        target is reached, and the agent has been stationary in the target
        for a period of time, the agent must return to home and repeat
        :param workspace_lims: Limits of the workspace. In a 2D example: ((0, 10), (0, 10)) targets are generated
                               randomly within a 10x10 workspace
        :param target_size_lims: Limits of the size of the target. The size of the target is chosen randomly
        :param home_pos: The position in the workspace of the home
        :param home_size: The size of the home position
        :param steps_to_wait: The number of calls to FittsTask.step() the agent must stay stationary within the
                              home or target circle for before the task is considered complete. This parameter is given
                              in terms of the number of calls to step() because the task should have no notion of time.
                              By allowing the encapsulating program to specify the time step and the length of time per
                              step, we limit the likelihood of having not synchronous simulation.
        :param stationary_tolerance: The tolerance to which changes in position are considered "stationary"
                                     Points that fall within the stationary_tolerance of one another are considered
                                     to be the same.
        :param render: Render the task to the screen
        """
        if render_kwargs is None:
            render_kwargs = {}

        assert 0 < len(workspace_lims) < 3, "Workspace should be 1D, 2D, or 3D"
        assert workspace_lims[0][0] < home_pos[0] < workspace_lims[0][1] \
               and workspace_lims[1][0] < home_pos[1] < workspace_lims[1][1], "Home pos must be within the workspace limits"
        assert len(workspace_lims) == len(home_pos), "Home position must have same dimensionality as workspace"
        assert len(target_size_lims) == 2, "Target size limits must be a tuple, (min, max)"
        assert target_size_lims[0] < target_size_lims[1], "First target size must be smaller than second target size"

        self.workspace_lims = np.array(workspace_lims)
        self.ndim = len(workspace_lims)

        self.target_pos = home_pos
        self.target_size = home_size
        self.is_home_state = FittsTask.CircleState.HOME
        self.home_size = home_size
        self.home_pos = home_pos
        self.target_size_lims = target_size_lims

        self.steps_to_wait = steps_to_wait
        self.stationary_tolerance = stationary_tolerance

        self.pointer_pts = []

        self.render = render
        if self.render:
            self.viewer = FittsTaskViewer(self, **render_kwargs)

    def step(self, p_pointer: np.ndarray) -> Tuple[bool, np.ndarray, float]:
        self.pointer_pts.append(p_pointer)
        # Render circle every step
        if self.viewer:
            self.viewer.render()

        success = False
        # Once the pointer enters the circle
        if self._pointer_in_circle():
            # Ensure the pointer is stationary
            if self._all_pts_close():
                # Waited long enough, flip state
                self.is_home_state = not self.is_home_state
                if self.is_home_state:
                    # Set the circle to home
                    self.target_pos = self.home_pos
                    self.target_size = self.home_size
                    success = True
                else:
                    # Set the circle to a new random point
                    self.target_size = np.random.uniform(self.target_size_lims[0], self.target_size_lims[1], 1)[0]
                    # Avoid off screen points
                    lower_lim = self.workspace_lims[:, 0] + self.target_size
                    upper_lim = self.workspace_lims[:, 1] - self.target_size
                    self.target_pos = np.random.uniform(lower_lim, upper_lim, 2)

        return success, self.target_pos, self.target_size

    def exit(self):
        if self.viewer:
            self.viewer.exit()
        sys.exit(0)

    def _pointer_in_circle(self):
        distance = np.linalg.norm(self.pointer_pts[-1] - self.target_pos)
        return distance < self.target_size

    def _all_pts_close(self):
        if not self.pointer_pts:
            return True
        pts = np.array(self.pointer_pts[-self.steps_to_wait:])
        in_tol = [np.all(np.abs(pts[:, i] - np.mean(pts[:, i])) < self.stationary_tolerance) for i in range(self.ndim)]
        return np.all(in_tol)
