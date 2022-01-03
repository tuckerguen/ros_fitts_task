#!/home/tucker/anaconda3/bin/python3
from collections import deque
from typing import Tuple

import dill
import numpy as np

from viewer import FittsTaskViewer


class FittsTask:
    def __init__(self,
                 workspace_lims: Tuple[Tuple[float, ...], ...],
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

        self._validate_inputs(workspace_lims, home_pos, target_size_lims)

        self.workspace_lims = np.array(workspace_lims)
        self.ndim = len(workspace_lims)

        if not all([workspace_lims[i][0] <= home_pos[i] <= workspace_lims[i][1] for i in range(self.ndim)]):
            raise ValueError("Home pos must be within the workspace limits")

        self.target_pos = home_pos
        self.target_size = home_size
        self.is_home_state = True
        self.home_size = home_size
        self.home_pos = home_pos
        self.target_size_lims = target_size_lims

        self.steps_to_wait = steps_to_wait
        self.target_wait = steps_to_wait
        self.stationary_tolerance = stationary_tolerance

        self.pointer_pts = []

        self.render = render
        self.viewer = None
        self.render_kwargs = render_kwargs
        self.try_render()

    def step(self, p_pointer: np.ndarray) -> Tuple[bool, bool, Tuple[float, ...], float]:
        if p_pointer is None:
            return True, False, self.target_pos, self.target_size

        self.pointer_pts.append(p_pointer)
        self.try_render()

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
                    self.steps_to_wait = np.random.randint(10, 50, 1)[0]
                else:
                    # Set the circle to a new random point
                    self.target_size = np.random.uniform(self.target_size_lims[0], self.target_size_lims[1], 1)[0]
                    # Avoid off-screen points
                    lower_lim = self.workspace_lims[:, 0] + self.target_size
                    upper_lim = self.workspace_lims[:, 1] - self.target_size
                    self.target_pos = np.random.uniform(lower_lim, upper_lim, 2)
                    self.steps_to_wait = self.target_wait

        return self.is_home_state, success, self.target_pos, self.target_size

    def try_render(self):
        if self.render and not self.viewer:
            self.viewer = FittsTaskViewer(self, **self.render_kwargs)
        if self.render:
            self.viewer.render()

    def pickle(self, f):
        with open(f, "wb") as pkl_file:
            setattr(self, "viewer", None)
            dill.dump(self, pkl_file)

    def remove_viewer(self):
        self.viewer = None

    def exit(self):
        if self.viewer:
            self.viewer.exit()

    def _pointer_in_circle(self):
        distance = np.linalg.norm(self.pointer_pts[-1] - self.target_pos)
        return distance < self.target_size

    def _all_pts_close(self):
        if not self.pointer_pts or len(self.pointer_pts) < self.steps_to_wait:
            return False
        pts = np.array(self.pointer_pts[-self.steps_to_wait:])
        in_tol = [np.all(np.abs(pts[:, i] - np.mean(pts[:, i])) < self.stationary_tolerance) for i in range(self.ndim)]
        return np.all(in_tol)

    @staticmethod
    def _validate_inputs(workspace_lims, home_pos, target_size_lims):
        if not (0 < len(workspace_lims) < 3):
            raise ValueError("Workspace should be 1D, 2D, or 3D")
        if len(workspace_lims) != len(home_pos):
            raise ValueError("Home position must have same dimensionality as workspace")
        if len(target_size_lims) != 2:
            raise ValueError("Target size limits must be a tuple, (min, max)")
        if target_size_lims[0] > target_size_lims[1]:
            raise ValueError("First target size must be smaller than second target size")


def from_pickle(f):
    with open(f, "rb") as pkl_file:
        return dill.load(pkl_file)


class TaskWrapper:
    def __init__(self, task):
        self.task = task

    def __getattr__(self, item):
        return self.task.__getattribute__(item)

    def remove_viewer(self):
        self.task.viewer = None


class TimeDelayWrapper(TaskWrapper):
    def __init__(self, task, delay_steps):
        super().__init__(task)
        if delay_steps == 0:
            self.delay_buffer = None
        else:
            self.delay_buffer = deque(maxlen=delay_steps)
            for _ in range(delay_steps):
                self.delay_buffer.append(np.array([0, 0]))

    def step(self, p_pointer: np.ndarray) -> Tuple[bool, bool, Tuple[float, ...], float]:
        if self.delay_buffer:
            pt = self.delay_buffer.pop()
            self.delay_buffer.appendleft(p_pointer)
        else:
            pt = p_pointer
        ret = self.task.step(pt)
        return ret
