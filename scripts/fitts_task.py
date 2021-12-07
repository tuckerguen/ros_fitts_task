#!/home/tucker/anaconda3/bin/python3
import os
import sys
import time
import math
import json
import rospy
import rospkg
import pickle
import pygame
import random
import quaternion
import numpy as np
from enum import Enum
from datetime import datetime
from typing import Tuple

sys.path.append('/home/tucker/thesis/ros_workspace/src/fitts_task/scripts')
from util import Trajectory, Timepoint, Trial, all_pts_close


class FittsTask:
    CircleState = Enum("CircleState", [("HOME", True), ("TARGET", False)])

    def __init__(self, workspace_lims: np.ndarray, target_size_lims: np.ndarray, home: np.ndarray, delay: int,
                 n_trials: int = 20,
                 render: bool = True):
        assert 0 < len(workspace_lims) < 3, "Workspace should be 1D, 2D, or 3D"
        assert len(workspace_lims) == len(home), "Home position must have same dimensionality as workspace"
        assert len(target_size_lims) == 2, "Target size limits must be a tuple, (min, max)"
        assert target_size_lims[0] < target_size_lims[1], "First target size must be smaller than second target size"

        self.workspace_lims = workspace_lims
        self.ndim = len(workspace_lims)
        self.n_trials = n_trials

        self.circle_pos = (0,) * self.ndim
        self.circle_rad = 0
        self.circle_state = FittsTask.CircleState.HOME
        self.home = home
        self.target_size_lims = target_size_lims

        self.n_trials = n_trials
        self.delay = delay
        self.delay_steps = 0

        self.pointer_pts = []

        self.render = render
        if self.render:
            self.viewer = FittsTaskViewer(self)

    def step(self, p_pointer: np.ndarray) -> Tuple[...]:
        # Render circle every step
        if self.viewer:
            self.viewer.render()

        # Once the pointer enters the circle
        if self._pointer_in_circle(p_pointer):
            # Wait inside the circle for delay steps
            if self.delay_steps > self.delay:
                # Waited long enough, flip state
                self.circle_state = not self.circle_state
                if self.circle_state == FittsTask.CircleState.HOME:
                    # Set the circle to home
                    self.circle_pos = self.home
                else:
                    # Set the circle to a new random point
                    self.circle_pos = np.random.uniform(self.workspace_lims, self.ndim)
            else:
                self.delay_steps += 1

        return

    def exit(self):
        if self.viewer:
            self.viewer.exit()
        sys.exit(0)

    def _pointer_in_circle(self, p_pointer: np.ndarray):
        distance = np.linalg.norm(p_pointer - self.circle_pos)
        return distance < self.circle_rad


def a(x: np.ndarray):
    print(x)


def main():
    # node = FittsTaskNode(screen_size=(0.6858, 0.3556))  # Big monitor (27")
    # node = FittsTaskNode(screen_size=(0.505, 0.2805))  # Small monitor (21.5")
    screen_size = (0.37, 0.23)  # laptop
    node = FittsTaskNode(screen_res=(1920, 1080))
    while node.is_pygame_running():
        node.step()


if __name__ == "__main__":
    main()
