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
sys.path.append('/home/tucker/thesis/ros_workspace/src/fitts_task/scripts')
from util import Trajectory, Timepoint, Trial, all_pts_close
from geometry_msgs.msg import PoseArray, Pose, Point, Quaternion
from std_msgs.msg import Header, Time



class FittsTaskHuman(FittsTask):
    def __init__(self, screen_size, screen_res, intro_text=("",), n_trials=20, frame_rate=60, background_color=(0, 0, 0)):
        super().__init__(screen_size, screen_res, render=True, n_trials=n_trials, frame_rate=frame_rate,
                         background_color=background_color)

        self.intro_text = intro_text
        render_lines_of_text(self.screen, )

    def step(self):
        if self.state == TaskState.INTRO:
            self.intro_step()
        elif self.state == TaskState.CALIBRATE:
            self.calibrate_step()
        elif self.state == TaskState.TASK:
            self.task_step()

        self._pygame_step()

    def intro_step(self):
        render_lines_of_text(self.screen, self.intro_text)
        key_pressed = pygame.key.get_pressed()
        if key_pressed[pygame.K_RETURN]:
            self.screen.fill(self.background_color)
            self.state = FittsTask.TaskState.CALIBRATE

    def calibrate_step(self):
        """
        With the marker placed at the origin of the workspace (world) coordinate frame, we collect
        some data to create the camera to world matrix. We invert that and apply it to 3D points
        collected from the camera in order to map those points to xy plane in camera space. Then, given
        the known size of the workspace and the known size of the screen, we directly map those points
        in the xy plane to the screen's pixel coordinates
        """
        self.collect_data = True
        self.t += 1 / self.frame_rate
        if len(self.pts) > 0:
            self.screen.fill(self.background_color)
            render_lines_of_text(self.screen, [f"Calibrating for 5s. t={self.t:.3f}"])
            if self.t > 5:
                self.make_transformation_matrix()
                self.to_task()
        else:
            render_lines_of_text(self.screen, ["Cannot see marker"])
            self.t = 0

    def make_transformation_matrix(self):
        # Construct the transformation matrix
        self.ws_origin = np.mean(np.array(self.pts), axis=0)
        self.ws_q = np.quaternion(np.mean(np.array(self.orientations)))

        print(self.ws_origin, self.ws_q)
        # Transformation matrix from camera frame workspace frame
        R_CW = quaternion.as_rotation_matrix(self.ws_q.inverse())
        self.T[:3, :3] = R_CW
        self.T[:3, 3] = np.matmul(R_CW, -self.ws_origin[:3])
        self.T[3, 3] = 1

    def pt_3D_to_screen(self, pt):
        ws_pt = np.matmul(self.T, pt)[:3]
        q = np.zeros(4)
        pa = PoseArray(Header(0, rospy.get_rostime(), "polaris_link"), [Pose(Point(ws_pt[0], ws_pt[1], ws_pt[2]), q)])
        self.pub.publish(pa)

        print(f"{ws_pt[0]:.2f}, {ws_pt[1]:.2f}, {ws_pt[2]:.2f}")
        print(self.ws_q * self.ws_q.inverse())

        # map value to screen
        # Check tool definition file for axis configuration
        screenx = ws_pt[2] * self.xscl
        screeny = ws_pt[1] * self.yscl
        return screenx, screeny

    def to_task(self):
        self.pts = []
        self._draw_init_circle()
        self.trajectory = Trajectory(self.circle_pos, self.circle_rad)
        self.state = FittsTask.TaskState.TASK