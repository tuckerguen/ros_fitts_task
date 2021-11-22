#!/home/tucker/anaconda3/bin/python3
import os
import sys
import math
import rospy
import rospkg
import pickle
import pygame
import random
import quaternion
import numpy as np
from enum import Enum
from util import render_lines_of_text
from geometry_msgs.msg import PoseArray

INTRO_TEXT = ["Performing the fitts task:",
              "1. Calibrate the motion capture",
              "Place the motion tracker at the origin of the workspace"
              "Press enter and wait until the system finds the origin"
              "2. Perform the task",
              "   A red circle will appear on the screen. Move the cursor to the red circle",
              "   Immediately after, a white circle will appear",
              "   AS QUICKLY AS POSSIBLE, move the cursor to the white circle"]

TaskState = Enum("TaskState", "INTRO CALIBRATE TASK SAVE EXIT")


class FittsTaskNode:
    def __init__(self, screen_size, n_trials=3, frame_rate=100, background_color=(0, 0, 0)):
        self.pts = []
        self.orientations = []
        self.collect_data = False
        self._ros_init()

        self._pygame_init()
        self.frame_rate = frame_rate
        self.background_color = background_color
        w, h = pygame.display.get_surface().get_size()
        self.screen_w = w
        self.screen_h = h

        render_lines_of_text(self.screen, INTRO_TEXT)

        self.state = TaskState.INTRO

        self.circle_pos = (0, 0)
        self.circle_rad = 0
        self.dt = 1/frame_rate
        self.t = 0.0
        self.is_init_circle = True
        self.init_circle_color = (255, 0, 0)
        self.target_circle_color = (0, 0, 255)
        self.radius_range = (10, 60)
        self.pointer = (w // 2, h // 2)
        self.n_trials = n_trials

        self.ws_origin = np.zeros(3)
        self.ws_q = np.quaternion(0, 0, 0, 1)
        self.T = np.zeros((4, 4))
        self.xscl = self.screen_w / screen_size[0]
        self.yscl = self.screen_h / screen_size[1]

        self.trajectories = []

    def step(self):
        # self.all_step_render()
        if self.state == TaskState.INTRO:
            self.intro_step()
        elif self.state == TaskState.CALIBRATE:
            self.calibrate_step()
        elif self.state == TaskState.TASK:
            self.task_step()

        self._pygame_step()

    def intro_step(self):
        render_lines_of_text(self.screen, INTRO_TEXT)
        key_pressed = pygame.key.get_pressed()
        if key_pressed[pygame.K_RETURN]:
            self.screen.fill(self.background_color)
            self.state = TaskState.CALIBRATE

    def calibrate_step(self):
        self.collect_data = True
        self.t += 1 / self.frame_rate
        if len(self.pts) > 0:
            self.screen.fill(self.background_color)
            render_lines_of_text(self.screen, [f"Calibrating for 5s. t={self.t:.3f}"])
            if self.t > 5:
                # Construct the transformation matrix
                self.ws_origin = np.mean(np.array(self.pts))
                self.ws_q = np.mean(np.array(self.orientations))
                R = quaternion.as_rotation_matrix(self.ws_q)
                self.T[:3, :3] = R
                self.T[3, 3] = 1
                self.to_task()
        else:
            render_lines_of_text(self.screen, ["Cannot see marker"])
            self.t = 0

    def to_task(self):
        self.pts = []
        self._draw_init_circle()
        self.trajectory = Trajectory(self.circle_pos, self.circle_rad)
        self.state = TaskState.TASK

    def task_step(self):
        # Render correct circle every step
        self.screen.fill(self.background_color)
        circle_color = self.init_circle_color if self.is_init_circle else self.target_circle_color
        pygame.draw.circle(self.screen, circle_color, self.circle_pos, self.circle_rad, 0)

        if len(self.pts) > 0:
            # For testing purposes (aligned with bag files)
            screenx, screeny = self._get_test_transformed_pt()

            # Compute the screen space point
            # pts = np.ones(4)
            curr_pt = self.pts[-1]
            # pts[:3] = curr_pt - self.ws_origin
            # screen_pt = np.matmul(self.T, pts)
            # screenx = screen_pt[2] * self.xscl
            # screeny = screen_pt[1] * self.yscl
            if not np.isnan(screenx) and not np.isnan(screeny):
                self.pointer = (int(screenx), int(screeny)-70)
                self.pointer = pygame.mouse.get_pos()

            # Draw user pointer
            pygame.draw.circle(self.screen, (0, 255, 0), [self.pointer[0], self.pointer[1]], 5, 0)

            if self._pointer_in_circle():
                self.is_init_circle = not self.is_init_circle
                if self.is_init_circle:
                    print("Adding trajectory to trajectories")
                    self.trajectories.append(self.trajectory)
                    self._draw_init_circle()
                    self.t = 0
                else:
                    print("Initializing trajectory")
                    if len(self.trajectories) < self.n_trials:
                        self._draw_random_circle()
                        self.t = 0
                        self.trajectory = Trajectory(self.circle_pos, self.circle_rad)
                        tp = Timepoint(self.t, self.pointer[0], self.pointer[1], curr_pt)
                        self.trajectory.add_timepoint(tp)
                    else:
                        self.is_init_circle = True
                        self.save()
                        self.exit()
            else:
                if not self.is_init_circle:
                    print("Adding timepoint")
                    self.t += self.dt
                    print(self.t)
                    tp = Timepoint(self.t, self.pointer[0], self.pointer[1], curr_pt)
                    tp.print()
                    self.trajectory.add_timepoint(tp)

    def save(self):
        print("saving")
        rospack = rospkg.RosPack()
        pkl_path = os.path.join(rospack.get_path('fitts_task'), "trajectories", "test_traj.pkl")
        with open(pkl_path, 'wb') as f:
            pickle.dump(self.trajectories, f)
        self.state = TaskState.EXIT

    def exit(self):
        pygame.quit()
        sys.exit(0)

    def _get_test_transformed_pt(self):
        pts = np.ones(4)
        ws_origin = np.array([0.1217, 0.2804, -0.9552])
        pts[:3] = self.pts[-1] - ws_origin
        table_plane_q = np.quaternion(0.0399, 0.1631, -0.2353, 0.9572)
        R_ws_yz = quaternion.as_rotation_matrix(table_plane_q.inverse())
        T_ws_yz = np.zeros((4, 4))
        T_ws_yz[:3, :3] = R_ws_yz
        T_ws_yz[3, 3] = 1

        rot_pt = np.matmul(T_ws_yz, pts)
        xscl = 1920 / 0.3683
        yscl = 1080 / 0.2286
        screenx = rot_pt[2] * xscl
        screeny = rot_pt[1] * yscl
        return screenx, screeny

    def _draw_init_circle(self):
        self.screen.fill(self.background_color)
        self.circle_rad = 10
        self.circle_pos = (10, self.screen.get_height() // 2 - self.circle_rad)
        pygame.draw.circle(self.screen, self.init_circle_color, self.circle_pos, self.circle_rad)

    def _draw_random_circle(self):
        self.circle_rad = random.randint(*self.radius_range)
        self.circle_pos = (random.randint(self.circle_rad, self.screen_w - self.circle_rad),
                           random.randint(self.circle_rad, self.screen_h - self.circle_rad))
        self.screen.fill(self.background_color)
        pygame.draw.circle(self.screen, self.target_circle_color, self.circle_pos, self.circle_rad, 0)

    def _pointer_in_circle(self):
        distance = math.sqrt((self.pointer[0] - self.circle_pos[0]) ** 2 + (self.pointer[1] - self.circle_pos[1]) ** 2)
        return distance < self.circle_rad

    def _pygame_init(self):
        pygame.init()
        self.screen = pygame.display.set_mode()
        pygame.display.set_caption('Fitts task')
        pygame.font.init()
        self.pg_clock = pygame.time.Clock()

    def _pygame_step(self):
        pygame.display.flip()
        pygame.display.update()
        self.pg_clock.tick(self.frame_rate)

    def is_pygame_running(self):
        running = True
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
        return running

    def _ros_init(self):
        rospy.init_node('read_polaris', anonymous=True)
        rospy.Subscriber("polaris_sensor/targets", PoseArray, self.polaris_callback, queue_size=1)

    def polaris_callback(self, pose_array):
        pos = pose_array.poses[0].position
        position = np.array([pos.x, pos.y, pos.z])
        q = pose_array.poses[0].orientation
        orientation = np.quaternion(q.w, q.x, q.y, q.z)
        if self.collect_data:
            if not np.any(np.isnan(position)):
                self.pts.append(position)
                self.orientations.append(orientation)


class Timepoint:
    def __init__(self, t, x, y, pt_3d):
        self.t = t
        self.x = x
        self.y = y
        self.pt_3d = pt_3d

    def print(self):
        print(f"{self.t:.2f}: ({self.x}, {self.y}), ({self.pt_3d[0]:0.3f}, {self.pt_3d[1]:0.3f}, {self.pt_3d[2]:0.3f})")


class Trajectory:
    def __init__(self, cpos, crad):
        self.cpos = cpos
        self.crad = crad
        self.data = []

    def add_timepoint(self, tp):
        self.data.append(tp)


def main():
    node = FittsTaskNode(screen_size=(0.6858, 0.3556))
    while node.is_pygame_running():
        node.step()


if __name__ == "__main__":
    main()
