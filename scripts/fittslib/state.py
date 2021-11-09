import pygame
import random
import numpy as np
from pyransac3d import Plane
import matplotlib.pyplot as plt


class StateMachine():
    def __init__(self, initial_state, **state_kwargs):
        self.state = initial_state(state_kwargs)
        self.data = {}

    def set_shared_data(self, **data):
        self.data = data

    def callback(self, pose_array):
        x = pose_array.poses[0].position.x
        y = pose_array.poses[0].position.y
        z = pose_array.poses[0].position.z
        if self.collect_data:
            self.xpts.append(x)
            self.ypts.append(y)
            self.zpts.append(z)


class State:
    def __init__(self, bgcolor=(0, 0, 0)):
        # Setup pygame

        TaskState.bgcolor = bgcolor
        self.state = None

    def loop_step(cls):
        pass

    def step(self):
        end = self.state.loop_step()
        if end:
            self.state = self.state.end()

    @staticmethod
    def callback(self, pose_array):
        x = pose_array.poses[0].position.x
        y = pose_array.poses[0].position.y
        z = pose_array.poses[0].position.z
        if self.collect_data:
            self.xpts.append(x)
            self.ypts.append(y)
            self.zpts.append(z)


class Intro(State):
    def __init__(self):
        super().__init__()
        # Introduction things
        pygame.font.init()  # you have to call this at the start,
        # if you want to use this module.

        intro_text = ["Performing the fitts task:",
                      "1. Calibrate the motion capture",
                      "   Once you hit ENTER, calibration will begin.",
                      "   While keeping the tooltip in contact with the table, move the motion tracker around in "
                      "a relatively large area",
                      "2. Perform the task",
                      "   A red circle will appear on the screen. Move the cursor to the red circle",
                      "   After 3 seconds, a white circle will appear",
                      "   AS QUICKLY AS POSSIBLE, move the cursor to the white circle"]
        render_lines_of_text(self.screen, intro_text)

    @classmethod
    def loop_step(cls):
        key = pygame.key.get_pressed()
        if key[pygame.K_RETURN]:
            return True
        return False

    def end(self):
        print("Beginning Calibration")
        # Remove text
        self.screen.fill(self.bgcolor)
        return Calibrate()


class Calibrate(TaskState):
    def __init__(self):
        super().__init__()
        self.surf = None
        self.fig = plt.figure()
        self.ax = plt.axes(projection='3d')
        self.ax.axes.set_xlim3d(left=0.02, right=0.14)
        self.ax.axes.set_ylim3d(bottom=-1.2, top=-0.7)
        self.ax.axes.set_zlim3d(bottom=1.4, top=1.9)

    @classmethod
    def loop_step(cls):
        if len(self.xpts) > 3:
            np_x = np.array(cls.xpts)
            np_y = np.array(cls.ypts)
            np_z = np.array(cls.zpts)
            pts = np.vstack([np_x, np_y, np_z]).T

            xs = np.linspace(np.min(cls.xpts), np.max(cls.xpts), 100)
            ys = np.linspace(np.min(cls.zpts), np.max(cls.zpts), 100)
            X, Y = np.meshgrid(xs, ys)

            if len(pts) > 0:
                plane_params, inliers = Plane().fit(pts, thresh=0.005, maxIteration=1000)
                if len(plane_params) > 0:
                    a, b, c, d = plane_params
                    Z = (d - a * X + b * Y) / c
                    self.surf = self.ax.plot_surface(X, Y, Z)

            if len(cls.npts) > 10 and np.all(cls.npts[-10:] == cls.npts[-1]):
                return FittsTask

            cls.npts = np.append(cls.npts, len(pts))
            return False

    def end(self):
        return FittsTask()


class FittsTask(TaskState):
    pass


def draw_random_circle(screen, color, bgcolor=(0, 0, 0)):
    crad = random.randint(20, 50)
    w, h = pygame.display.get_surface().get_size()
    cpos = (random.randint(crad, w - crad), random.randint(crad, h - crad))
    screen.fill(bgcolor)
    pygame.draw.circle(screen, color, cpos, crad, 0)
    return cpos, crad


def render_lines_of_text(screen, text_lines):
    text_height = 30
    line_gap = 5
    font = pygame.font.SysFont('Arial', text_height)
    for i, text_line in enumerate(text_lines):
        textsurface = font.render(text_line, False, (255, 255, 255))
        screen.blit(textsurface, (0, (text_height + line_gap) * i))
