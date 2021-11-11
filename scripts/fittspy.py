#!/home/tucker/anaconda3/bin/python3
# license removed for brevity
import sys
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import PointCloud
from geometry_msgs.msg import PoseArray
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

from itertools import product, combinations
import numpy as np
import pygame
import math
import random
from enum import Enum
from time import sleep
from pyransac3d import Plane
import quaternion

ws_lims = [[0,0],[0.33, 0.33]]
screen_lims = [[0,0], [1920,1080]]
xscl = 1920/0.33
yscl = 1080/0.33

xpts = []
ypts = []
zpts = []
collect_data = False

ws_origin = np.array([0.14147, 0.19254, -0.91928])
table_plane_q = np.quaternion(-0.1662, 0.0399, 0.9594, 0.2239)
R_ws_yz = quaternion.as_rotation_matrix(table_plane_q.inverse())
T_ws_yz = np.zeros((4,4))
T_ws_yz[:3, :3] = R_ws_yz
T_ws_yz[3, 3] = 1
T_ws_yz[:3, 3] = ws_origin

TaskState = Enum("TaskState", "INTRO CALIBRATE TASK SAVE")


# class Polaris:
def polaris_targets_callback(pose_array):
    global xpts, ypts, zpts, collect_data
    x = pose_array.poses[0].position.x
    y = pose_array.poses[0].position.y
    z = pose_array.poses[0].position.z
    q = pose_array.poses[0].orientation
    # print(f"{x:.4f}", f"{y:4f}", f"{z:.4f}")
    if collect_data:
        xpts.append(x)
        ypts.append(y)
        zpts.append(z)


def read_polaris():
    global xpts, ypts, zpts, collect_data
    fig = plt.figure()
    # ax = plt.axes(projection='3d')
    # ax.set_xlabel("X")
    # ax.set_ylabel("Y")
    # ax.set_zlabel("Z")

    # ax.axes.set_xlim3d(left=0.02, right=0.14)
    # ax.axes.set_ylim3d(bottom=-1.2, top=-0.7)
    # ax.axes.set_zlim3d(bottom=1.4, top=1.9)

    print("Created sub")
    rospy.init_node('read_polaris', anonymous=True)
    rospy.Subscriber("polaris_sensor/targets", PoseArray, polaris_targets_callback, queue_size=1)

    # Task parameters
    bgcolor = (0, 0, 0)

    # Setup task
    pygame.init()
    screen = pygame.display.set_mode()
    pygame.display.set_caption('Fitts task')

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
    render_lines_of_text(screen, intro_text)

    # init task
    cpos = (0, 0)
    crad = 0
    trajectories = []
    trajectory = None
    dt = 0.05
    t = 0.0

    task_state = TaskState.INTRO
    print(task_state)

    # Main loop
    running = True
    clock = pygame.time.Clock()

    npts = np.array([])

    is_init_circle = True

    while running:
        # print("OUTER", collect_data)
        if task_state == TaskState.INTRO:
            key = pygame.key.get_pressed()
            if key[pygame.K_RETURN]:
                print("Beginning Calibration")
                # Remove text
                screen.fill(bgcolor)
                # Add calibration text
                text = [""]
                # Move on
                task_state = TaskState.TASK
                plt.ion()
                plt.show()

        elif task_state == TaskState.CALIBRATE:
            # Set the initial origin and rotation matrices that are
            # currently global
            collect_data = True
            print(len(xpts))
            if len(xpts) > 3:
                np_x = np.array(xpts)
                np_y = np.array(ypts)
                np_z = np.array(zpts)
                pts = np.vstack([np_x, np_y, np_z, np.ones(np_x.shape[0])])
                # Transform points from table plane to xy plane (relative to camera)
                rot_pts = np.matmul(T_ws_yz, pts)
                rot_pts = rot_pts[:3, :]
                plt.plot(-rot_pts[2], rot_pts[1], 'blue')

                fig.canvas.draw()
                fig.canvas.flush_events()

        elif task_state == TaskState.TASK:
            collect_data = True
            if len(xpts) > 0:
                pt = np.vstack([np.array(xpts[-1]), np.array(ypts[-1]), np.array(zpts[-1]), 1])
                rot_pt = np.matmul(T_ws_yz, pt)
                rot_pt = rot_pt[:3, :]
                screenx = -rot_pt[2] * xscl
                screeny = -rot_pt[1] * yscl
                # plt.plot(-rot_pt[2], rot_pt[1], 'blue')
                # Map to screen coordinates
                print(rot_pt, screenx, screeny)
                if not np.isnan(screeny) and not np.isnan(screeny):
                    # pygame.mouse.set_pos([int(screenx), int(screeny)])
                    screen.fill(bgcolor)
                    pygame.draw.circle(screen, (0, 255, 0), [int(screenx), int(screeny)], 5, 0)

                pygame.draw.circle(screen, (255, 255, 255), cpos, crad, 0)
                mx, my = pygame.mouse.get_pos()
                # print(dist(mx, my, cpos[0], cpos[1]))
                if dist(mx, my, cpos[0], cpos[1]) < crad:
                    if is_init_circle:
                        print("Start")
                        cpos, crad = draw_random_circle(screen, (255, 255, 255))
                        is_init_circle = False
                        sleep(2)
                    else:
                        print("Success")
                        cpos, crad = draw_init_circle(screen)
                        is_init_circle = True
                        t = 0.0

            if not is_init_circle:
                trajectories.append(trajectory)
                trajectory = Trajectory(cpos, crad)

                t += dt
                tp = Timepoint(t, mx, my, 0)
                tp.print()
                trajectory.add_timepoint(tp)

        elif task_state == TaskState.SAVE:
            pass


        pygame.display.flip()
        pygame.display.update()
        clock.tick(100)
        running = check_pygame_done()


class Timepoint:
    def __init__(self, t, x, y, z):
        self.t = t
        self.x = x
        self.y = y
        self.z = z

    def print(self):
        print(f"{self.t:.2f}: ({self.x}, {self.y})")


class Trajectory:
    def __init__(self, cpos, crad):
        self.cpos = cpos
        self.crad = crad
        self.data = []

    def add_timepoint(self, tp):
        self.data.append(tp)


def draw_init_circle(screen, color=(255, 0, 0), bgcolor=(0, 0, 0)):
    crad = 10
    cpos = (10, screen.get_height()//2 - crad)
    screen.fill(bgcolor)
    pygame.draw.circle(screen, color, cpos, crad, 0)
    return cpos, crad

def draw_random_circle(screen, color, bgcolor=(0, 0, 0)):
    crad = random.randint(10, 60)
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
        screen.blit(textsurface, (0, (text_height+line_gap) * i))


def dist(x0, y0, x1, y1):
    return math.sqrt((x0 - x1) ** 2 + (y0 - y1) ** 2)


def check_pygame_done():
    running = True
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False
    return running


if __name__ == '__main__':
    try:
        read_polaris()
    except rospy.ROSInterruptException:
        print("Exception")
