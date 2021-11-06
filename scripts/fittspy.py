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


xpts = []
ypts = []
zpts = []
collect_data = False

TaskState = Enum("TaskState", "INTRO CALIBRATE TASK SAVE")


# class Polaris:
def polaris_targets_callback(pose_array):
    global xpts, ypts, zpts, collect_data
    x = pose_array.poses[0].position.x
    y = pose_array.poses[0].position.y
    z = pose_array.poses[0].position.z
    # print(f"{x:.4f}", f"{y:4f}", f"{z:.4f}")
    if collect_data:
        xpts.append(x)
        ypts.append(y)
        zpts.append(z)

    # ax.plot3D(xpts, ypts, zpts, 'blue')
    pt = np.array([z, y, z])
    length = 0.01
    translated = pt + np.array([length, length, length])
    # ax.quiver(x, y, z, rotated[0], rotated[1], rotated[2])
    # ax.plot3D(x, y, z, 'blue')
    # fig.canvas.draw()
    # fig.canvas.flush_events()


def read_polaris():
    global xpts, ypts, zpts, collect_data
    fig = plt.figure()
    ax = plt.axes(projection='3d')

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

    task_state = TaskState.INTRO
    print(task_state)

    calibration_points = []
    q = None
    # Main loop
    running = True
    clock = pygame.time.Clock()
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
                task_state = TaskState.CALIBRATE
                plt.ion()
                plt.show()

        elif task_state == TaskState.CALIBRATE:
            collect_data = True
            print(len(xpts))
            if len(xpts) > 0:
                print("Computing plane")
                np_x = np.array(xpts)
                np_y = np.array(ypts)
                np_z = np.array(zpts)
                pts = np.vstack([np_x, np_y, np_z])
                # subtract out the centroid and take the SVD
                centroid = np.mean(pts, axis=1, keepdims=True)
                print(centroid)
                svd = np.linalg.svd(pts - centroid)
                # Extract the left singular vectors
                left = svd[0]
                if len(left) > 0:
                    normal = left[:,-1].astype(np.float64)
                    # print(normal)
                    centroid = centroid[:, 0].astype(np.float64)
                    shifted = centroid + normal
                    print(shifted)
                    ax.plot3D(pts[0], pts[1], pts[2], 'blue')
                    if q is not None:
                        q.remove()
                    q = ax.quiver(centroid[0], centroid[1], centroid[2],
                            normal[0], normal[1], normal[2], length=0.2, normalize=True)
                    # ax.plot3D(centroid[0], centroid[1], centroid[2], 'red')

                    fig.canvas.draw()
                    fig.canvas.flush_events()


            done_calibration = False
            if done_calibration:
                task_state = TaskState.TASK
                cpos, crad = draw_random_circle(screen, (255, 0, 0))

        elif task_state == TaskState.TASK:
            mx, my = pygame.mouse.get_pos()
            # print(dist(mx, my, cpos[0], cpos[1]))
            if dist(mx, my, cpos[0], cpos[1]) < crad:
                print("Success")
                cpos, crad = draw_random_circle(screen, (255, 255, 255))
        elif task_state == TaskState.SAVE:
            pass

        pygame.display.flip()
        pygame.display.update()
        clock.tick(60)
        #
        running = check_pygame_done()


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
