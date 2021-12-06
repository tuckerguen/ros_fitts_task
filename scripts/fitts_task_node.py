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
from util import render_lines_of_text, Trajectory, Timepoint, Trial, all_pts_close
from geometry_msgs.msg import PoseArray, Pose, Point, Quaternion
from std_msgs.msg import Header, Time

DEBUG = True

INTRO_TEXT = ["Performing the fitts task:",
              "1. Calibrate the motion capture",
              "Place the motion tracker at the origin of the workspace",
              "Press enter and wait until the system finds the origin",
              "2. Perform the task",
              "   A red circle will appear on the screen. Move the cursor to the red circle",
              "   Immediately after, a white circle will appear",
              "   AS QUICKLY AS POSSIBLE, move the cursor to the white circle"]

TaskState = Enum("TaskState", "INTRO CALIBRATE TASK SAVE EXIT")

"""
Data comes in at 20Hz, program
"""


class FittsTaskNode:
    def __init__(self, screen_size, trial_dirname="tucker", n_trials=20, frame_rate=60, background_color=(0, 0, 0)):
        self.trial_dirname = trial_dirname

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
        self.dt = 1 / frame_rate
        self.t = 0.0
        self.is_init_circle = True
        self.init_circle_color = (255, 0, 0)
        self.target_circle_color = (0, 0, 255)
        self.radius_range = (10, 60)
        self.pointer = (w // 2, h // 2)
        self.n_trials = n_trials
        self.delay = 1
        self.delay_timer = 0

        self.ws_origin = np.zeros(3)
        self.ws_q = np.quaternion(0, 0, 0, 1)
        self.T = np.zeros((4, 4))
        self.xscl = self.screen_w / screen_size[0]
        self.yscl = self.screen_h / screen_size[1]

        self.trial = Trial()
        self.trajectory = None

        self.pointer_pts = []

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
        self.state = TaskState.TASK

    def task_step(self):
        # Render circle every step
        self.screen.fill(self.background_color)
        circle_color = self.init_circle_color if self.is_init_circle else self.target_circle_color
        pygame.draw.circle(self.screen, circle_color, self.circle_pos, self.circle_rad, 0)

        if len(self.pts) > 0:
            # Compute the screen space point
            curr_pt = self.pts[-1]
            screenx, screeny = self.pt_3D_to_screen(curr_pt)

            print(screenx, screeny)
            if not np.isnan(screenx) and not np.isnan(screeny):
                if DEBUG:
                    self.pointer = pygame.mouse.get_pos()
                    self.pointer_pts.append(np.array(self.pointer))
                else:
                    self.pointer = (int(screenx), int(screeny))

            # Draw user pointer
            pygame.mouse.set_visible(False)
            pygame.draw.circle(self.screen, (0, 255, 0), [self.pointer[0], self.pointer[1]], 5, 0)

            # Handle trajectory and circle logic
            # TODO: Clean up this logic
            if self._pointer_in_circle():
                # Ensure that the pointer is (roughly) stopped in the target circle
                if not self.is_init_circle:
                    if not all_pts_close(self.pointer_pts[-int(self.delay // self.dt):], 2):
                        self.delay_timer = 0
                    else:
                        self.delay_timer = self.delay
                # If we've waited long enough in the circle
                if self.delay_timer >= self.delay:
                    # Flip state
                    self.is_init_circle = not self.is_init_circle
                    if self.is_init_circle:
                        # Draw the init circle
                        self.trial.add_trajectory(self.trajectory)
                        self._draw_init_circle()
                        self.t = 0
                        self.delay_timer = 0
                    else:
                        # Show new target
                        print(f"DELAY {self.delay_timer}")
                        if len(self.trial.trajectories) < self.n_trials:
                            self._draw_random_circle()
                            self.t = 0
                            self.trajectory = Trajectory(self.circle_pos, self.circle_rad)
                            tp = Timepoint(self.t, self.pointer[0], self.pointer[1], curr_pt)
                            self.trajectory.add_timepoint(tp)
                            self.delay_timer = 0
                        else:
                            self.is_init_circle = True
                            self.save()
                            self.exit()
                else:
                    self.delay_timer += self.dt
                    print(f"DELAY {self.delay_timer}")
            else:
                if not self.is_init_circle:
                    self.t += self.dt
                    tp = Timepoint(self.t, self.pointer[0], self.pointer[1], curr_pt)
                    self.trajectory.add_timepoint(tp)

    def save(self):
        print("saving")
        rospack = rospkg.RosPack()
        timestamp = datetime.now().strftime("%Y-%m-%d_%H-%M-%S-%f")
        target_dir = os.path.join(rospack.get_path('fitts_task'), "trials", self.trial_dirname)
        if not os.path.exists(target_dir):
            os.makedirs(target_dir, exist_ok=True)

        traj_pkl_path = os.path.join(target_dir, f"{timestamp}.pkl")
        with open(traj_pkl_path, 'wb') as f:
            pickle.dump(self.trial, f)

        # TODO: Save the important parameters of the task like timestep and such
        # task_pkl_path = os.path.join(target_dir, f"{timestamp}_task_obj.pkl")
        # with open(task_pkl_path, "wb") as f:
        #     pickle.dump(self, f)

        self.state = TaskState.EXIT

    def exit(self):
        pygame.quit()
        sys.exit(0)

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
        self.screen = pygame.display.set_mode((0, 0), pygame.FULLSCREEN)
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
        self.pub = rospy.Publisher("fitts/cspacepts", PoseArray, queue_size=1)

    def polaris_callback(self, pose_array):
        pos = pose_array.poses[0].position
        position = np.array([pos.x, pos.y, pos.z, 1])
        q = pose_array.poses[0].orientation
        orientation = np.quaternion(q.w, q.x, q.y, q.z)
        if self.collect_data:
            if not np.any(np.isnan(position)):
                self.pts.append(position)
                self.orientations.append(orientation)
                # self.delay_timer = time.perf_counter()



def main():
    # node = FittsTaskNode(screen_size=(0.6858, 0.3556))  # Big monitor (27")
    # node = FittsTaskNode(screen_size=(0.505, 0.2805))  # Small monitor (21.5")
    node = FittsTaskNode(screen_size=(0.37, 0.23))  # laptop
    while node.is_pygame_running():
        node.step()


if __name__ == "__main__":
    main()
