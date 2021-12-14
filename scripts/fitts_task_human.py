#!/home/tucker/anaconda3/bin/python3
import sys

import numpy as np
import pygame
import quaternion
import rospy

sys.path.append('/home/tucker/thesis/ros_workspace/src/fitts_task/scripts')

from geometry_msgs.msg import PoseArray
from util import is_pygame_running, render_lines_of_text
from task import FittsTask


class FittsMocap:
    def __init__(self,  n_trials=20, frame_rate=60, task_kwargs=None):
        rospy.init_node('read_polaris', anonymous=True)
        rospy.Subscriber("polaris_sensor/targets", PoseArray, self.polaris_callback, queue_size=1)
        self.collect_data = False
        self.marker_visible = False

        self.is_calibrate = True
        self.n_trials = n_trials

        self.framerate = 60
        self.calib_secs = 5
        self.delay_secs = 1

        self.cb_pt = np.nan
        self.cb_q = np.nan

        self.T = np.zeros((4, 4))

    def run(self):
        self.calibrate()

        clock = pygame.time.Clock()

        self.task = FittsTask(workspace_lims=((0, 0.37), (0, 0.23)),
                              target_size_lims=(0.01, 0.02),
                              home_pos=(0.01, 0.12),
                              home_size=0.01,
                              steps_to_wait=int(self.delay_secs * self.framerate),
                              render=True,
                              render_kwargs=dict(display_size=(2560, 1440), fullscreen=True))
        n = 0
        while n < self.n_trials:
            p_pointer = self.get_mocap_pt()
            print(p_pointer)
            success, target_pos, target_size = self.task.step(p_pointer)
            if success:
                n += 1
            clock.tick(self.framerate)

    def get_mocap_pt(self):
        if not np.any(np.isnan(self.cb_pt)):
            return np.matmul(self.T, self.cb_pt)[1:3]
        else:
            return None

    def calibrate(self):
        """
        With the marker placed at the origin of the workspace (world) coordinate frame, we collect
        some data to create the camera to world matrix. We invert that and apply it to 3D points
        collected from the camera in order to map those points to xy plane in camera space. Then, given
        the known size of the workspace and the known size of the screen, we directly map those points
        in the xy plane to the screen's pixel coordinates
        """
        pygame.init()
        screen = pygame.display.set_mode((0, 0), pygame.FULLSCREEN)
        pygame.display.set_caption('Fitts task')
        pygame.font.init()
        render_lines_of_text(screen, ["Running calibration"])

        pg_clock = pygame.time.Clock()

        calib_pts = []
        calib_qs = []

        dt = 1 / self.framerate
        wait_steps = self.framerate * self.calib_secs
        i = 0

        while i < wait_steps:
            screen.fill((0, 0, 0))
            if np.any(np.isnan(self.cb_pt)) or np.any(np.isnan(self.cb_q)):
                render_lines_of_text(screen, ["Cannot see marker"])
            else:
                # print(i*dt, self.cb_q, self.cb_pt)
                render_lines_of_text(screen, [f"Calibrating for 5s. t={i * dt:.3f}"])
                calib_pts.append(self.cb_pt)
                calib_qs.append(self.cb_q)
                i += 1

            pygame.display.flip()
            pygame.display.update()
            pg_clock.tick(self.framerate)

            if not is_pygame_running():
                return False

        pygame.quit()
        self.make_transformation_matrix(calib_pts, calib_qs)

        return True

    def make_transformation_matrix(self, calib_pts, calib_qs):
        # Construct the transformation matrix
        ws_origin = np.mean(np.array(calib_pts), axis=0)
        ws_q = np.quaternion(np.mean(np.array(calib_qs)))

        print(ws_origin, ws_q)
        # Transformation matrix from camera frame workspace frame
        rot_mat = quaternion.as_rotation_matrix(ws_q.inverse())
        self.T[:3, :3] = rot_mat
        self.T[:3, 3] = np.matmul(rot_mat, -ws_origin[:3])
        self.T[3, 3] = 1

    def polaris_callback(self, pose_array):
        pos = pose_array.poses[0].position
        position = np.array([pos.x, pos.y, pos.z, 1])
        q = pose_array.poses[0].orientation
        orientation = np.quaternion(q.w, q.x, q.y, q.z)
        self.cb_pt = position
        self.cb_q = orientation


if __name__ == "__main__":
    task = FittsMocap()
    task.run()
