#!/home/tucker/anaconda3/bin/python3
import sys
import os

import dill
import numpy as np
import pygame
import quaternion
import rospy
import rosbag

sys.path.append('/home/tucker/thesis/ros_workspace/src/fitts_task/scripts')

from geometry_msgs.msg import PoseArray
from util import is_pygame_running, render_lines_of_text


class MocapTracker:
    def __init__(self, bag_path):
        # Set up ROS
        rospy.init_node('read_polaris', anonymous=True)
        rospy.Subscriber("polaris_sensor/targets", PoseArray, self.polaris_callback, queue_size=1)

        self.bag = rosbag.Bag(bag_path, "w")

        # Data tracking and control flow
        self.collect_data = False
        self.marker_visible = False

        # Calibration
        self.is_calibrate = True
        self.cb_pt = np.nan  # Calibration point
        self.cb_q = np.nan  # Calibration quaternion
        self.T = None  # Computed transformation matrix from calibration

        # Default framerate, doesn't matter during calibration
        self.framerate = 60

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.bag.close()
        return True

    def get_mocap_pt(self):
        if not np.any(np.isnan(self.cb_pt)):
            pt = np.matmul(self.T, self.cb_pt)[1:3]
            pt = pt[::-1]
            return pt
        else:
            return None

    def calibrate(self, time):
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
        wait_steps = self.framerate * time
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
        self._make_transformation_matrix(calib_pts, calib_qs)

        return True

    def _make_transformation_matrix(self, calib_pts, calib_qs):
        # Construct the transformation matrix
        ws_origin = np.mean(np.array(calib_pts), axis=0)
        ws_q = np.quaternion(np.mean(np.array(calib_qs)))

        # print(ws_origin, ws_q)
        # Transformation matrix from camera frame workspace frame
        rot_mat = quaternion.as_rotation_matrix(ws_q.inverse())

        # Create the full transformation matrix
        T = np.zeros((4, 4))
        T[:3, :3] = rot_mat
        T[:3, 3] = np.matmul(rot_mat, -ws_origin[:3])
        T[3, 3] = 1
        self.T = T

    def polaris_callback(self, pose_array):
        self.bag.write("polaris_sensor/targets", pose_array)

        pos = pose_array.poses[0].position
        position = np.array([pos.x, pos.y, pos.z, 1])
        q = pose_array.poses[0].orientation
        orientation = np.quaternion(q.w, q.x, q.y, q.z)
        self.cb_pt = position
        self.cb_q = orientation

    def save(self, fn):
        # indepentently save the T matrix
        # Avoids conflicts with rosbag when loading this object from pickle
        np.save(os.path.join(os.path.dirname(fn), "T.npy"), self.T)

        with open(fn, "wb") as f:
            dill.dump(self, f)


