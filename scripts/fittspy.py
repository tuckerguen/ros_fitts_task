#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import PointCloud
from geometry_msgs.msg import PoseArray
import matplotlib.pyplot as plt
from itertools import product, combinations
import numpy as np

fig = plt.figure()
ax = plt.axes(projection='3d')
xpts = []
ypts = []
zpts = []


def quat_to_rot_mat(q):
    return np.array([
        [2*(q.x**2+q.y**2)-1, 2*(q.y*q.z-q.x*q.w), 2*(q.y*q.w+q.x*q.z)],
        [2*(q.y*q.z+q.x*q.w), 2*(q.x**2+q.z**2)-1, 2*(q.z*q.w-q.x*q.y)],
        [2*(q.y*q.w-q.x*q.z), 2*(q.z*q.w+q.x*q.y), 2*(q.x**2+q.w**2)-1]
    ])

def polaris_targets_callback(pose_array):
    print(len(pose_array.poses))
    x = pose_array.poses[0].position.x
    y = pose_array.poses[0].position.y
    z = pose_array.poses[0].position.z
    print(f"{x:.4f}", f"{y:4f}", f"{z:.4f}")
    xpts.append(x)
    ypts.append(y)
    zpts.append(z)
    # ax.plot3D(xpts, ypts, zpts, 'blue')
    pt = np.array([z, y, z])
    length = 0.01
    translated = pt + np.array([length, length, length])
    rotated = translated * quat_to_rot_mat(pose_array.poses[0].orientation)
    ax.quiver(x, y, z, rotated[0], rotated[1], rotated[2])
    # ax.plot3D(x, y, z, 'blue')
    fig.canvas.draw()
    fig.canvas.flush_events()



def read_polaris():
    print("Created sub")
    rospy.init_node('read_polaris', anonymous=True)
    rospy.Subscriber("polaris_sensor/targets", PoseArray, polaris_targets_callback, queue_size=1)

    plt.show()
    rospy.spin()

if __name__ == '__main__':
    try:
        read_polaris()
    except rospy.ROSInterruptException:
        print("Exception")
