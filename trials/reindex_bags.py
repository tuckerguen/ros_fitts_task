import os
import subprocess

for f in os.scandir("/home/tucker/thesis/ros_workspace/src/fitts_task/trials"):
    if os.path.isdir(f.path):
        print(f.path)
        for f1 in os.scandir(f.path):
            for f2 in os.scandir(f1.path):
                if f2.name == "rosbag.bag":
                    print(f"reindexing {f2.path}")
                    subprocess.run(["rosbag reindex " + f2.path], shell=True)
