#!/usr/bin/python3

import numpy as np
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt
import sys

left_lane_file      = '/home/binbin/disk/project-code/lane_quality_assessment/ICNet-master/LaneRecognitionTracking/data/2011_09_26_0056/Lanes/worldLeftCoordinates.txt'#sys.argv[1]
center_lane_file    = '/home/binbin/disk/project-code/lane_quality_assessment/ICNet-master/LaneRecognitionTracking/data/2011_09_26_0056/Lanes/worldCenterCoordinates.txt'#sys.argv[2]
right_lane_file     = '/home/binbin/disk/project-code/lane_quality_assessment/ICNet-master/LaneRecognitionTracking/data/2011_09_26_0056/Lanes/worldRightCoordinates.txt'#sys.argv[3]

left_fin    = open(left_lane_file)
center_fin  = open(center_lane_file)
right_fin   = open(right_lane_file)

left_pts = np.array([np.array(line.split()).astype(np.float) for line in left_fin if line.strip() != ''])
center_pts = np.array([np.array(line.split()).astype(np.float) for line in center_fin if line.strip() != ''])
right_pts = np.array([np.array(line.split()).astype(np.float) for line in right_fin if line.strip() != ''])

fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
ax.scatter(left_pts[:, 0], left_pts[:, 1], left_pts[:, 2], c='r')
ax.scatter(center_pts[:, 0], center_pts[:, 1], center_pts[:, 2], c='g')
ax.scatter(right_pts[:, 0], right_pts[:, 1], right_pts[:, 2], c='b')

ax.set_xlabel('Forward')
ax.set_ylabel('Leftward')
ax.set_zlabel('Upward')

left_fin.close()
center_fin.close()
right_fin.close()

plt.show()