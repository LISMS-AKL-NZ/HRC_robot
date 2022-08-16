#!/usr/bin/env python
#!\file
#
# \author  Wanqing Xia wxia612@aucklanduni.ac.nz
# \date    2022-07-22
#
#
# ---------------------------------------------------------------------

import sys
import time
import rospy
import numpy as np

from pycontrol.robot import UR5eRobot
from pycontrol.camera import AzureKinectCamera

if __name__ == "__main__":
    rospy.init_node("combined_control")
    robot = UR5eRobot()
    camera = AzureKinectCamera()

    pose_list =[]
    pose_list.append([-0.132, -0.803, 0.478, 0.0, -3.141, 0.0])
    robot.execute_cartesian_trajectory(pose_list)

    # TODO: add machine vision code
    while True:
        current_pose = robot.get_actual_pose()
        detection = camera.get_detect()
        follow_list = [] # go to picking position
        if detection.unpacked_rot != -100:
            tx = detection.unpacked_tx / 1000
            ty = detection.unpacked_ty / 10000
            follow_list.append([current_pose.position.x + tx, current_pose.position.y + ty, 0.478, 0.0, -3.141, 0.0])
            robot.execute_cartesian_trajectory(follow_list)