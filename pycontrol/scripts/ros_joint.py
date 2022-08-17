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

if __name__ == "__main__":
    rospy.init_node("combined_control")
    robot = UR5eRobot()

    pose = []
    pose.append([0, -1.57, -2.616, -0.523, 1.57, 1.57])
    robot.execute_joint_trajectory(pose)

    state = robot.get_joint_pose()
    print(state)