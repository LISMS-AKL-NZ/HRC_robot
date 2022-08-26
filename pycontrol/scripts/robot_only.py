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

    angle = 1.571
    try:
        while True:
            pose_list =[]
            pose_list.append([0.0, -angle, 0.0, -angle, angle, 0.0])
            robot.execute_joint_trajectory(pose_list)

            time.sleep(1)

            rot_list =[]
            rot_list.append([0.0, -angle, 0.0, -angle, 0.0, 0.0])
            robot.execute_joint_trajectory(rot_list)

            time.sleep(1)
    except KeyboardInterrupt:
        pass