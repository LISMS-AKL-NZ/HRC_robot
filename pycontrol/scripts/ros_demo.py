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

    pose_list =[]
    pose_list.append([-0.132, -0.803, 0.478, 0.0, -3.141, 0.0])
    robot.execute_cartesian_trajectory(pose_list)

    # TODO: add machine vision code

    picking_list = [] # go to picking position
    picking_list.append([-0.132, -0.803, 0.235, 0.0, -3.141, 0.0])
    picking_list.append([-0.132, -0.803, 0.235, 2.223, -2.219, 0.0])
    picking_list.append([-0.132, -0.803, 0.195, 2.223, -2.219, 0.0])

    robot.execute_cartesian_trajectory(picking_list)


    retract_list = []
    retract_list.append([-0.132, -0.803, 0.35, 0.0, -3.141, 0.0])
    retract_list.append([-0.132, -0.297, 0.272, 0.0, -3.141, 0.0])

    robot.execute_cartesian_trajectory(retract_list)