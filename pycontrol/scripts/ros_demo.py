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
from pycontrol.gripper import Robotiq85Gripper

def open_gripper(gripper):
    success = gripper.open()
    if success:
        rospy.loginfo('Successfully opened')
        time.sleep(2)
    else:
        rospy.loginfo('Open gripper failed')
        raise Exception("Cannot open gripper")
    time.sleep(1)

def close_gripper(gripper):
    success = gripper.close()
    if success:
        rospy.loginfo('Successfully closed')
        time.sleep(1)
    else:
        rospy.loginfo('Close gripper failed')
        raise Exception("Cannot close gripper")
    time.sleep(1)

if __name__ == "__main__":
    rospy.init_node("combined_control")
    robot = UR5eRobot()
    gripper = Robotiq85Gripper()
    # camera = AzureKinectCamera()
    while True:
        pose_list =[]
        pose_list.append([-0.067, -0.132, 1.08, 1.911, -1.889, 3.604])
        robot.execute_cartesian_trajectory(pose_list)
        open_gripper(gripper)

        time.sleep(1)

        rot_list =[]
        rot_list.append([-0.067, -0.132, 1.08, 1.572, -3.822, 2.932])
        robot.execute_cartesian_trajectory(rot_list)
        close_gripper(gripper)

        time.sleep(1)