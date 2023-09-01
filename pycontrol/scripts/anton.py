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
import signal

from pycontrol.gripper import Robotiq85Gripper
from pycontrol.robot import UR5eRobot
from pycontrol.camera import AzureKinectCamera

def sig_handler(signal, frame):
    print("Existing Program...")
    sys.exit(0)

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
    camera = AzureKinectCamera()


    signal.signal(signal.SIGINT, sig_handler)

    # send robot to home position
    robot.go_home()

    # opening gripper
    open_gripper(gripper)

    while True:
        # turn over to conveyor side
        pose_list = []
        pose_list.append([0.483, -0.189, 0.506, 2.226, -2.217, 0.0])
        robot.execute_cartesian_trajectory(pose_list)

        over_list =[]
        over_list.append([0.483, -0.189, 0.130, 2.226, -2.217, 0.0])
        robot.execute_cartesian_trajectory(over_list)
        
        # close gripper
        close_gripper(gripper)


        # turn over to conveyor side
        pose_list = []
        pose_list.append([0.483, -0.189, 0.506, 2.226, -2.217, 0.0])
        pose_list.append([0.478, -0.180, 0.710, 2.338, -2.368, 2.535])
        pose_list.append([0.355, 0.136, 0.812, 4.62, -0.788, 2.145])
        robot.execute_cartesian_trajectory(pose_list)
        robot.go_home()

        over_list =[]
        over_list.append([0.483, -0.189, 0.130, 2.226, -2.217, 0.0])
        robot.execute_cartesian_trajectory(over_list)

        # open gripper
        open_gripper(gripper)

        robot.go_home()