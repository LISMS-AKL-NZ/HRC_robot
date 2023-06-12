#!/usr/bin/env python
#!\file
#
# \author  Wanqing Xia wxia612@aucklanduni.ac.nz
# \date    2023-06-01
#
#
# ---------------------------------------------------------------------

import sys
import time
import rospy
import numpy as np
import signal

from pycontrol.gripper import Robotiq85Gripper
from pycontrol.sensor import FT300Sensor
from pycontrol.robot import UR5eRobot

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
    sensor = FT300Sensor()

    signal.signal(signal.SIGINT, sig_handler)

    # send robot to home position
    robot.go_home()

    # opening gripper
    open_gripper(gripper)

    while True:
        # observe above table
        observe_list = []
        observe_list.append([0.320, -0.132, 0.751, 2.227, -2.217, 0.0])
        robot.execute_cartesian_trajectory(observe_list)

        input('Press any key to continue.\n')
        
        #pick up a tool
        pick_list = []
        pick_list.append([0.349, -0.446, 0.138, 0.0, -3.141, 0.0])
        robot.execute_cartesian_trajectory(pick_list)

        # close gripper
        close_gripper(gripper)

        handover_list = []
        handover_list.append([0.349, -0.446, 0.250, 0.0, -3.141, 0.0])
        handover_list.append([0.453, 0.355, 0.294, 2.218, -2.678, 1.779])
        robot.execute_cartesian_trajectory(handover_list)

        sensor_reading = sensor.get_reading()
        while True:
            new_reading = sensor.get_reading()
            if abs(new_reading.Fx - sensor_reading.Fx) > 9 or abs(new_reading.Fy - sensor_reading.Fy) > 5 or abs(new_reading.Fz - sensor_reading.Fz) > 8:
                print(new_reading.Fx - sensor_reading.Fx, new_reading.Fy - sensor_reading.Fy, new_reading.Fz - sensor_reading.Fz)
                # open gripper
                open_gripper(gripper)
                break
            else:
                pass
