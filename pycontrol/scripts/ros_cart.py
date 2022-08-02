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

from pycontrol.gripper import Robotiq85Gripper
from pycontrol.sensor import FT300Sensor
from pycontrol.robot import UR5eRobot
from pycontrol.conveyor import ConveyorBelt


if __name__ == "__main__":
    rospy.init_node("combined_control")
    robot = UR5eRobot()
    gripper = Robotiq85Gripper()
    sensor = FT300Sensor()
    conveyor = ConveyorBelt()

    if gripper.get_stat().position < 0.075:
        success = gripper.open()
        if success:
            rospy.loginfo('Successfully opened')
            time.sleep(2)
        else:
            rospy.loginfo('Open gripper failed')
            raise Exception("Cannot open gripper")


    pose_list = [] # move to over observing plate
    pose_list.append([0.297, -0.132, 0.272, 2.226, -2.217, 0.0])
    pose_list.append([-0.132, -0.297, 0.272, 0.0, -3.141, 0.0])

    robot.execute_cartesian_trajectory(pose_list)

    conveyor.go_home()

    while True:
        if conveyor.get_coveyor_stat().current_position < 1:
            break
        else:
            time.sleep(0.1)

    dumb_list =[]
    dumb_list.append([-0.132, -0.803, 0.478, 0.0, -3.141, 0.0])
    robot.execute_cartesian_trajectory(dumb_list)

    # TODO: add machine vision code

    picking_list = [] # go to picking position
    picking_list.append([-0.132, -0.803, 0.235, 0.0, -3.141, 0.0])
    picking_list.append([-0.132, -0.803, 0.235, 2.223, -2.219, 0.0])
    picking_list.append([-0.132, -0.803, 0.195, 2.223, -2.219, 0.0])

    robot.execute_cartesian_trajectory(picking_list)

    if gripper.get_stat().position > 0.05:
        success = gripper.close()
    if success:
        rospy.loginfo('Successfully closed')
        time.sleep(1)
    else:
        rospy.loginfo('Close gripper failed')
        raise Exception("Cannot close gripper")

    retract_list = []
    retract_list.append([-0.132, -0.803, 0.35, 0.0, -3.141, 0.0])
    retract_list.append([-0.132, -0.297, 0.272, 0.0, -3.141, 0.0])
    retract_list.append([0.297, -0.132, 0.272, 2.226, -2.217, 0.0])

    robot.execute_cartesian_trajectory(retract_list)

    conveyor.set_position(270)

    handover_list = []
    handover_list.append([0.586, -0.132, 0.233, 2.52, -2.51, 1.797])
    robot.execute_cartesian_trajectory(handover_list)
    time.sleep(1)

    while True:
        if np.sum(np.abs(robot.get_error())) < 0.01:
            if np.abs(conveyor.get_coveyor_stat().current_position - 270) < 2:
                break
            else:
                time.sleep(0.1)
        else:
            time.sleep(0.1)

    time.sleep(0.5)

    sensor_reading = sensor.get_reading()

    while True:
        new_reading = sensor.get_reading()

        if abs(new_reading.Fx - sensor_reading.Fx) > 5 or abs(new_reading.Fy - sensor_reading.Fy) > 5 or abs(new_reading.Fz - sensor_reading.Fz) > 5:
            success = gripper.open()
            if success:
                rospy.loginfo('Successfully opened')
                time.sleep(2)
            else:
                rospy.loginfo('Open gripper failed')
                raise Exception("Cannot open gripper")
            break
        else:
            pass

    observe_list = []
    observe_list.append([0.586, -0.132, 0.233, 2.227, -2.217, 0.0])
    observe_list.append([0.586, -0.132, 0.663, 2.227, -2.217, 0.0])
    robot.execute_cartesian_trajectory(observe_list)

    # TODO: add machine vision code

    pick_list = []
    pick_list.append([0.586, -0.132, 0.095, 2.227, -2.217, 0.0])
    robot.execute_cartesian_trajectory(pick_list)

    if gripper.get_stat().position > 0.05:
        success = gripper.close()
    if success:
        rospy.loginfo('Successfully closed')
        time.sleep(1)
    else:
        rospy.loginfo('Close gripper failed')
        raise Exception("Cannot close gripper")

    robot.go_home()

    conveyor.set_position(540)

    place_list= []
    place_list.append([0.297, -0.132, 0.272, 2.226, -2.217, 0.0])
    place_list.append([0.175, 0.273, 0.272, 3.14, -0.231, 0.0])
    robot.execute_cartesian_trajectory(place_list)

    if gripper.get_stat().position < 0.075:
        success = gripper.open()
    if success:
        rospy.loginfo('Successfully opened')
        time.sleep(2)
    else:
        rospy.loginfo('Open gripper failed')
        raise Exception("Cannot open gripper")

    robot.go_home()
    conveyor.set_position(250)