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

from pycontrol.gripper import Robotiq85Gripper
from pycontrol.sensor import FT300Sensor
from pycontrol.robot import UR5eRobot



if __name__ == "__main__":
    rospy.init_node("combined_control")
    robot = UR5eRobot()
    gripper = Robotiq85Gripper()
    sensor = FT300Sensor()

    if gripper.get_stat().position < 0.075:
        success = gripper.open()
        if success:
            rospy.loginfo('Successfully opened')
            time.sleep(2)
        else:
            raise Exception("Cannot open gripper")


    pose_list = []
    pose_list.append([0.369, -0.123, 0.498, 2.241, -2.191, -0.014])
    pose_list.append([0.709, 0.247, 0.486, 2.731, -1.604, 0.004])
    pose_list.append([0.756, 0.257, 0.393, 2.686, -1.603, 0.018])

    robot.execute_cartesian_trajectory(pose_list)

    
    if gripper.get_stat().position > 0.075:
        success = gripper.close()
        if success:
            rospy.loginfo('Successfully closed')
            time.sleep(2)
        else:
            rospy.loginfo('Close gripper failed')
            raise Exception("Cannot open gripper")

    handover_list = []
    handover_list.append([0.620, 0.215, 0.515, 2.693, -1.639, 0.041])
    handover_list.append([0.1, -0.242, 0.688, 1.382, -2.865, -0.02])
    handover_list.append([0.137, -0.460, 0.379, 0.049, 2.97, -1.037])


    robot.execute_cartesian_trajectory(handover_list)

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

    robot.go_home()
