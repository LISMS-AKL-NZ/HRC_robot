#!/usr/bin/env python
# license removed for brevity
from turtle import position
import rospy
from std_msgs.msg import Header
from control_msgs.msg import JointTrajectoryControllerState
from math import *

def get_goal():
    goal = JointTrajectoryControllerState()
    goal.header = Header()
    goal.joint_names = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
    goal.header.stamp = rospy.Time.now()
    goal.desired.positions = [0.50, -0.55, -2.6, -1.57, 1.58, 0.0]
    goal.desired.velocities = [0.1, 0, 0, 0, 0, 0]
    goal.desired.time_from_start = rospy.Duration(2.0)
    return goal


def talker():
    pub = rospy.Publisher('scaled_pos_joint_traj_controller/command', JointTrajectoryControllerState, queue_size=10)
    rospy.init_node('send_joints', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        goal = get_goal()
        rospy.loginfo(goal)
        pub.publish(goal)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass