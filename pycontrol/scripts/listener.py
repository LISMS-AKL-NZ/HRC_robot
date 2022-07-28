#!/usr/bin/env python
import imp
import rospy
from control_msgs.msg import JointTrajectoryControllerState

import geometry_msgs.msg as geometry_msgs
from cartesian_control_msgs.msg import (
    FollowCartesianTrajectoryAction,
    FollowCartesianTrajectoryGoal,
    CartesianTrajectoryPoint,
    FollowCartesianTrajectoryActionFeedback,
)

def callback(data):
    rospy.loginfo(data.feedback.actual.pose)
    
def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('listener', anonymous=True)

    rospy.Subscriber("/forward_cartesian_traj_controller/follow_cartesian_trajectory/feedback", FollowCartesianTrajectoryActionFeedback, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()