#!/usr/bin/env python

# -- BEGIN LICENSE BLOCK ----------------------------------------------
# Copyright 2021 FZI Forschungszentrum Informatik
# Created on behalf of Universal Robots A/S
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
# -- END LICENSE BLOCK ------------------------------------------------
#
# ---------------------------------------------------------------------
# !\file
#
# \author  Wanqing Xia wxia612@aucklanduni.ac.nz
# \date    2022-07-22
#
#
# ---------------------------------------------------------------------
import sys

import rospy
import actionlib
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectoryPoint
from controller_manager_msgs.srv import SwitchControllerRequest, SwitchController
from controller_manager_msgs.srv import LoadControllerRequest, LoadController
import geometry_msgs.msg as geometry_msgs
from cartesian_control_msgs.msg import (
    FollowCartesianTrajectoryAction,
    FollowCartesianTrajectoryGoal,
    CartesianTrajectoryPoint,
)

# Compatibility for python2 and python3
if sys.version_info[0] < 3:
    input = raw_input

# If your robot description is created with a tf_prefix, those would have to be adapted
JOINT_NAMES = [
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint",
]

# All of those controllers can be used to execute joint-based trajectories.
# The scaled versions should be preferred over the non-scaled versions.
JOINT_TRAJECTORY_CONTROLLER = ["scaled_pos_joint_traj_controller"]


# We'll have to make sure that none of these controllers are running, as they will
# be conflicting with the joint trajectory controllers
CONFLICTING_CONTROLLERS = ["joint_group_vel_controller", "twist_controller"]


class TrajectoryClient:
    """Small trajectory client to test a joint trajectory"""

    def __init__(self):
        rospy.init_node("ros_demo")

        timeout = rospy.Duration(5)
        self.switch_srv = rospy.ServiceProxy(
            "controller_manager/switch_controller", SwitchController
        )
        self.load_srv = rospy.ServiceProxy("controller_manager/load_controller", LoadController)
        try:
            self.switch_srv.wait_for_service(timeout.to_sec())
        except rospy.exceptions.ROSException as err:
            rospy.logerr("Could not reach controller switch service. Msg: {}".format(err))
            sys.exit(-1)

        self.joint_trajectory_controller = JOINT_TRAJECTORY_CONTROLLER[0]

    def switch_controller(self, target_controller):
        """Activates the desired controller and stops all others from the predefined list above"""
        other_controllers = (
            JOINT_TRAJECTORY_CONTROLLER
            + CONFLICTING_CONTROLLERS
        )

        other_controllers.remove(target_controller)

        srv = LoadControllerRequest()
        srv.name = target_controller
        self.load_srv(srv)

        srv = SwitchControllerRequest()
        srv.stop_controllers = other_controllers
        srv.start_controllers = [target_controller]
        srv.strictness = SwitchControllerRequest.BEST_EFFORT
        self.switch_srv(srv)

    def send_joint_trajectory(self, position):
        """Creates a trajectory and sends it using the selected action server"""

        # make sure the correct controller is loaded and activated
        self.switch_controller(self.joint_trajectory_controller)
        trajectory_client = actionlib.SimpleActionClient(
            "{}/follow_joint_trajectory".format(self.joint_trajectory_controller),
            FollowJointTrajectoryAction,
        )

        # Wait for action server to be ready
        timeout = rospy.Duration(5)
        if not trajectory_client.wait_for_server(timeout):
            rospy.logerr("Could not reach controller action server.")
            sys.exit(-1)

        # Create and fill trajectory goal
        goal = FollowJointTrajectoryGoal()
        goal.trajectory.joint_names = JOINT_NAMES

        # The following list are arbitrary positions
        # Change to your own needs if desired

        # duration_list = [3.0, 6.0, 9.0, 12.0, 15.0, 18.0, 21.0, 24.0, 27.0, 30.0, 33.0, 36.0, 39.0, 42.0]

        point = JointTrajectoryPoint()
        point.positions = position
        point.time_from_start = rospy.Duration(3.0)
        goal.trajectory.points.append(point)

        rospy.loginfo("Executing trajectory using the {}".format(self.joint_trajectory_controller))
        rospy.loginfo("Moving joints to position {}".format(position))

        trajectory_client.send_goal(goal)
        trajectory_client.wait_for_result()

        result = trajectory_client.get_result()
        rospy.loginfo("Trajectory execution finished in state {}".format(result.error_code))



if __name__ == "__main__":
    client = TrajectoryClient()
    cnt = 0

    position_list = [[-0.56, -1.28, -2.51, -0.95, 1.58, 0]]# lift
    position_list.append([-1.57, -1.28, -2.51, -0.95, 1.58, 0])# turn
    position_list.append([-1.57, -1.83, -1.31, -1.59, 1.58, 0])# Pick observe
    position_list.append([-1.57, -2.22, -1.31, -1.17, 1.58, 0])# Pick position
    position_list.append([-1.57, -1.83, -1.31, -1.59, 1.58, 0])# Pick observe
    position_list.append([-1.57, -1.0, -2.65, -1.07, 1.58, 0])# retract
    position_list.append([0, -1.0, -2.65, -1.07, 1.58, 0])# turn to worker
    position_list.append([0, -2.32, -1.43, -0.85, 1.58, 0])# deliver
    position_list.append([0, -1.84, -1.0, -1.94, 1.58, 0])# observe new
    position_list.append([0, -2.32, -1.43, -0.85, 1.58, 0])# pickup
    position_list.append([0, -1.0, -2.65, -1.07, 1.58, 0])# retract
    position_list.append([0.96, -1.0, -2.65, -1.07, 1.58, 0])# final deliver
    position_list.append([0, -1.0, -2.65, -1.07, 1.58, 0])# turn back
    position_list.append([0, -1.58, -2.62, -0.53, 1.58, 0])# Home position

    # The controller choice is obviously not required to move the robot. It is a part of this demo
    # script in order to show all available trajectory controllers.
    while True:
        terminal_input = input("Please enter 'Y' to execute next move or 'N' to quit:    ")
        if terminal_input == "y":
            print("Executing move ...")
            client.send_joint_trajectory(position_list[cnt])
            cnt += 1
        elif terminal_input == "n":
            print("Exiting ...")
            rospy.loginfo("Exiting as requested by user.")
            sys.exit(0)
            break
        elif terminal_input == "home":
            client.send_joint_trajectory([0, -1.58, -2.62, -0.53, 1.58, 0])
            rospy.loginfo("Returning to home position without collision detection, be careful.")
        else:       
            pass