#!/usr/bin/env python
#!\file
#
# \author  Wanqing Xia wxia612@aucklanduni.ac.nz
# \date    2022-07-28
#
#
# ---------------------------------------------------------------------

import sys
from turtle import pos
import rospy
import time
import numpy as np

import sys
import time
import rospy
import actionlib

from controller_manager_msgs.srv import SwitchControllerRequest, SwitchController
from controller_manager_msgs.srv import LoadControllerRequest, LoadController
import geometry_msgs.msg as geometry_msgs
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal, FollowJointTrajectoryFeedback
from trajectory_msgs.msg import JointTrajectoryPoint
from cartesian_control_msgs.msg import (
    FollowCartesianTrajectoryAction,
    FollowCartesianTrajectoryGoal,
    CartesianTrajectoryPoint,
    FollowCartesianTrajectoryActionFeedback,
)

from scipy.spatial.transform import Rotation
from ur_ikfast import ur_kinematics

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
JOINT_TRAJECTORY_CONTROLLERS = [
    "scaled_pos_joint_traj_controller",
    "scaled_vel_joint_traj_controller",
    "pos_joint_traj_controller",
    "vel_joint_traj_controller",
    "forward_joint_traj_controller",
]

# All of those controllers can be used to execute Cartesian trajectories.
# The scaled versions should be preferred over the non-scaled versions.
CARTESIAN_TRAJECTORY_CONTROLLERS = [
    "pose_based_cartesian_traj_controller",
    "joint_based_cartesian_traj_controller",
    "forward_cartesian_traj_controller",
]

# We'll have to make sure that none of these controllers are running, as they will
# be conflicting with the joint trajectory controllers
CONFLICTING_CONTROLLERS = ["joint_group_vel_controller", "twist_controller"]


class UR5eRobot:
    """Small trajectory client to test a joint trajectory"""

    def __init__(self):

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

        self.cartesian_trajectory_controller = CARTESIAN_TRAJECTORY_CONTROLLERS[2]
        self.joint_trajectory_controller = JOINT_TRAJECTORY_CONTROLLERS[0]
        # switch to cartesian controller and close al other controllers
        self._switch_controller(self.cartesian_trajectory_controller)
        self._current_controller = self.cartesian_trajectory_controller

        rospy.Subscriber("/forward_cartesian_traj_controller/follow_cartesian_trajectory/feedback", FollowCartesianTrajectoryActionFeedback, self._update_robot_pose, queue_size=10)
        rospy.Subscriber("/scaled_pos_joint_traj_controller/state", FollowJointTrajectoryFeedback, self._update_joint_pose, queue_size=10)

        self._actual_pose = geometry_msgs.Pose()
        self._desired_pose = geometry_msgs.Pose()
        self._position_error = [0, 0, 0]
        self._joint_pose = JointTrajectoryPoint().positions
        self.ur5e_arm = ur_kinematics.URKinematics('ur5e')

        self.home_pos = [[0.297, -0.132, 0.200, 2.231, -2.216, 0.0]]
        # self.go_home()
        
    def go_home(self):
        self.execute_cartesian_trajectory(self.home_pos)

    def _update_robot_pose(self,data):
        self._actual_pose = data.feedback.actual.pose
        self._desired_pose = data.feedback.desired.pose
        self._position_error[0] = self._actual_pose.position.x - self._desired_pose.position.x
        self._position_error[1] = self._actual_pose.position.y - self._desired_pose.position.y
        self._position_error[2] = self._actual_pose.position.z - self._desired_pose.position.z

    def _update_joint_pose(self, data):
        self._joint_pose = data.actual.positions

    def get_actual_pose(self):
        rx, ry, rz = self._quat2vec(self._actual_pose.orientation.x, self._actual_pose.orientation.y, self._actual_pose.orientation.z, self._actual_pose.orientation.w)
        pose= [self._actual_pose.position.x, self._actual_pose.position.y, self._actual_pose.position.z, rx, ry, rz]
        return pose

    def get_desired_pose(self):
        rx, ry, rz = self._quat2vec(self._desired_pose.orientation.x, self._desired_pose.orientation.y, self._desired_pose.orientation.z, self._desired_pose.orientation.w)
        pose = [self._desired_pose.position.x, self._desired_pose.position.y, self._desired_pose.position.z, rx, ry, rz]
        return pose
    
    def get_joint_pose(self):
        if self._current_controller != self.joint_trajectory_controller:
            self._switch_controller(self.joint_trajectory_controller)
        time.sleep(0.1)
        return self._joint_pose

    def get_pos_error(self):
        return self._position_error


    def execute_joint_trajectory(self, pose_list):
        """Creates a trajectory and sends it using the selected action server"""
        if self._current_controller != self.joint_trajectory_controller:
            self._switch_controller(self.joint_trajectory_controller)
        
        if pose_list:
            for i, pose in enumerate(pose_list):
                if len(pose) == 6:

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

                    point = JointTrajectoryPoint()
                    point.positions = pose
                    point.time_from_start = rospy.Duration(1)
                    goal.trajectory.points.append(point)

                    trajectory_client.send_goal(goal)
                    trajectory_client.wait_for_result()

                    result = trajectory_client.get_result()
                    rospy.loginfo("Rotating joints to position {} finished in state {}".format(pose, result.error_code))
                else:
                    rospy.logerr("Each action should have 6 elements, this one has {}".format(len(pose)))

        else:
            rospy.logerr("Action list is empty")


    def execute_cartesian_trajectory(self, pose_list):
        """Creates a Cartesian trajectory and sends it using the selected action server"""
        if self._current_controller != self.cartesian_trajectory_controller:
            self._switch_controller(self.cartesian_trajectory_controller)

        if pose_list:
            for i, pose in enumerate(pose_list):
                if len(pose) == 6:

                    travel_dist = np.linalg.norm([self._actual_pose.position.x - pose[0],
                                                    self._actual_pose.position.y - pose[1],
                                                    self._actual_pose.position.z - pose[2]])
                    if travel_dist < 0.1:
                        travel_dist = 0.1
                        # if np.abs(self._actual_pose.position.tx - pose[3]) > 0.785 or \
                        #         np.abs(self._actual_pose.position.ty - pose[4]) > 0.785 or \
                        #         np.abs(self._actual_pose.position.tz - pose[5]) > 0.785:
                        #     travel_dist = 0.2
                    # make sure the correct controller is loaded and activated
                    goal = FollowCartesianTrajectoryGoal()
                    trajectory_client = actionlib.SimpleActionClient(
                        "{}/follow_cartesian_trajectory".format(self.cartesian_trajectory_controller),
                        FollowCartesianTrajectoryAction,
                    )

                    # Wait for action server to be ready
                    timeout = rospy.Duration(5)
                    if not trajectory_client.wait_for_server(timeout):
                        rospy.logerr("Could not reach controller action server.")
                        sys.exit(-1)

                    # The following list are arbitrary positions
                    # Change to your own needs if desired

                    point = CartesianTrajectoryPoint()
                    point.pose = self._convert_pose(pose)
                    point.time_from_start = rospy.Duration(travel_dist/0.25)
                    goal.trajectory.points.append(point)

                    # rospy.loginfo("Executing trajectory using the {}".format(self.cartesian_trajectory_controller))
                    trajectory_client.send_goal(goal)
                    trajectory_client.wait_for_result()

                    result = trajectory_client.get_result()

                    rospy.loginfo("Moving to point {} finished in state {}".format(pose, result.error_code))
                else:
                    rospy.logerr("Each action should have 6 elements, this one has {}".format(len(pose)))

        else:
            rospy.logerr("Action list is empty")

    def _switch_controller(self, target_controller):
        """Activates the desired controller and stops all others from the predefined list above"""
        other_controllers = (
            JOINT_TRAJECTORY_CONTROLLERS
            + CARTESIAN_TRAJECTORY_CONTROLLERS
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
        self._current_controller = target_controller

    def _convert_pose(self, pose):
        return geometry_msgs.Pose(geometry_msgs.Vector3(pose[0], pose[1], pose[2]), geometry_msgs.Quaternion(*self._vec2quat(pose[3], pose[4], pose[5])))

    def _vec2quat(self, x, y, z):
        # Create a rotation object from rotation vector in radians
        rot = Rotation.from_rotvec([x, y, z])
        # Convert to quaternions and print
        rq = rot.as_quat()
        return rq[0], rq[1], rq[2], rq[3]

    def _quat2vec(self, x, y, z, w):
        # Create a rotation object from rotation vector in radians
        rot = Rotation.from_quat([x, y, z, w])
        # Convert to quaternions and print
        rr = rot.as_rotvec()
        return rr[0], rr[1], rr[2]
    
    def joint_to_cart(self, pose):
        # convert joint pose(in radians) to cartisian point
        pose_quat = self.ur5e_arm.forward(pose)
        vec = self._quat2vec(pose_quat[3],pose_quat[4], pose_quat[5], pose_quat[6])
        return [pose_quat[0], pose_quat[1], pose_quat[2], vec[1], -vec[0], vec[2]]
