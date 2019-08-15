#!/usr/bin/env python

import rospy
import actionlib
import math
import sys

from control_msgs.msg import (
    FollowJointTrajectoryAction,
    FollowJointTrajectoryGoal,
)

from trajectory_msgs.msg import (
    JointTrajectoryPoint,
)

from copy import copy

class Trajectory(object):
    def __init__(self, ns, joint_names):
        
        self._ns = ns
        self._joint_names = joint_names

        # create action client
        self._client = actionlib.SimpleActionClient(
           self._ns + "/" + "follow_joint_trajectory",
           FollowJointTrajectoryAction)

        # verify joint trajectory action server is available
        server_up = self._client.wait_for_server(timeout=rospy.Duration(10.0))
        if not server_up:
            rospy.logerr("Timed out waiting for Joint Trajectory"
                         " Action Server to connect. Start the action server"
                         " before running example.")
            rospy.signal_shutdown("Timed out waiting for Action Server")
            sys.exit(1)

        # create request
        self._goal = FollowJointTrajectoryGoal()

    def add_point(self, positions, time):
        '''
        Adds a waypoint to trajectory.
        '''
        point = JointTrajectoryPoint()
        point.positions = copy(positions)
        point.time_from_start = rospy.Duration(time)
        self._goal.trajectory.points.append(point)

    def start(self):
        '''
        Sends FollowJointTrajectoryAction request to action server.
        '''
        self._goal.trajectory.joint_names = self._joint_names
        self._goal.trajectory.header.stamp = rospy.Time.now()
        self._client.send_goal(self._goal,feedback_cb=self._feedback)

    def stop(self):
        '''
        Preempts trajectory execution.
        '''
        if (self._client.gh is not None and
            self._client.get_state() == actionlib.GoalStatus.ACTIVE):
            self._client.cancel_goal()

    def wait(self, timeout=10.0):
        '''
        Waits for and verifies trajectory execution result.
        '''
        finish = self._client.wait_for_result(timeout=rospy.Duration(timeout))
        result = (self._client.get_result().error_code == 0)

        if all([finish, result]):
            return True
        else:
            msg = ("Trajectory action failed or did not finish before "
                   "timeout/interrupt.")
            rospy.logwarn(msg)
            return False

    def result(self):
        '''
        Returns the trajectory execution result.
        '''
        return self._client.get_result()

    def _feedback(self,data):
        '''
        Feedback callback
        '''
        print(data)

def main():
    print("Initializing node... ")
    rospy.init_node("follow_joint_trajectory_node")
    print("Running. Ctrl-c to quit")

    joints_names_list=["shoulder_pan_joint",
                    "shoulder_lift_joint",
                    "elbow_joint",
                    "wrist_1_joint",
                    "wrist_2_joint",
                    "wrist_3_joint"]

    traj = Trajectory(ns="arm_controller",joint_names=joints_names_list)
    rospy.on_shutdown(traj.stop)

    # First Trajectory W.P.
    traj.add_point([math.radians(30),
                    math.radians(-120),
                    math.radians(120),
                    0,
                    math.radians(90),
                    0], 3.0)

    # Second Trajectory W.P.
    traj.add_point([math.radians(90),
                    math.radians(-120),
                    math.radians(120),
                    math.radians(90),
                    math.radians(90),
                    0], 6.0)
    traj.start()
    traj.wait(15.0)
    print("Exiting - Joint Trajectory Action Test Complete")

if __name__ == "__main__":
    main()
