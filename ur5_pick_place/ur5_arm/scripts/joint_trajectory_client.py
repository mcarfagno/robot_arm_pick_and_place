#!/usr/bin/env python

import rospy
import actionlib

from control_msgs.msg import (
    FollowJointTrajectoryAction,
    FollowJointTrajectoryGoal,
)
from trajectory_msgs.msg import (
    JointTrajectoryPoint,
)

from copy import copy

class Trajectory(object):
   def __init__(self, ns):
       self._client = actionlib.SimpleActionClient(
           ns + "/" + "follow_joint_trajectory",
           FollowJointTrajectoryAction,
       )
       self._goal = FollowJointTrajectoryGoal()
       server_up = self._client.wait_for_server(timeout=rospy.Duration(10.0))
       if not server_up:
           rospy.logerr("Timed out waiting for Joint Trajectory"
                         " Action Server to connect. Start the action server"
                         " before running example.")
           rospy.signal_shutdown("Timed out waiting for Action Server")
           sys.exit(1)

    def add_point(self, positions, time):
       point = JointTrajectoryPoint()
        point.positions = copy(positions)
        point.time_from_start = rospy.Duration(time)
        self._goal.trajectory.points.append(point)

    def start(self):
        self._goal.trajectory.header.stamp = rospy.Time.now()
        self._client.send_goal(self._goal,feedback_cb=self._feedback)

    def stop(self):
        self._client.cancel_goal()

    def wait(self, timeout=15.0):
        self._client.wait_for_result(timeout=rospy.Duration(timeout))

    def result(self):
        return self._client.get_result()

    def _feedback(self,data):
        print(data.actual.time_from_start)

def main():

    traj = Trajectory("arm_controller")
    rospy.on_shutdown(traj.stop)

    # Command Current Joint Positions first
    limb_interface = baxter_interface.limb.Limb(limb)
    current_angles = [limb_interface.joint_angle(joint) for joint in limb_interface.joint_names()]
    traj.add_point(current_angles, 0.0)

    p1 = positions[limb]
    traj.add_point(p1, 7.0)
    traj.add_point([x * 0.75 for x in p1], 9.0)
    traj.add_point([x * 1.25 for x in p1], 12.0)
    traj.start()
    traj.wait(15.0)
    print("Exiting - Joint Trajectory Action Test Complete")

if __name__ == "__main__":
    smain()
