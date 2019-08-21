#!/usr/bin/env python

import rospy

from trajectory_msgs.msg import (
    JointTrajectory,
    JointTrajectoryPoint,
)

import math

class Gripper(object):
    '''
    Class to send joints commands
    to gripper in gazebo.
    '''
    def __init__(self,ns='gripper_controller',joint_names=['left_gripper_finger_joint','right_gripper_finger_joint']):
                
        self._ns = ns
        
        self._pub = rospy.Publisher(self._ns + '/command', JointTrajectory, queue_size=1)
        
        self._command = JointTrajectory()
        self._command.joint_names=joint_names
        self._command.points = [None]

        self._OPEN=-0.005
        self._CLOSE=0.025

    def _gripper_open(self):
        '''
        Sets JointTrajectory command to open the gripper
        '''
        self._command.points[0] = JointTrajectoryPoint()
        self._command.points[0].positions = [self._OPEN,self._OPEN]
        self._command.points[0].velocities = [0.03,0.03]
        self._command.points[0].effort = [0.75,0.75]
        self._command.points[0].time_from_start = rospy.Duration(1.0)


    def _gripper_close(self):
        '''
        Sets JointTrajectory command to close the gripper
        '''
        self._command.points[0] = JointTrajectoryPoint()
        self._command.points[0].positions = [self._CLOSE,self._CLOSE]
        self._command.points[0].velocities = [0.03,0.03]
        self._command.points[0].effort = [0.75,0.75]
        self._command.points[0].time_from_start = rospy.Duration(1.0)

    def send_gripper_command(self,gripper_cmd="open"):
        '''
        Send command to te gripper
        '''
        if gripper_cmd=="open":
            self._gripper_open()

        if gripper_cmd=="close":
            self._gripper_close()

        self._command.header.stamp=rospy.Time.now()
        self._pub.publish(self._command)


def main():
    print("Initializing node... ")
    rospy.init_node("gripper_command_node")
    print("Running. Ctrl-c to quit")

    grip = Gripper()

    cmd="close"

    while not rospy.is_shutdown():

        print("Commanding the gripper to: {}".format(cmd))
        grip.send_gripper_command(cmd)

        if cmd=="close":
            cmd="open"
        else:
            cmd="close"

        rospy.sleep(rospy.Duration(10))

if __name__ == '__main__':
    main()