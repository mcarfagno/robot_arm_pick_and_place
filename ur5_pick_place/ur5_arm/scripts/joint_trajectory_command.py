#!/usr/bin/env python

import rospy
from trajectory_msgs.msg import JointTrajectory,JointTrajectoryPoint
import math

def send_joints_command():

    pub = rospy.Publisher('/arm_controller/command', JointTrajectory, queue_size=10)
    rospy.init_node('joint_trajectory_cmd_node', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():

        cmd_msg = JointTrajectory()
        cmd_msg.header.stamp = rospy.Time.now()
        cmd_msg.joint_names = ["shoulder_pan_joint","shoulder_lift_joint","elbow_joint","wrist_1_joint","wrist_2_joint","wrist_3_joint"]

        point_msg = JointTrajectoryPoint()
        point_msg.positions = [math.radians(90),math.radians(-90),math.radians(90),math.radians(45),math.radians(90),0]
        point_msg.velocities = [0.1,0.1,0.1,0.1,0.1,0.1]
        point_msg.accelerations = [0.1,0.1,0.1,0.1,0.1,0.1]
        point_msg.effort = [0.1,0.1,0.1,0.1,0.1,0.1]
        point_msg.time_from_start = rospy.Duration(1,0)

        cmd_msg.points.append(point_msg)

        pub.publish(cmd_msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        send_joints_command()
    except rospy.ROSInterruptException:
        pass
