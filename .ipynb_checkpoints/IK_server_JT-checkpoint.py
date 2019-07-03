#!/usr/bin/env python

import rospy
import tf
from kuka_arm.srv import *
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from geometry_msgs.msg import Pose
from sympy import *
from mpmath import radians

def handle_calculate_IK(req):
    rospy.loginfo("Received {} poses from the plan".format(len(req.poses)))
    if len(req.poses)<1:
        return -1
    else:
        #Initialize response
        joint_trajectory_list =[]
        for x in xrange(0, len(req.poses)):
            '''
            IK Steps:
            -
            -
            -
            '''
            joint_trajectory_point = JointTrajectoryPoint()

            #Define  DH param symbols
            d1, d2, d3, d4, d5, d6, d7 = symbols('d1:8')
            a0, a1, a2, a3, a4, a5, a6 = symbols('a0:7')
            alpha0,alpha1,alpha2,alpha3,alpha4,alpha5,alpha6 = symbols('alpha0:7')

            #Define Joint Variables symbols
            q1, q2, q3, q4, q5, q6, q7 =symbols('q1:8')

            #Modified DH Parameters Table
            DH_table = {alpha0:     0, a0:      0, d1:  0.75, q1:      q1,
                        alpha1: -pi/2, a1:   0.35, d2:     0, q2: q2-pi/2,
                        alpha2:     0, a2:   1.25, d3:     0, q3:      q3,
                        alpha3: -pi/2, a3: -0.054, d4:   1.5, q4:      q4,
                        alpha4:  pi/2, a4:      0, d5:     0, q5:      q5,
                        alpha5: -pi/2, a5:      0, d6:     0, q6:      q6,
                        alpha6:     0, a6:      0, d7: 0.303, q7:       0}

            #Define Homogeneous Transform Matrix from DH parameters
            def homogeneous_transform(q, d, a, alpha):

                T = Matrix([[cos(q)           ,           -sin(q),           0,             a],
                            [sin(q)*cos(alpha), cos(q)*cos(alpha), -sin(alpha), -sin(alpha)*d],
                            [sin(q)*sin(alpha), cos(q)*sin(alpha),  cos(alpha),  cos(alpha)*d],
                            [                0,                 0,           0,             1]])
                return T

            #Create individual transformation matrices
            T0_1  = homogeneous_transform(q1, d1, a0, alpha0).subs(DH_table)
            T1_2  = homogeneous_transform(q2, d2, a1, alpha1).subs(DH_table)
            T2_3  = homogeneous_transform(q3, d3, a2, alpha2).subs(DH_table)
            T3_4  = homogeneous_transform(q4, d4, a3, alpha3).subs(DH_table)
            T4_5  = homogeneous_transform(q5, d5, a4, alpha4).subs(DH_table)
            T5_6  = homogeneous_transform(q6, d6, a5, alpha5).subs(DH_table)
            T6_EE = homogeneous_transform(q7, d7, a6, alpha6).subs(DH_table)

            #Compute FK
            T0_EE = T0_1*T1_2*T2_3*T3_4*T4_5*T5_6*T6_EE

            #Extract EE position and orientation from request
            px = req.poses[x].position.x
            py = req.poses[x].position.y
            pz = req.poses[x].position.z

            (roll, pitch, yaw) =tf.transformations.euler_from_quaternion([req.poses[x].orientation.x,
                                                                          req.poses[x].orientation.y,
                                                                          req.poses[x].orientation.z,
                                                                          req.poses[x].orientation.w,])

            #Compute EE rotation Matrix using RPY Euler angles
            r,p,y = symbols('r,p,y')

            ROT_x = Matrix([[ 1,      0,       0],
                            [ 0, cos(r), -sin(r)],
                            [ 0, sin(r),  cos(r)]])

            ROT_y = Matrix([[  cos(p), 0, sin(p)],
                            [       0, 1,      0],
                            [ -sin(p), 0, cos(p)]])

            ROT_z = Matrix([[ cos(y), -sin(y), 0],
                            [ sin(y),  cos(y), 0],
                            [      0,       0, 1]])

            ROT_EE = ROT_z * ROT_y * ROT_x

            #Allign DH with URDF file
            rot_err = ROT_z.subs(y,radians(180)) * ROT_y.subs(p,radians(-90))

            #End-Effector orientation Matrix in DH frame
            ROT_EE = ROT_EE * rot_err
            rot_ee = ROT_EE.subs({'r':roll ,'p':pitch ,'y':yaw})

            #End-Effector pose vector
            pos_ee = Matrix([[px],
                             [py],
                             [pz]])

            #TODO: JT Algorithm

            #Populate response
            joint_trajectory_point.positions = [theta1,theta2,theta3,theta4,theta5,theta6]
            joint_trajectory_list.append(joint_trajectory_point)

    return CalculateIKResponse(joint_trajectory_list)

def IK_server():
    rospy.init_node("IK_server_node")
    server=rospy.Service("calculate_ik",CalculateIK,handle_calculate_IK)
    print "Ready to receive an IK request"
    rospy.spin()

if __name__ == '__main__':
    IK_server()