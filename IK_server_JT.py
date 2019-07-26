#!/usr/bin/env python

import rospy
import tf
from kuka_arm.srv import *
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from geometry_msgs.msg import Pose
from sympy import *
import numpy as np

def handle_calculate_IK(req):
    rospy.loginfo("Received {} poses from the plan".format(len(req.poses)))
    if len(req.poses)<1:
        return -1
    else:

        #Define  DH param symbols
        d1, d2, d3, d4, d5, d6, d7 = symbols('d1:8')
        a0, a1, a2, a3, a4, a5, a6 = symbols('a0:7')
        alpha0,alpha1,alpha2,alpha3,alpha4,alpha5,alpha6 = symbols('alpha0:7')

        #Define Joint Variables symbols
        q1, q2, q3, q4, q5, q6, q7 =symbols('q1:8')

        #Modified UR5 DH Parameters Table
        DH_table = { alpha0: 0,      a0: 0,       d1: 0.089159, q1: q1, #Shoulder Pan Joint
                     alpha1: pi/2,   a1: 0,        d2: 0,       q2: q2, #Shoulder Lift Joint
                     alpha2: 0,      a2: -0.42500, d3: 0,       q3: q3, #Elbow Joint
                     alpha3: 0,      a3: -0.39225, d4: 0.10915, q4: q4, #Wirst 1 Joint
                     alpha4: pi/2,   a4: 0,        d5: 0.09465, q5: q5, #Wirst 2 Joint
                     alpha5: -pi/2,  a5: 0,        d6: 0.0823,  q6: q6, #Wirst 3 Joint
                     alpha6: 0,      a6: 0,        d7: 0.085,   q7: 0}  #Gripper Offset

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

        angle = symbols('angle')

        R_y = Matrix([[  cos(angle), 0, sin(angle)],
                      [           0, 1,          0],
                      [ -sin(angle), 0, cos(angle)]])

        R_z = Matrix([[ cos(angle), -sin(angle),  0],
                      [ sin(angle),  cos(angle),  0],
                      [          0,           0,  1]])

        #Gazebo to DH base link orientation offset
        R_BASE = R_z.subs('angle',pi)
        T_BASE = R_BASE.col_insert(3, Matrix([0, 0, 0]))
        T_BASE = T_BASE.row_insert(3, Matrix([[0, 0, 0, 1]]))

        #DH to Gazebo EE orientation offset
        R_CORR = R_z.subs('angle',pi/2)*R_y.subs('angle',-pi/2)
        T_CORR = R_CORR.col_insert(3, Matrix([0, 0, 0]))
        T_CORR = T_CORR.row_insert(3, Matrix([[0, 0, 0, 1]]))

        T0_EE_GAZEBO = T_BASE*T0_EE*T_CORR

        #Compute EE Pose Vector
        X = zeros(6,1)
        X[0:3,0] = T[0:3,3]
        X[3,0] = atan2(T0_EE_GAZEBO[2,1],T0_EE_GAZEBO[2,2]) #Roll
        X[4,0] = atan2(-T0_EE_GAZEBO[2,0], sqrt(T0_EE_GAZEBO[0,0]*T0_EE_GAZEBO[0,0] + T0_EE_GAZEBO[1,0]*T0_EE_GAZEBO[1,0])) #Pitch
        X[5,0] = atan2(T0_EE_GAZEBO[1,0],T0_EE_GAZEBO[0,0]) #YAW
        x=lambdify([q1,q2,q3,q4,q5,q6],X,"numpy")

        #Compute Analytical Jacobian
        J = zeros(6,6)
        J[:,0] = diff(X,q1)
        J[:,1] = diff(X,q2)
        J[:,2] = diff(X,q3)
        J[:,3] = diff(X,q4)
        J[:,4] = diff(X,q5)
        J[:,5] = diff(X,q6)
        j=lambdify([q1,q2,q3,q4,q5,q6],J,"numpy")

        #Initialize response
        joint_trajectory_list =[]

        for i in xrange(0, len(req.poses)):

            joint_trajectory_point = JointTrajectoryPoint()

            #Extract EE goal pose
            px = req.poses[i].position.x
            py = req.poses[i].position.y
            pz = req.poses[i].position.z

            (roll, pitch, yaw) =tf.transformations.euler_from_quaternion([req.poses[i].orientation.x,
                                                                          req.poses[i].orientation.y,
                                                                          req.poses[i].orientation.z,
                                                                          req.poses[i].orientation.w,])

            x_goal = np.array([[px,py,pz,roll,pitch,yaw]]).T

            #Initialize
            '''
            TODO: get theta values from Gazebo instead
            joint_state_msg = rospy.wait_for_message(topic, topic_type, timeout=None)
            '''
            q = np.array([[0,0,0,0,0,0]]).T
            x_ee=x(q[0],q[1],q[2],q[3],q[4],q[5]).astype(np.float64)
            x_ee=np.squeeze(x_ee, axis=2)

            #ompute Error
            delta_x=x_goal-x_ee
            delta_x[3]=( delta_x[3] + np.pi) % (2 * np.pi ) - np.pi
            delta_x[4]=( delta_x[4] + np.pi) % (2 * np.pi ) - np.pi
            delta_x[5]=( delta_x[5] + np.pi) % (2 * np.pi ) - np.pi

            while(np.sum(np.abs(delta_x)) > 0.01):

                ja=(j(q[0],q[1],q[2],q[3],q[4],q[5])).astype(np.float64)
                delta_q = 0.25*(ja.T).dot(delta_x)
                q=q+delta_q

                x_ee=x(q[0],q[1],q[2],q[3],q[4],q[5]).astype(np.float64)
                x_ee=np.squeeze(x_ee, axis=2)

                delta_x=x_goal-x_ee
                delta_x[3]=( delta_x[3] + np.pi) % (2 * np.pi ) - np.pi
                delta_x[4]=( delta_x[4] + np.pi) % (2 * np.pi ) - np.pi
                delta_x[5]=( delta_x[5] + np.pi) % (2 * np.pi ) - np.pi

                if (np.sum(np.abs(delta_x)) >= 10):
                    break

            #Populate response
            joint_trajectory_point.positions = [q[0]q[1],q[2],q[3],q[4],q[5]]
            joint_trajectory_list.append(joint_trajectory_point)

    return CalculateIKResponse(joint_trajectory_list)

def IK_server():
    rospy.init_node("IK_server_node")
    server=rospy.Service("calculate_ik",CalculateIK,handle_calculate_IK)
    print "Ready to receive an IK request"
    rospy.spin()

if __name__ == '__main__':
    IK_server()
