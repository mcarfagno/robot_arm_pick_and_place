#!/usr/bin/env python

import rospy
import tf
from ur5_arm.srv import *
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState
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

        #Gazebo to DH base link transform
        base_off = rospy.get_param("~base_position") # offset of the base relative to the world frame in gazebo
        R_BASE = R_z.subs('angle',pi)                # yaw offset of urdf base_link frame vs DH frame
        T_BASE = R_BASE.col_insert(3, Matrix([base_off[0],
                                             base_off[1],
                                             base_off[2]]))

        T_BASE = T_BASE.row_insert(3, Matrix([[0, 0, 0, 1]]))

        #DH to Gazebo EE orientation offset
        R_CORR = R_z.subs('angle',pi/2)*R_y.subs('angle',-pi/2)
        T_CORR = R_CORR.col_insert(3, Matrix([0, 0, 0]))
        T_CORR = T_CORR.row_insert(3, Matrix([[0, 0, 0, 1]]))

        T0_EE_GAZEBO = T_BASE*T0_EE*T_CORR

        # Compute EE Pose Vector -> RPY
        X = zeros(6,1)
        X[0:3,0] = T0_EE_GAZEBO[0:3,3]
        X[3,0] = atan2(T0_EE_GAZEBO[2,1],T0_EE_GAZEBO[2,2]) #Roll
        X[4,0] = atan2(-T0_EE_GAZEBO[2,0], sqrt(T0_EE_GAZEBO[0,0]*T0_EE_GAZEBO[0,0] + T0_EE_GAZEBO[1,0]*T0_EE_GAZEBO[1,0])) #Pitch
        X[5,0] = atan2(T0_EE_GAZEBO[1,0],T0_EE_GAZEBO[0,0]) #Yaw
        x=lambdify([q1,q2,q3,q4,q5,q6],X,"numpy")

        #Compute Analytical Jacobian
        J = zeros(6,6)
        # J[:,0] = diff(X,q1)
        # J[:,1] = diff(X,q2)
        # J[:,2] = diff(X,q3)
        # J[:,3] = diff(X,q4)
        # J[:,4] = diff(X,q5)
        # J[:,5] = diff(X,q6)
        J[:,0] = X.diff(q1)
        J[:,1] = X.diff(q2)
        J[:,2] = X.diff(q3)
        J[:,3] = X.diff(q4)
        J[:,4] = X.diff(q5)
        J[:,5] = X.diff(q6)
        j=lambdify([q1,q2,q3,q4,q5,q6],J,"numpy")

        # # Compute EE Pose Vector -> QUATERNIONS
        # X = zeros(7,1)
        # X[0:3,0] = T0_EE_GAZEBO[0:3,3]
        # X[3,0] = 0.5*sign(T0_EE_GAZEBO[2,1]-T0_EE_GAZEBO[1,2])*sqrt(T0_EE_GAZEBO[0,0]-T0_EE_GAZEBO[1,1]-T0_EE_GAZEBO[2,2]+1)  #qx
        # X[4,0] = 0.5*sign(T0_EE_GAZEBO[0,2]-T0_EE_GAZEBO[2,0])*sqrt(-T0_EE_GAZEBO[0,0]+T0_EE_GAZEBO[1,1]-T0_EE_GAZEBO[2,2]+1) #qy
        # X[5,0] = 0.5*sign(T0_EE_GAZEBO[1,0]-T0_EE_GAZEBO[0,1])*sqrt(-T0_EE_GAZEBO[0,0]-T0_EE_GAZEBO[1,1]+T0_EE_GAZEBO[2,2]+1) #qz
        # X[6,0] = 0.5*sqrt(T0_EE_GAZEBO[0,0]+T0_EE_GAZEBO[1,1]+T0_EE_GAZEBO[2,2]+1)                                            #w
        # x=lambdify([q1,q2,q3,q4,q5,q6],X,"numpy")

        # #Compute Analytical Jacobian
        # J = zeros(7,6)
        # J[:,0] = X.diff(q1)
        # J[:,1] = X.diff(q2)
        # J[:,2] = X.diff(q3)
        # J[:,3] = X.diff(q4)
        # J[:,4] = X.diff(q5)
        # J[:,5] = X.diff(q6)
        # j=lambdify([q1,q2,q3,q4,q5,q6],J,"numpy")

        #Initialize response
        joint_trajectory_list =[]

        for i in xrange(0, len(req.poses)):

            joint_trajectory_point = JointTrajectoryPoint()

            #Extract EE goal pose from request -> RPY
            px = req.poses[i].position.x
            py = req.poses[i].position.y
            pz = req.poses[i].position.z

            (roll, pitch, yaw) =tf.transformations.euler_from_quaternion([req.poses[i].orientation.x,
                                                                          req.poses[i].orientation.y,
                                                                          req.poses[i].orientation.z,
                                                                          req.poses[i].orientation.w,])

            x_goal = np.array([[px,py,pz,roll,pitch,yaw]]).T

            # #Extract EE goal pose from request -> QUATERNION
            # px = req.poses[i].position.x
            # py = req.poses[i].position.y
            # pz = req.poses[i].position.z
            # qx = req.poses[i].orientation.x
            # qy = req.poses[i].orientation.y
            # qz = req.poses[i].orientation.z
            # w = req.poses[i].orientation.w

            # x_goal = np.array([[px,py,pz,qx,qy,qz,w]]).T

            #IK Algorithm Params
            itr = 0
            l=rospy.get_param("~ik_solver/lambda",1.0)
            tolerance = rospy.get_param("~ik_solver/tolerance",0.01)
            max_iter = rospy.get_param("~ik_solver/max_iter",1000)

            # Allocate NP Arrays
            N=6 #number of joints
            M=6 #space variables
            q = np.empty(N).reshape(6,1) #Joints angles
            x_ee = np.empty(M).reshape(6,1) #EE Position 
            delta_x = np.empty(M).reshape(6,1) #EE Error
            ja=np.empty((M,N)) #Analitycal Jacobean

            #Initialize: get real joint states from topic else zeros
            try:
                joint_state_msg = rospy.wait_for_message(rospy.get_param("~js_topic","joint_states"), JointState, timeout=2)
                q[:,0] = joint_state_msg.position[0:6]
            except rospy.exceptions.ROSException as e:
                q[:,0] = [0,0,0,0,0,0]

            # q[].item() is used otherwise a np array is returned
            x_ee[:]=x(q[0].item(),
                    q[1].item(),
                    q[2].item(),
                    q[3].item(),
                    q[4].item(),
                    q[5].item()).astype(np.float64)

            #Compute EE Error
            for idx in range(M):
                delta_x[idx] = x_goal[idx] - x_ee[idx]
                
            delta_x[3] = ( delta_x[3] + np.pi) % (2 * np.pi ) - np.pi
            delta_x[4] = ( delta_x[4] + np.pi) % (2 * np.pi ) - np.pi
            delta_x[5] = ( delta_x[5] + np.pi) % (2 * np.pi ) - np.pi
            
            while(np.sum(np.abs(delta_x)) > tolerance):
                
                ja[:]=j(q[0].item(),
                        q[1].item(),
                        q[2].item(),
                        q[3].item(),
                        q[4].item(),
                        q[5].item()).astype(np.float64)

            
                q[:]=q+(ja.T).dot( np.linalg.inv(ja.dot(ja.T) + l**2*np.eye(6)) ).dot(delta_x)

                x_ee[:]=x(q[0].item(),
                        q[1].item(),
                        q[2].item(),
                        q[3].item(),
                        q[4].item(),
                        q[5].item()).astype(np.float64)
                
                for idx in range(M):
                    delta_x[idx] = x_goal[idx] - x_ee[idx]

                delta_x[3] = ( delta_x[3] + np.pi) % (2 * np.pi ) - np.pi
                delta_x[4] = ( delta_x[4] + np.pi) % (2 * np.pi ) - np.pi
                delta_x[5] = ( delta_x[5] + np.pi) % (2 * np.pi ) - np.pi
                
                itr+=1
                
                if itr>max_iter:
                    rospy.logwarn("IK Algorithm exceded number of allowed iterations")
                    break
                
                if itr%100 == 0:
                    rospy.loginfo("ik_error at iter {}: {}".format(itr,np.sum(np.abs(delta_x))))

                if (np.sum(np.abs(delta_x)) >= 10):
                    rospy.logerr("ik error is diverging!")
                    return rospy.ServiceException("Unable to find IK solution.")

            # Fill response msg
            joint_trajectory_point.positions = [q[0].item(),
                                                q[1].item(),
                                                q[2].item(),
                                                q[3].item(),
                                                q[4].item(),
                                                q[5].item()]

            joint_trajectory_list.append(joint_trajectory_point)
        
    rospy.loginfo("Success! Found {} IK Solutions!".format(len(joint_trajectory_list)))
    return CalculateIKResponse(joint_trajectory_list)

def IK_server():
    rospy.init_node("IK_server_node")
    server=rospy.Service("calculate_ik",CalculateIK,handle_calculate_IK)
    print "Ready to receive an IK request"
    rospy.spin()

if __name__ == '__main__':
    IK_server()
