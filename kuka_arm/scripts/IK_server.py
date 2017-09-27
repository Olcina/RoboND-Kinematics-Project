#!/usr/bin/env python

# Copyright (C) 2017 Electric Movement Inc.
#
# This file is part of Robotic Arm: Pick and Place project for Udacity
# Robotics nano-degree program
#
# All Rights Reserved.

# Author: Harsh Pandya

# import modules
import pickle
import numpy as np
import rospy
import tf
import os
from kuka_arm.srv import *
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from geometry_msgs.msg import Pose
from mpmath import *
from sympy import *
from rot_xyz import rot_x, rot_y, rot_z
from DH_transform_matrix import get_DH_transform_matrix


def handle_calculate_IK(req):
    #Load TRANS_MATRIX from pkl
    # rad to deg conversion
    rad2deg = 180/np.pi

    # Define DH param symbols
    alpha0, alpha1, alpha2, alpha3, alpha4,alpha5, alpha6 = symbols('alpha0:7')
    a0,a1,a2,a3,a4,a5,a6 = symbols('a0:7')
    d1,d2,d3,d4,d5,d6,d7 = symbols('d1:8')
    tetha1, tetha2,tetha3, tetha4,tetha5, tetha6,tetha7 = symbols('tetha1:8')


    # Modified DH params
    s = {alpha0:    0 , a0:      0  , d1:  0.75 ,
         alpha1:-pi/2 , a1:   0.35  , d2:     0 , tetha2: tetha2-pi/2,
         alpha2:    0 , a2:   1.25  , d3:     0 ,
         alpha3:-pi/2 , a3: -0.054  , d4:   1.5 ,
         alpha4: pi/2 , a4:      0  , d5:     0 ,
         alpha5:-pi/2 , a5:      0  , d6:     0 ,
         alpha6:    0 , a6:      0  , d7: 0.303 , tetha7:       0,
    }
    #roll, pitch, yaw symbols
    r, p, y = symbols('r p y')

    #Load or dump TRANS_MATRIX from pkl
    #T0_1
    if not os.path.exists('T0_1.p'):
        T0_1 = get_DH_transform_matrix(alpha0,a0,d1,tetha1).subs(s)
        pickle.dump(T0_1,open('T0_1.p','wb'))
    else:
        T0_1 = pickle.load(open('T0_1.p','rb'))        
    #T1_2

    if not os.path.exists('T1_2.p'):
        T1_2 = get_DH_transform_matrix(alpha1,a1,d2,tetha2).subs(s)
        pickle.dump(T0_1,open('T1_2.p','wb'))
    else:
        T1_2 = pickle.load(open('T1_2.p','rb'))        

    #T0_3
    if not os.path.exists('T2_3.p'):
        T2_3 = get_DH_transform_matrix(alpha2,a2,d3,tetha3).subs(s)
        pickle.dump(T2_3,open('T2_3.p','wb'))
    else:
        T2_3 = pickle.load(open('T2_3.p','rb'))
        
    
    #T0_3
    if not os.path.exists('T0_3.p'):
        T0_3 = T0_1 * T1_2 * T2_3
        pickle.dump(T0_3,open('T0_3.p','wb'))
    else:
        T0_3 = pickle.load(open('T0_3.p','rb'))
    

    Rot_Error = rot_z(pi)*rot_y(-pi/2)

    R_X = rot_x(r)
    R_Y = rot_y(p)
    R_Z = rot_z(y)

    Rrpy = R_Z * R_Y * R_X * Rot_Error
    
     
    R0_3 = T0_3[0:3,0:3]
    R3_6 = R0_3**(-1) * Rrpy
    rospy.loginfo("Received %s eef-poses from the plan" % len(req.poses))
    if len(req.poses) < 1:
        print "No valid poses received"
        return -1
    else:
        # Initialize service response
        joint_trajectory_list = []
        for x in xrange(0, len(req.poses)):
            # IK code starts here
            joint_trajectory_point = JointTrajectoryPoint()

            # Define Modified DH Transformation matrix
                # defined in: ~/RoboND-Kinematics-Project/DH_transform_matrix.py

            # Create individual transformation matrices
                # defined in: ~/RoboND-Kinematics-Project/forward_kinematics.py

            # Extract end-effector position and orientation from request
	        # px,py,pz = end-effector position
	        # roll, pitch, yaw = end-effector orientation
            px = req.poses[x].position.x
            py = req.poses[x].position.y
            pz = req.poses[x].position.z

            (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(
                [req.poses[x].orientation.x, req.poses[x].orientation.y,
                    req.poses[x].orientation.z, req.poses[x].orientation.w])

            # Calculate joint angles using Geometric IK method


            ROT_EE = Rrpy.subs({'r': roll, 'y': yaw, 'p': pitch})


            P_ee = Matrix([[px], [py],[pz]])

            #R_x3 = Rrpy[0:3,2]
            #print('P_ee = ',P_ee)
            #print('R_x3 = ',R_x3)
            #calculte the Wrist position

            WC_00 = P_ee - s[d7] * ROT_EE[:,2]
            WC_00 = WC_00.subs(s)
            ### MY CODE FOR q1, q2, q3

            #calculate q1
            q11 = atan2(WC_00[1],WC_00[0]).subs(s)
            #first need the position of the WC in the 0_2 frame
            WC_z_1 = WC_00[2]-s[d1]
            WC_x_1 = sqrt(WC_00[0]**2 + WC_00[1]**2) - s[a1]
            WC_y_1 = 0
            # aux triangle
            a = sqrt(WC_x_1**2 + WC_z_1**2)
            b = 1.25
            c = sqrt(1.5**2 + 0.054**2)
            # Aux angles
            betha1 = atan2(WC_z_1,WC_x_1)
            betha2 = acos((-c**2 + a**2 + b**2)/(2*a*b))
            betha3 = acos((b**2 + c**2 - a**2)/(2*b*c))
            betha4 = atan2(s[a3],s[d4])
            #q2, q3
            q22 = pi/2 - betha1 - betha2
            q33 = pi/2 - betha3 + betha4


	        ## UDACITY CODE for q1, q2, q3
            q1 = atan2(WC_00[1],WC_00[0])
            side_a =  1.501
            side_b = sqrt(pow((sqrt(WC_00[0]*WC_00[0] + WC_00[1]*WC_00[1]) - 0.35),2) + pow((WC_00[2]-0.75),2))
            side_c = 1.25

            angle_a = acos((side_b * side_b + side_c * side_c - side_a * side_a) / (2 * side_b * side_c))
            angle_b = acos((side_a * side_a + side_c * side_c - side_b * side_b) / (2 * side_a * side_c))
            angle_c = acos((side_a * side_a + side_b * side_b - side_c * side_c) / (2 * side_a * side_b))

            q2 = pi /2 - angle_a - atan2(WC_00[2] - 0.75, sqrt(WC_00[0] * WC_00[0] + WC_00[1] * WC_00[1]) - 0.35)
            q3 = pi /2 - (angle_b + 0.036)

            #Calculate q4,q5,q6
            angles = {tetha1: q1,tetha2:q2, tetha3:q3}

            R3_6_to_num = R3_6.subs({tetha1: q1,tetha2:q2, tetha3:q3, r: roll, y: yaw, p: pitch})
            #Comparison between UDACITY and my solution -- NEAR 0.0 In simulation
            # print('q1 diff:', q1-q11)
            # print('q1 diff:', q2-q22)
            # print('q1 diff:', q3-q33)
            r13 = R3_6_to_num[0,2]
            r33 = R3_6_to_num[2,2]
            r22 = R3_6_to_num[1,1]
            r21 = R3_6_to_num[1,0]
            r23 = R3_6_to_num[1,2]

            q5 = atan2(sqrt(r13**2 + r33**2),r23)
            if sin(q5) < 0:
                q4 = atan2(-r33, r13)
                q6 = atan2(r22, -r21)
            else:
                q4 = atan2(r33, -r13)
                q6 = atan2(-r22, r21)
            #q4 = atan2(R3_6[2,2], -R3_6[0,2])
            #q5 = atan2(sqrt(R3_6[0,2] * R3_6[0,2] + R3_6[2,2] * R3_6[2,2]), R3_6[1,2])
            #q6 = atan2(-R3_6[1,1], R3_6[1,0])

            # Populate response for the IK request
            # In the next line replace theta1,theta2...,theta6 by your joint angle variables
            joint_trajectory_point.positions = [q1, q2, q3, q4, q5, q6]
            joint_trajectory_list.append(joint_trajectory_point)

        rospy.loginfo("length of Joint Trajectory List: %s" % len(joint_trajectory_list))
        return CalculateIKResponse(joint_trajectory_list)


def IK_server():
    # initialize node and declare calculate_ik service
    rospy.init_node('IK_server')
    s = rospy.Service('calculate_ik', CalculateIK, handle_calculate_IK)
    print "Ready to receive an IK request"
    rospy.spin()

if __name__ == "__main__":
    IK_server()
