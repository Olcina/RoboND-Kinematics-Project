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
from kuka_arm.srv import *
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from geometry_msgs.msg import Pose
from mpmath import *
from sympy import *
from rot_xyz import rot_x, rot_y, rot_z

#Load TRANS_MATRIX from pkl
matrix_dict = pickle.load(open('T0_X-matrix.pkl','rb'))
T0_3 = matrix_dict['T0_3']
# rad to deg conversion
rad2deg = 180/np.pi

def handle_calculate_IK(req):
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

            # Define DH param symbols
            alpha0, alpha1, alpha2, alpha3, alpha4,alpha5, alpha6 = symbols('alpha0:7')
            a0,a1,a2,a3,a4,a5,a6 = symbols('a0:7')
            d1,d2,d3,d4,d5,d6,d7 = symbols('d1:8')
            tetha1, tetha2,tetha3, tetha4,tetha5, tetha6,tetha7 = symbols('tetha1:8')



            # Joint angle symbols
            #q1,q2,q3,q4,q5,q6 = symbols('q1:7')


            # Modified DH params
            s = {alpha0:    0 , a0:      0  , d1:  0.75 ,
                 alpha1:-pi/2 , a1:   0.35  , d2:     0 , tetha2: tetha2-pi/2,
                 alpha2:    0 , a2:   1.25  , d3:     0 ,
                 alpha3:-pi/2 , a3: -0.054  , d4:   1.5 ,
                 alpha4: pi/2 , a4:      0  , d5:     0 ,
                 alpha5:-pi/2 , a5:      0  , d6:     0 ,
                 alpha6:    0 , a6:      0  , d7: 0.303 , tetha7:       0,
            }


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
            r, p, y = symbols('r p y')
            
            R_X = rot_x(r)
            R_Y = rot_y(p)
            R_Z = rot_z(y)
            Rrpy = R_Z * R_Y * R_X

            Rot_Error = R_Z.subs(y, pi) * R_Y.subs(p, -pi/2.0)

            ROT_EE = Rrpy* Rot_Error

            ROT_EE = ROT_EE.subs({'r': roll, 'y': yaw, 'p': pitch})


            P_ee = Matrix([[px], [py],[pz]])

            R_x3 = Rrpy[0:3,2]
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
            
            R0_3 = T0_3[0:3,0:3]
            R0_3 = R0_3.subs(angles)

            R3_6 = R0_3.inv("LU") * ROT_EE
            #Comparison between UDACITY and my solution -- NEAR 0.0 In simulation
            print('q1 diff:', q1-q11)
            print('q1 diff:', q2-q22)
            print('q1 diff:', q3-q33)
                
                
            q4 = atan2(R3_6[2,2], -R3_6[0,2])
            q5 = atan2(sqrt(R3_6[0,2] * R3_6[0,2] + R3_6[2,2] * R3_6[2,2]), R3_6[1,2])
            q6 = atan2(-R3_6[1,1], R3_6[1,0])

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
