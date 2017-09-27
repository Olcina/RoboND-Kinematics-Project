from sympy import *
from time import time
from mpmath import radians
import pickle
import tf
from WC_calculation import calculate_WC, calculate_EE
from DH_transform_matrix import rot_x, rot_y, rot_z
'''
Format of test case is [ [[EE position],[EE orientation as quaternions]],[WC location],[joint angles]]
You can generate additional test cases by setting up your kuka project and running `$ roslaunch kuka_arm forward_kinematics.launch`
From here you can adjust the joint angles to find thetas, use the gripper to extract positions and orientation (in quaternion xyzw) and lastly use link 5
to find the position of the wrist center. These newly generated test cases can be added to the test_cases dictionary.
'''
#load symbols for the DH parameters
alpha0, alpha1, alpha2, alpha3, alpha4,alpha5, alpha6 = symbols('alpha0:7')
a0,a1,a2,a3,a4,a5,a6 = symbols('a0:7')
d1,d2,d3,d4,d5,d6,d7 = symbols('d1:8')
tetha1, tetha2,tetha3, tetha4,tetha5, tetha6,tetha7 = symbols('tetha1:8')
#DH matrix
s = {alpha0:    0 , a0:      0  , d1:  0.75 ,
     alpha1:-pi/2 , a1:   0.35  , d2:     0 , tetha2: tetha2-pi/2,
     alpha2:    0 , a2:   1.25  , d3:     0 ,
     alpha3:-pi/2 , a3: -0.054  , d4:   1.5 ,
     alpha4: pi/2 , a4:      0  , d5:     0 ,
     alpha5:-pi/2 , a5:      0  , d6:     0 ,
     alpha6:    0 , a6:      0  , d7: 0.303 , tetha7:       0,
}

test_cases = {1:[[[2.16135,-1.42635,1.55109],
                  [0.708611,0.186356,-0.157931,0.661967]],
                  [1.89451,-1.44302,1.69366],
                  [-0.65,0.45,-0.36,0.95,0.79,0.49]],
              2:[[[-0.56754,0.93663,3.0038],
                  [0.62073, 0.48318,0.38759,0.480629]],
                  [-0.638,0.64198,2.9988],
                  [-0.79,-0.11,-2.33,1.94,1.14,-3.68]],
              3:[[[-1.3863,0.02074,0.90986],
                  [0.01735,-0.2179,0.9025,0.371016]],
                  [-1.1669,-0.17989,0.85137],
                  [-2.99,-0.12,0.94,4.06,1.29,-4.12]],
              4:[],
              5:[]}

#Load the transformation matrix from pickle object
matrix_dict = pickle.load(open('T0_X-matrix.pkl','rb'))
T0_3 = matrix_dict['T0_3']



def test_code(test_case):
    ## Set up code
    ## Do not modify!
    x = 0
    class Position:
        def __init__(self,EE_pos):
            self.x = EE_pos[0]
            self.y = EE_pos[1]
            self.z = EE_pos[2]
    class Orientation:
        def __init__(self,EE_ori):
            self.x = EE_ori[0]
            self.y = EE_ori[1]
            self.z = EE_ori[2]
            self.w = EE_ori[3]

    position = Position(test_case[0][0])
    orientation = Orientation(test_case[0][1])

    class Combine:
        def __init__(self,position,orientation):
            self.position = position
            self.orientation = orientation

    comb = Combine(position,orientation)

    class Pose:
        def __init__(self,comb):
            self.poses = [comb]

    req = Pose(comb)
    start_time = time()

    ########################################################################################
    ##
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

    ## Insert IK code here!
    #calculate q1
    q1 = atan2(WC_00[1],WC_00[0]).subs(s)
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
    q2 = pi/2 - betha1 - betha2
    q3 = pi/2 - betha3 + betha4

    #Calculate q4,q5,q6
    angles = {tetha1: q1,tetha2:q2, tetha3:q3}

    R0_3 = T0_3[0:3,0:3]
    R0_3 = R0_3.subs(angles)

    R3_6 = R0_3.inv("LU") * ROT_EE
    print(R3_6)

    q4 = atan2(R3_6[2,2], -R3_6[0,2])
    q5 = atan2(sqrt(R3_6[0,2] * R3_6[0,2] + R3_6[2,2] * R3_6[2,2]), R3_6[1,2])
    q6 = atan2(-R3_6[1,1], R3_6[1,0])

    theta1 = q1
    theta2 = q2
    theta3 = q3
    theta4 = q4
    theta5 = q5
    theta6 = q6

    ##
    ########################################################################################

    ########################################################################################
    ## For additional debugging add your forward kinematics here. Use your previously calculated thetas
    ## as the input and output the position of your end effector as your_ee = [x,y,z]

    ## (OPTIONAL) YOUR CODE HERE!
    #calculate_WC get the WC usingforward kinematics
    angles2_FK = {tetha1: q1,tetha2: q2, tetha3: q3,tetha4: q4,tetha5: q5,tetha6: q6}
    WC_FK = calculate_WC(angles2_FK)
    WC_FK = N(WC_FK)

    EE_FK = calculate_EE(angles2_FK)
    EE_FK = N(EE_FK)

    ## End your code input for forward kinematics here!
    ########################################################################################

    ## For error analysis please set the following variables of your WC location and EE location in the format of [x,y,z]
    your_wc = [WC_FK[0],WC_FK[1],WC_FK[2]] # <--- Load your calculated WC values in this array
    your_ee = [EE_FK[0],EE_FK[1],EE_FK[2]] # <--- Load your calculated end effector value from your forward kinematics
    ########################################################################################

    ## Error analysis
    print ("\nTotal run time to calculate joint angles from pose is %04.4f seconds" % (time()-start_time))

    # Find WC error
    if not(sum(your_wc)==3):
        wc_x_e = abs(your_wc[0]-test_case[1][0])
        wc_y_e = abs(your_wc[1]-test_case[1][1])
        wc_z_e = abs(your_wc[2]-test_case[1][2])
        wc_offset = sqrt(wc_x_e**2 + wc_y_e**2 + wc_z_e**2)
        print ("\nWrist error for x position is: %04.8f" % wc_x_e)
        print ("Wrist error for y position is: %04.8f" % wc_y_e)
        print ("Wrist error for z position is: %04.8f" % wc_z_e)
        print ("Overall wrist offset is: %04.8f units" % wc_offset)

    # Find theta errors
    t_1_e = abs(theta1-test_case[2][0])
    t_2_e = abs(theta2-test_case[2][1])
    t_3_e = abs(theta3-test_case[2][2])
    t_4_e = abs(theta4-test_case[2][3])
    t_5_e = abs(theta5-test_case[2][4])
    t_6_e = abs(theta6-test_case[2][5])
    print ("\nTheta 1 error is: %04.8f" % t_1_e)
    print ("Theta 2 error is: %04.8f" % t_2_e)
    print ("Theta 3 error is: %04.8f" % t_3_e)
    print ("Theta 4 error is: %04.8f" % t_4_e)
    print ("Theta 5 error is: %04.8f" % t_5_e)
    print ("Theta 6 error is: %04.8f" % t_6_e)
    print ("\n**These theta errors may not be a correct representation of your code, due to the fact \
           \nthat the arm can have muliple positions. It is best to add your forward kinmeatics to \
           \nconfirm whether your code is working or not**")
    print (" ")

    # Find FK EE error
    if not(sum(your_ee)==3):
        ee_x_e = abs(your_ee[0]-test_case[0][0][0])
        ee_y_e = abs(your_ee[1]-test_case[0][0][1])
        ee_z_e = abs(your_ee[2]-test_case[0][0][2])
        ee_offset = sqrt(ee_x_e**2 + ee_y_e**2 + ee_z_e**2)
        print ("\nEnd effector error for x position is: %04.8f" % ee_x_e)
        print ("End effector error for y position is: %04.8f" % ee_y_e)
        print ("End effector error for z position is: %04.8f" % ee_z_e)
        print ("Overall end effector offset is: %04.8f units \n" % ee_offset)




if __name__ == "__main__":
    # Change test case number for different scenarios
    test_case_number = 3

    test_code(test_cases[test_case_number])
