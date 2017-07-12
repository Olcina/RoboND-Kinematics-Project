import numpy as np
import pickle
from sympy import symbols, cos, sin, pi, sqrt, simplify, symbol, atan2, acos, N
from sympy.matrices import Matrix
from DH_transform_matrix import get_DH_transform_matrix
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from WC_calculation import calculate_WC

def rot_x(q):
    R_x = Matrix([[ 1,              0,        0],
                  [ 0,        cos(q), -sin(q)],
                  [ 0,        sin(q),  cos(q)]])
    return R_x

def rot_y(q):
    R_y = Matrix([[ cos(q),        0,  sin(q)],
                  [       0,        1,        0],
                  [-sin(q),        0,  cos(q)]])
    return R_y

def rot_z(q):
    R_z = Matrix([[ cos(q), -sin(q),        0],
                  [ sin(q),  cos(q),        0],
                  [ 0,              0,        1]])
    return R_z
rad2deg = 180/np.pi
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
#Load the transformation matrix
matrix_dict = pickle.load(open('T0_X-matrix.pkl','rb'))


#########################################################
NUMBER_OF_POINTS_TO_DRAW = 8
#Define angles
angles = {tetha1: 0,tetha2:pi/8, tetha3:pi/8,tetha4:0,tetha5:pi/2,tetha6:pi/4}
#calculate_WC get the WC usingforward kinematics
WC_00 = calculate_WC(angles)
WC_00 = N(WC_00)
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
#Define final angles for the Inverse Kinematics

#create the figure
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
#calculate points wiht the forward kinematics
point_0 = Matrix([0,0,0,1])
point_1 = matrix_dict['T0_1'].subs(angles)*point_0
point_2 = matrix_dict['T0_2'].subs(angles)*point_0
point_3 = matrix_dict['T0_3'].subs(angles)*point_0
point_4 = matrix_dict['T0_4'].subs(angles)*point_0
point_5 = matrix_dict['T0_5'].subs(angles)*point_0
point_6 = matrix_dict['T0_6'].subs(angles)*point_0
point_ee = matrix_dict['T0_G'].subs(angles)*point_0

calculated_points = []
calculated_points.append(point_0)
calculated_points.append(point_1)
calculated_points.append(point_2)
calculated_points.append(point_3)
calculated_points.append(point_4)
calculated_points.append(point_5)
calculated_points.append(point_6)
calculated_points.append(point_ee)
pre_point = []
x = NUMBER_OF_POINTS_TO_DRAW
for i in range(0, x):
    point = calculated_points[i]
    ax.scatter(point[0],point[1],point[2],color = 'g')
    if i < 4 or i > 6:
        ax.text(point[0],point[1],point[2],  '%s' % (str(i)), size=20, zorder=1, color='k')
    else:
        pass
        # ax.text(point[0],point[1],point[2],  '%s' % ('WC'), size=20, zorder=1, color='k')
    if pre_point != []:
        ax.plot([pre_point[0],point[0]],[pre_point[1],point[1]],[pre_point[2],point[2]])
    pre_point = point

#Define final angles for the Inverse Kinematics
cal_angles = {tetha1:q1,tetha2:N(q2,5), tetha3:N(q3,5),tetha4:0,tetha5:0,tetha6:0}
#create the figure
# fig = plt.figure()
# ax = fig.add_subplot(111, projection='3d')
#calculate points wiht the forward kinematics
point_0 = Matrix([0,0,0,1])
point_1 = matrix_dict['T0_1'].subs(cal_angles)*point_0
point_2 = matrix_dict['T0_2'].subs(cal_angles)*point_0
point_3 = matrix_dict['T0_3'].subs(cal_angles)*point_0
point_4 = matrix_dict['T0_4'].subs(cal_angles)*point_0
point_5 = matrix_dict['T0_5'].subs(cal_angles)*point_0
point_6 = matrix_dict['T0_6'].subs(cal_angles)*point_0
point_ee = matrix_dict['T0_G'].subs(cal_angles)*point_0
calculated_points = []
calculated_points.append(point_0)
calculated_points.append(point_1)
calculated_points.append(point_2)
calculated_points.append(point_3)
calculated_points.append(point_4)
calculated_points.append(point_5)
calculated_points.append(point_6)
calculated_points.append(point_ee)
pre_point = []
x = NUMBER_OF_POINTS_TO_DRAW
for i in range(0, x):
    point = calculated_points[i]
    ax.scatter(point[0],point[1],point[2],color = 'y')
    if i < 4 or i > 6:
        ax.text(point[0],point[1],point[2],  '%s' % (str(i)), size=20, zorder=1, color='k')
    else:
        pass
        # ax.text(point[0],point[1],point[2],  '%s' % ('WC'), size=20, zorder=1, color='k')
    if pre_point != []:
        ax.plot([pre_point[0],point[0]],[pre_point[1],point[1]],[pre_point[2],point[2]])
    pre_point = point

print('***** AUX TRIANGLE *****')
print('a,b,c =',a,b,c)
print('betha1 = ',betha1)
print('betha2 = ',betha2)
print('betha3 = ',betha3)
print('betha4 = ',betha4)
print('***** q1, q2, q3 ******')
print('q1 = ', q1,' in deg = ', q1*rad2deg)
print('q2 = ', q2,' in deg = ', q2*rad2deg)
print('q3 = ', q3,' in deg = ', q3*rad2deg)
print('***********************')
print('configuration_angles = ', angles)
print('calculate_angles = ', cal_angles)
print('forward kinematics WC = ',N(WC_00[0],5),N(WC_00[1],5),N(WC_00[2],5))
print('inverse kinematics WC = ',N(point_4[0],5),N(point_4[1],5),N(point_4[2],5))
print('total error = ', sum(i-j for i,j in zip(WC_00,point_4)))
plt.show()



#get the pose
px, py, pz, roll, pitch, yaw = 2.153, 0.000, 1.947,0,0,0
#define final positions
Rrpy = rot_x(roll)*rot_y(pitch)*rot_z(yaw)

P_ee = Matrix([px,py,pz])


R_x3 = Rrpy[0:3,2]
print('P_ee = ',P_ee)
print('R_x3 = ',R_x3)
#calculate the Wrist position
#
# WC_00 = P_ee - d7*R_x3
# WC_00 = WC_00.subs(s)
